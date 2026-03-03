#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <mutex>

class Teleoperation : public rclcpp::Node {
public:
  Teleoperation()
  : Node("teleop_node")
  {
    // ── Drive / Turret params ─────────────────────────────────────────────
    joy_topic_        = this->declare_parameter<std::string>("joy_topic", "/joy");
    axis_ly_          = this->declare_parameter<int>("axis_ly", 1);
    axis_rx_          = this->declare_parameter<int>("axis_rx", 3);
    axis_lt_          = this->declare_parameter<int>("axis_lt", -1);
    axis_rt_          = this->declare_parameter<int>("axis_rt", -1);
    btn_lt_           = this->declare_parameter<int>("btn_lt", -1);
    btn_rt_           = this->declare_parameter<int>("btn_rt", -1);
    invert_ly_        = this->declare_parameter<bool>("invert_ly", false);
    invert_rx_        = this->declare_parameter<bool>("invert_rx", true);
    invert_lt_        = this->declare_parameter<bool>("invert_lt", false);
    invert_rt_        = this->declare_parameter<bool>("invert_rt", false);
    deadman_button_   = this->declare_parameter<int>("deadman_button", -1);
    deadband_         = this->declare_parameter<double>("deadband", 0.08);
    mix_scale_        = this->declare_parameter<double>("mix_scale", 1.0);
    max_rpm_cmd_      = this->declare_parameter<double>("max_rpm_cmd", 30.0);
    turret_max_rad_s_ = this->declare_parameter<double>("turret_max_rad_s", 2.0);
    publish_rate_hz_  = this->declare_parameter<double>("publish_rate_hz", 100.0);
    stale_timeout_s_  = this->declare_parameter<double>("stale_timeout_s", 0.5);
    ema_tau_s_        = this->declare_parameter<double>("ema_tau_s", 0.05);

    // ── Actuator + Stepper direction buttons ──────────────────────────────
    btn_act_extend_   = this->declare_parameter<int>("btn_act_extend",  -1);
    btn_act_retract_  = this->declare_parameter<int>("btn_act_retract", -1);
    btn_step_cw_      = this->declare_parameter<int>("btn_step_cw",  -1);
    btn_step_ccw_     = this->declare_parameter<int>("btn_step_ccw", -1);

    // ── Stepper speed buttons ─────────────────────────────────────────────
    btn_speed_up_     = this->declare_parameter<int>("btn_speed_up",   -1);
    btn_speed_down_   = this->declare_parameter<int>("btn_speed_down", -1);
    stepper_speed_min_  = this->declare_parameter<int>("stepper_speed_min",  300);
    stepper_speed_max_  = this->declare_parameter<int>("stepper_speed_max", 3000);
    stepper_speed_step_ = this->declare_parameter<int>("stepper_speed_step", 300);
    stepper_speed_init_ = this->declare_parameter<int>("stepper_speed_init", 500);
    stepper_speed_ = stepper_speed_init_;

    // ── Derived ───────────────────────────────────────────────────────────
    const double RPM_TO_RAD_S = 2.0 * M_PI / 60.0;
    max_rad_s_ = max_rpm_cmd_ * RPM_TO_RAD_S;

    // ── Publishers ────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(1).best_effort();
    pub_left_          = this->create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  qos);
    pub_right_         = this->create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", qos);
    pub_turret_        = this->create_publisher<std_msgs::msg::Float64>("/turret/cmd_vel",      qos);
    pub_actuator_      = this->create_publisher<std_msgs::msg::Int32>  ("/actuator/cmd",        qos);
    pub_stepper_       = this->create_publisher<std_msgs::msg::Int32>  ("/stepper/cmd",         qos);
    pub_stepper_speed_ = this->create_publisher<std_msgs::msg::Int32>  ("/stepper/speed",       qos);

    // ── Joy subscriber ────────────────────────────────────────────────────
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        axes_    = msg->axes;
        buttons_ = msg->buttons;
        last_msg_time_ = this->now();
      });

    // ── Timer ─────────────────────────────────────────────────────────────
    timer_ = this->create_wall_timer(
      std::chrono::microseconds((int)(1e6 / std::max(1.0, publish_rate_hz_))),
      std::bind(&Teleoperation::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "teleop_node ready  joy=%s  max_rpm=%.1f  "
      "act=[%d/%d]  step=[%d/%d]  speed_btn=[%d/%d]  speed_init=%d",
      joy_topic_.c_str(), max_rpm_cmd_,
      btn_act_extend_, btn_act_retract_,
      btn_step_cw_,    btn_step_ccw_,
      btn_speed_up_,   btn_speed_down_,
      stepper_speed_);
  }

private:
  inline float shape(float x, bool invert) const {
    if (invert) x = -x;
    if (std::fabs(x) < deadband_) return 0.0f;
    float s = (std::fabs(x) - (float)deadband_) / (1.0f - (float)deadband_);
    return std::copysign(std::clamp(s, 0.0f, 1.0f), x);
  }

  inline int btn(const std::vector<int32_t>& b, int i) const {
    return (i >= 0 && i < (int)b.size()) ? b[i] : 0;
  }

  void on_timer() {
    std::vector<float>   axes;
    std::vector<int32_t> btns;
    rclcpp::Time         last;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      axes = axes_; btns = buttons_; last = last_msg_time_;
    }

    const auto now = this->now();
    bool ok = (now - last).seconds() < stale_timeout_s_;
    if (ok && deadman_button_ >= 0)
      ok = btn(btns, deadman_button_);

    // ── Drive ─────────────────────────────────────────────────────────────
    float ly = 0, rx = 0, lt = 0, rt = 0;
    if (ok) {
      if (axis_ly_ >= 0 && axis_ly_ < (int)axes.size()) ly = shape(axes[axis_ly_], invert_ly_);
      if (axis_rx_ >= 0 && axis_rx_ < (int)axes.size()) rx = shape(axes[axis_rx_], invert_rx_);
      if (axis_lt_ >= 0 && axis_lt_ < (int)axes.size()) lt = shape(axes[axis_lt_], invert_lt_);
      if (axis_rt_ >= 0 && axis_rt_ < (int)axes.size()) rt = shape(axes[axis_rt_], invert_rt_);
      if (btn_lt_ >= 0 && btn(btns, btn_lt_)) lt = 1.0f;
      if (btn_rt_ >= 0 && btn(btns, btn_rt_)) rt = 1.0f;
    }

    const double left_cmd   = (ly * mix_scale_ + rx * mix_scale_) * max_rad_s_;
    const double right_cmd  = (ly * mix_scale_ - rx * mix_scale_) * max_rad_s_;
    const double turret_cmd = (double)(rt - lt) * turret_max_rad_s_;

    const double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    const double a  = (ema_tau_s_ > 1e-4) ? dt / (ema_tau_s_ + dt) : 1.0;
    left_f_   += a * (left_cmd   - left_f_);
    right_f_  += a * (right_cmd  - right_f_);
    turret_f_ += a * (turret_cmd - turret_f_);

    std_msgs::msg::Float64 fm;
    fm.data = left_f_;   pub_left_->publish(fm);
    fm.data = right_f_;  pub_right_->publish(fm);
    fm.data = turret_f_; pub_turret_->publish(fm);

    // ── Actuator ──────────────────────────────────────────────────────────
    {
      int32_t cmd = 0;
      if (ok) {
        if      (btn(btns, btn_act_extend_))  cmd =  1;
        else if (btn(btns, btn_act_retract_)) cmd = -1;
      }
      if (cmd != last_act_cmd_) {
        std_msgs::msg::Int32 m; m.data = cmd;
        pub_actuator_->publish(m);
        last_act_cmd_ = cmd;
      }
    }

    // ── Stepper direction ─────────────────────────────────────────────────
    {
      int32_t cmd = 0;
      if (ok) {
        if      (btn(btns, btn_step_cw_))  cmd =  1;
        else if (btn(btns, btn_step_ccw_)) cmd = -1;
      }
      if (cmd != last_step_cmd_) {
        std_msgs::msg::Int32 m; m.data = cmd;
        pub_stepper_->publish(m);
        last_step_cmd_ = cmd;
      }
    }

    // ── Stepper speed (edge-triggered — tap, not hold) ────────────────────
    if (ok) {
      bool sup = btn(btns, btn_speed_up_);
      bool sdn = btn(btns, btn_speed_down_);

      if (sup && !last_speed_up_) {
        stepper_speed_ = std::min(stepper_speed_ + stepper_speed_step_, stepper_speed_max_);
        std_msgs::msg::Int32 m; m.data = stepper_speed_;
        pub_stepper_speed_->publish(m);
        RCLCPP_INFO(get_logger(), "Stepper speed -> %d steps/sec", stepper_speed_);
      }
      if (sdn && !last_speed_down_) {
        stepper_speed_ = std::max(stepper_speed_ - stepper_speed_step_, stepper_speed_min_);
        std_msgs::msg::Int32 m; m.data = stepper_speed_;
        pub_stepper_speed_->publish(m);
        RCLCPP_INFO(get_logger(), "Stepper speed -> %d steps/sec", stepper_speed_);
      }

      last_speed_up_   = sup;
      last_speed_down_ = sdn;
    } else {
      last_speed_up_ = last_speed_down_ = false;
    }
  }

private:
  // Drive params
  std::string joy_topic_;
  int  axis_ly_, axis_rx_, axis_lt_, axis_rt_;
  int  btn_lt_, btn_rt_, deadman_button_;
  bool invert_ly_, invert_rx_, invert_lt_, invert_rt_;
  double deadband_, mix_scale_, max_rpm_cmd_, max_rad_s_;
  double turret_max_rad_s_, publish_rate_hz_, stale_timeout_s_, ema_tau_s_;

  // Actuator / stepper direction params
  int btn_act_extend_, btn_act_retract_;
  int btn_step_cw_,    btn_step_ccw_;

  // Stepper speed params
  int btn_speed_up_,     btn_speed_down_;
  int stepper_speed_min_, stepper_speed_max_, stepper_speed_step_, stepper_speed_init_;

  // State
  std::mutex           mtx_;
  std::vector<float>   axes_;
  std::vector<int32_t> buttons_;
  rclcpp::Time         last_msg_time_{0, 0, RCL_ROS_TIME};

  double left_f_{0}, right_f_{0}, turret_f_{0};

  int32_t last_act_cmd_{0};
  int32_t last_step_cmd_{0};
  int     stepper_speed_{500};

  bool last_speed_up_{false};
  bool last_speed_down_{false};

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_, pub_right_, pub_turret_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   pub_actuator_, pub_stepper_, pub_stepper_speed_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleoperation>());
  rclcpp::shutdown();
  return 0;
}
