#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <atomic>
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
    // ----- Parameters -----
    joy_topic_        = this->declare_parameter<std::string>("joy_topic", "/joy");

    axis_ly_          = this->declare_parameter<int>("axis_ly", 1);   // Left stick Y
    axis_rx_          = this->declare_parameter<int>("axis_rx", 3);   // Right stick X

    // Triggers (many controllers map these differently; Pro Controller often uses buttons)
    axis_lt_          = this->declare_parameter<int>("axis_lt", -1);  
    axis_rt_          = this->declare_parameter<int>("axis_rt", -1);  
    btn_lt_           = this->declare_parameter<int>("btn_lt", -1);   // button mapping
    btn_rt_           = this->declare_parameter<int>("btn_rt", -1);

    invert_ly_        = this->declare_parameter<bool>("invert_ly", false);
    invert_rx_        = this->declare_parameter<bool>("invert_rx", true);
    invert_lt_        = this->declare_parameter<bool>("invert_lt", false);
    invert_rt_        = this->declare_parameter<bool>("invert_rt", false);

    deadman_button_   = this->declare_parameter<int>("deadman_button", -1); // -1 disables
    deadband_         = this->declare_parameter<double>("deadband", 0.08);
    mix_scale_        = this->declare_parameter<double>("mix_scale", 1.0);
    max_rpm_cmd_      = this->declare_parameter<double>("max_rpm_cmd", 30.0);
    turret_max_rad_s_ = this->declare_parameter<double>("turret_max_rad_s", 2.0);
    publish_rate_hz_  = this->declare_parameter<double>("publish_rate_hz", 100.0);
    stale_timeout_s_  = this->declare_parameter<double>("stale_timeout_s", 0.5);
    ema_tau_s_        = this->declare_parameter<double>("ema_tau_s", 0.05);

    // Convert max RPM to rad/s
    const double RPM_TO_RAD_S = 2.0 * M_PI / 60.0;
    max_rad_s_ = max_rpm_cmd_ * RPM_TO_RAD_S;

    // ----- Publishers -----
    auto qos = rclcpp::QoS(1).best_effort();
    pub_left_   = this->create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  qos);
    pub_right_  = this->create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", qos);
    pub_turret_ = this->create_publisher<std_msgs::msg::Float64>("/turret/cmd_vel",      qos);

    // ----- Joy subscriber -----
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        axes_ = msg->axes;
        buttons_ = msg->buttons;
        last_msg_time_ = this->now();
      });

    // ----- Timer to publish at fixed rate -----
    timer_ = this->create_wall_timer(
      std::chrono::microseconds((int)(1e6 / std::max(1.0, publish_rate_hz_))),
      std::bind(&Teleoperation::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
      "teleop_node ready (joy_topic=%s, max_rpm=%.1f -> max_rad_s=%.3f)",
      joy_topic_.c_str(), max_rpm_cmd_, max_rad_s_);
  }

private:
  inline float shape(float x, bool invert) const {
    if (invert) x = -x;
    if (std::fabs(x) < deadband_) return 0.0f;

    float s = (std::fabs(x) - (float)deadband_) / (1.0f - (float)deadband_);
    s = std::clamp(s, 0.0f, 1.0f);
    return std::copysign(s, x);
  }

  void on_timer() {
    // Copy latest inputs under lock
    std::vector<float> axes;
    std::vector<int32_t> btns;
    rclcpp::Time last;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      axes = axes_;
      btns = buttons_;
      last = last_msg_time_;
    }

    // Stale/Deadman gating
    bool ok = false;
    const auto now = this->now();
    if ((now - last).seconds() < stale_timeout_s_) {
      if (deadman_button_ < 0 ||
          (deadman_button_ < (int)btns.size() && btns[deadman_button_])) {
        ok = true;
      }
    }

    float ly = 0.f, rx = 0.f;

    // Triggers can be axes OR buttons; we support both.
    float lt = 0.f, rt = 0.f;

    if (ok) {
      if (axis_ly_ >= 0 && axis_ly_ < (int)axes.size()) ly = shape(axes[axis_ly_], invert_ly_);
      if (axis_rx_ >= 0 && axis_rx_ < (int)axes.size()) rx = shape(axes[axis_rx_], invert_rx_);

      if (axis_lt_ >= 0 && axis_lt_ < (int)axes.size()) lt = shape(axes[axis_lt_], invert_lt_);
      if (axis_rt_ >= 0 && axis_rt_ < (int)axes.size()) rt = shape(axes[axis_rt_], invert_rt_);

      // If triggers are buttons, map to 0/1
      if (btn_lt_ >= 0 && btn_lt_ < (int)btns.size() && btns[btn_lt_]) lt = 1.0f;
      if (btn_rt_ >= 0 && btn_rt_ < (int)btns.size() && btns[btn_rt_]) rt = 1.0f;
    }

    // Mix
    const float forward = ly * (float)mix_scale_;
    const float turn    = rx * (float)mix_scale_;

    const double left_cmd  = (forward + turn) * max_rad_s_;
    const double right_cmd = (forward - turn) * max_rad_s_;

    // Turret: RT - LT
    const double turret_cmd = (double)(rt - lt) * turret_max_rad_s_;

    // EMA smoothing
    const double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    const double a  = (ema_tau_s_ > 1e-4) ? dt / (ema_tau_s_ + dt) : 1.0;

    left_f_   += a * (left_cmd   - left_f_);
    right_f_  += a * (right_cmd  - right_f_);
    turret_f_ += a * (turret_cmd - turret_f_);

    // Publish
    std_msgs::msg::Float64 m;
    m.data = left_f_;   pub_left_->publish(m);
    m.data = right_f_;  pub_right_->publish(m);
    m.data = turret_f_; pub_turret_->publish(m);
  }

private:
  // Params
  std::string joy_topic_;
  int axis_ly_, axis_rx_;
  int axis_lt_, axis_rt_;
  int btn_lt_, btn_rt_;
  int deadman_button_;
  bool invert_ly_, invert_rx_, invert_lt_, invert_rt_;
  double deadband_, mix_scale_, max_rpm_cmd_, max_rad_s_, turret_max_rad_s_;
  double publish_rate_hz_, stale_timeout_s_, ema_tau_s_;

  // Latest /joy state
  std::mutex mtx_;
  std::vector<float> axes_;
  std::vector<int32_t> buttons_;
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};

  // Smoothed outputs
  double left_f_{0.0}, right_f_{0.0}, turret_f_{0.0};

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_, pub_right_, pub_turret_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleoperation>());
  rclcpp::shutdown();
  return 0;
}
