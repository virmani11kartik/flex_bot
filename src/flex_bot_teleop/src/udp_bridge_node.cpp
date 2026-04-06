#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

static inline double radps_to_rpm(double rad_s) {
  return rad_s * 60.0 / (2.0 * M_PI);
}
static inline double rpm_to_radps(double rpm) {
  return rpm * (2.0 * M_PI) / 60.0;
}

// -------------------- IMU helpers --------------------
static uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

// Convert RPY (rad) -> quaternion
static inline void rpy_to_quat(double roll, double pitch, double yaw,
                               double &qx, double &qy, double &qz, double &qw) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  qw = cr*cp*cy + sr*sp*sy;
  qx = sr*cp*cy - cr*sp*sy;
  qy = cr*sp*cy + sr*cp*sy;
  qz = cr*cp*sy - sr*sp*cy;
}

// -------------------- UDP packets --------------------
#pragma pack(push, 1)
struct CmdPacket {
  float left_rpm;
  float right_rpm;
};
struct RpmFeedbackPacket {
  float left_rpm;
  float right_rpm;
  double timestamp;
};

// IMU packet (as per your standalone receiver)
struct UdpImuPacket {
  uint32_t magic;        // "UIMU" = 0x554D4955
  uint16_t version;      // 1
  uint16_t payload_len;  // sizeof(UdpImuPacket)
  uint32_t seq;
  uint64_t t_monotonic_ns;
  float roll;
  float pitch;
  float yaw;
  uint32_t crc;
};
#pragma pack(pop)

class FlexBotUdpBridge : public rclcpp::Node {
public:
  FlexBotUdpBridge() : Node("flex_bot_udp_bridge") {
    imx7_ip_     = declare_parameter<std::string>("imx7_ip", "192.168.0.2");
    cmd_port_    = declare_parameter<int>("cmd_port", 5001);
    fb_port_     = declare_parameter<int>("fb_port", 5002);

    // IMU
    imu_port_    = declare_parameter<int>("imu_port", 5005);
    imu_frame_id_= declare_parameter<std::string>("imu_frame_id", "xsens_imu");
    imu_rpy_in_deg_ = declare_parameter<bool>("imu_rpy_in_degrees", false); // set true if iMX7 sends degrees

    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 50.0);

    // If true: ROS cmd topics are rad/s -> convert to rpm for UDP.
    cmds_are_radps_ = declare_parameter<bool>("cmds_are_radps", true);

    // If true: publish feedback as rad/s topics (optional convenience)
    publish_feedback_radps_ = declare_parameter<bool>("publish_feedback_radps", false);

    auto qos = rclcpp::QoS(10).best_effort();

    // Subscribe commands
    sub_left_cmd_ = create_subscription<std_msgs::msg::Float64>(
      "/left_wheel/cmd_vel", qos,
      [this](const std_msgs::msg::Float64 &m){
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        left_cmd_ = m.data;
      });

    sub_right_cmd_ = create_subscription<std_msgs::msg::Float64>(
      "/right_wheel/cmd_vel", qos,
      [this](const std_msgs::msg::Float64 &m){
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        right_cmd_ = m.data;
      });

    // Publish feedback (rpm always)
    pub_left_rpm_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/vel_rpm", qos);
    pub_right_rpm_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/vel_rpm", qos);

    // Optional: publish rad/s too
    if (publish_feedback_radps_) {
      pub_left_radps_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/vel_radps", qos);
      pub_right_radps_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/vel_radps", qos);
    }

    // IMU publisher
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/xsens_imu", rclcpp::QoS(50).best_effort());

    setup_udp();

    running_.store(true);
    rx_thread_  = std::thread([this]{ this->wheel_rx_loop(); });
    imu_thread_ = std::thread([this]{ this->imu_rx_loop(); });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
    cmd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]{ this->send_cmd_once(); });

    RCLCPP_INFO(get_logger(),
      "UDP bridge: cmd-> %s:%d, wheel fb<- :%d, imu<- :%d",
      imx7_ip_.c_str(), cmd_port_, fb_port_, imu_port_);
  }

  ~FlexBotUdpBridge() override {
    running_.store(false);
    if (rx_thread_.joinable()) rx_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();
    if (sock_tx_ >= 0) ::close(sock_tx_);
    if (sock_rx_wheel_ >= 0) ::close(sock_rx_wheel_);
    if (sock_rx_imu_ >= 0) ::close(sock_rx_imu_);
  }

private:
  void setup_udp() {
    // TX (commands to iMX7)
    sock_tx_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_tx_ < 0) throw std::runtime_error("Failed to create TX socket");

    std::memset(&imx7_addr_, 0, sizeof(imx7_addr_));
    imx7_addr_.sin_family = AF_INET;
    imx7_addr_.sin_port   = htons(cmd_port_);
    if (::inet_pton(AF_INET, imx7_ip_.c_str(), &imx7_addr_.sin_addr) != 1) {
      throw std::runtime_error("inet_pton failed for imx7_ip");
    }

    // RX wheel feedback bind
    sock_rx_wheel_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_rx_wheel_ < 0) throw std::runtime_error("Failed to create wheel RX socket");

    sockaddr_in local_w{};
    local_w.sin_family = AF_INET;
    local_w.sin_addr.s_addr = INADDR_ANY;
    local_w.sin_port = htons(fb_port_);

    int reuse = 1;
    ::setsockopt(sock_rx_wheel_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (::bind(sock_rx_wheel_, (sockaddr*)&local_w, sizeof(local_w)) < 0) {
      throw std::runtime_error("bind() failed on fb_port");
    }

    // RX IMU bind
    sock_rx_imu_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_rx_imu_ < 0) throw std::runtime_error("Failed to create IMU RX socket");

    sockaddr_in local_i{};
    local_i.sin_family = AF_INET;
    local_i.sin_addr.s_addr = INADDR_ANY;
    local_i.sin_port = htons(imu_port_);

    ::setsockopt(sock_rx_imu_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (::bind(sock_rx_imu_, (sockaddr*)&local_i, sizeof(local_i)) < 0) {
      throw std::runtime_error("bind() failed on imu_port");
    }

    // recv timeouts so threads can exit
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 20000;

    ::setsockopt(sock_rx_wheel_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(sock_rx_imu_,   SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  void send_cmd_once() {
    double l = 0.0, r = 0.0;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      l = left_cmd_;
      r = right_cmd_;
    }

    CmdPacket pkt{};
    if (cmds_are_radps_) {
      pkt.left_rpm  = static_cast<float>(radps_to_rpm(l));
      pkt.right_rpm = static_cast<float>(radps_to_rpm(r));
    } else {
      pkt.left_rpm  = static_cast<float>(l);
      pkt.right_rpm = static_cast<float>(r);
    }

    ::sendto(sock_tx_, &pkt, sizeof(pkt), 0,
             (sockaddr*)&imx7_addr_, sizeof(imx7_addr_));
  }

  void wheel_rx_loop() {
    RpmFeedbackPacket pkt{};
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);

    while (running_.load()) {
      const ssize_t n = ::recvfrom(sock_rx_wheel_, &pkt, sizeof(pkt), 0, (sockaddr*)&src, &srclen);
      if (n < 0) continue;
      if (n != (ssize_t)sizeof(pkt)) continue;

      // Publish RPM
      std_msgs::msg::Float64 m;
      m.data = (double)pkt.left_rpm;  pub_left_rpm_->publish(m);
      m.data = (double)pkt.right_rpm; pub_right_rpm_->publish(m);

      // Optionally publish rad/s
      if (publish_feedback_radps_) {
        m.data = rpm_to_radps((double)pkt.left_rpm);  pub_left_radps_->publish(m);
        m.data = rpm_to_radps((double)pkt.right_rpm); pub_right_radps_->publish(m);
      }
    }
  }

  void imu_rx_loop() {
    UdpImuPacket pkt{};
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);

    while (running_.load()) {
      const ssize_t n = ::recvfrom(sock_rx_imu_, &pkt, sizeof(pkt), 0, (sockaddr*)&src, &srclen);
      if (n < 0) continue;
      if (n != (ssize_t)sizeof(UdpImuPacket)) continue;

      // Header checks
      if (pkt.magic != 0x554D4955u || pkt.version != 1 || pkt.payload_len != sizeof(UdpImuPacket)) {
        continue;
      }

      // CRC check
      const uint32_t rx_crc = pkt.crc;
      pkt.crc = 0;
      const uint32_t calc = crc32(reinterpret_cast<const uint8_t*>(&pkt),
                                  sizeof(UdpImuPacket) - sizeof(uint32_t));
      if (calc != rx_crc) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU CRC mismatch (seq=%u)", pkt.seq);
        continue;
      }

      double roll  = (double)pkt.roll;
      double pitch = (double)pkt.pitch;
      double yaw   = (double)pkt.yaw;

      if (imu_rpy_in_deg_) {
        const double d2r = M_PI / 180.0;
        roll  *= d2r;
        pitch *= d2r;
        yaw   *= d2r;
      }

      double qx, qy, qz, qw;
      rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw);

      sensor_msgs::msg::Imu imu;
      imu.header.stamp = this->now();
      imu.header.frame_id = imu_frame_id_;

      imu.orientation.x = qx;
      imu.orientation.y = qy;
      imu.orientation.z = qz;
      imu.orientation.w = qw;

      // We do not have gyro/accel in this packet; mark unknown covariances.
      // ROS convention: covariance[0] = -1 means "unknown".
      for (int i = 0; i < 9; i++) {
        imu.orientation_covariance[i] = 0.0;
        imu.angular_velocity_covariance[i] = 0.0;
        imu.linear_acceleration_covariance[i] = 0.0;
      }
      imu.angular_velocity_covariance[0] = -1.0;
      imu.linear_acceleration_covariance[0] = -1.0;

      pub_imu_->publish(imu);
    }
  }

private:
  // params
  std::string imx7_ip_;
  int cmd_port_{5001};
  int fb_port_{5002};

  int imu_port_{5005};
  std::string imu_frame_id_{"xsens_imu"};
  bool imu_rpy_in_deg_{false};

  double cmd_rate_hz_{50.0};
  bool cmds_are_radps_{true};
  bool publish_feedback_radps_{true};

  // ros
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_cmd_, sub_right_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_rpm_, pub_right_rpm_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_radps_, pub_right_radps_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // cmd state
  std::mutex cmd_mtx_;
  double left_cmd_{0.0};
  double right_cmd_{0.0};

  // udp
  int sock_tx_{-1};
  int sock_rx_wheel_{-1};
  int sock_rx_imu_{-1};
  sockaddr_in imx7_addr_{};

  // threads
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  std::thread imu_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlexBotUdpBridge>());
  rclcpp::shutdown();
  return 0;
}
