#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>

// PGV message — uses geometry_msgs/PointStamped for position
// and a custom-like structure via basic ROS types
#include <geometry_msgs/msg/point_stamped.hpp>

// We publish PGV as a custom topic using a simple struct republished
// via std_msgs until a proper custom msg is added.
// Topic: /pgv/data  (geometry_msgs/PointStamped for position)
// Topic: /pgv/tag_id (std_msgs/Float64 — tag ID, 0 = no tag visible)
// Topic: /pgv/angle_deg (std_msgs/Float64 — heading angle in degrees)

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

// ── IMU helpers ──────────────────────────────────────────────────────────────
static uint32_t crc32_calc(const uint8_t* data, size_t len) {
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

static inline void rpy_to_quat(double roll, double pitch, double yaw,
                                double &qx, double &qy, double &qz, double &qw) {
  const double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
  qw = cr*cp*cy + sr*sp*sy;
  qx = sr*cp*cy - cr*sp*sy;
  qy = cr*sp*cy + sr*cp*sy;
  qz = cr*cp*sy - sr*sp*cy;
}

// ── UDP packet definitions ────────────────────────────────────────────────────
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

struct UdpImuPacket {
  uint32_t magic;         // "UIMU" = 0x554D4955
  uint16_t version;       // 1
  uint16_t payload_len;
  uint32_t seq;
  uint64_t t_monotonic_ns;
  float roll;
  float pitch;
  float yaw;
  uint32_t crc;
};

// PGV packet — matches pgv_tx.cpp UdpPgvPacket exactly
struct UdpPgvPacket {
  uint32_t magic;           // 'PGV1' = 0x50475631
  uint16_t version;         // 1
  uint16_t payload_len;
  uint32_t seq;
  uint64_t host_time_us;    // sender monotonic time in us
  int32_t  x_01mm;          // x position in 0.1mm units
  int32_t  y_01mm;          // y position in 0.1mm units
  int16_t  ang_01deg;       // heading angle in 0.1° units
  int16_t  spd_01mms;       // speed in 0.1 mm/s units
  uint32_t pgv_ts_ms;       // PGV internal timestamp ms
  uint8_t  tag_id;          // data matrix tag ID (0 = no tag)
  uint8_t  reserved[3];
};

#pragma pack(pop)

// ntohl for 64-bit
static inline uint64_t ntohll_u64(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (uint64_t(ntohl(uint32_t(x & 0xFFFFFFFFULL))) << 32) | ntohl(uint32_t(x >> 32));
#else
  return x;
#endif
}

// ── Node ─────────────────────────────────────────────────────────────────────
class FlexBotUdpBridge : public rclcpp::Node {
public:
  FlexBotUdpBridge() : Node("flex_bot_udp_bridge") {

    // ── Parameters ────────────────────────────────────────────────────────────
    imx7_ip_          = declare_parameter<std::string>("imx7_ip",          "192.168.0.2");
    cmd_port_         = declare_parameter<int>("cmd_port",                  5001);
    fb_port_          = declare_parameter<int>("fb_port",                   5002);
    imu_port_         = declare_parameter<int>("imu_port",                  5005);
    pgv_port_         = declare_parameter<int>("pgv_port",                  5003);
    imu_frame_id_     = declare_parameter<std::string>("imu_frame_id",     "xsens_imu");
    pgv_frame_id_     = declare_parameter<std::string>("pgv_frame_id",     "base_link");
    imu_rpy_in_deg_   = declare_parameter<bool>("imu_rpy_in_degrees",       false);
    cmd_rate_hz_      = declare_parameter<double>("cmd_rate_hz",             50.0);
    cmds_are_radps_   = declare_parameter<bool>("cmds_are_radps",            true);
    publish_feedback_radps_ = declare_parameter<bool>("publish_feedback_radps", true);

    auto qos = rclcpp::QoS(10).best_effort();

    // ── Wheel command subscribers ──────────────────────────────────────────────
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

    // ── Wheel feedback publishers ──────────────────────────────────────────────
    pub_left_rpm_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/vel_rpm",  qos);
    pub_right_rpm_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/vel_rpm", qos);
    if (publish_feedback_radps_) {
      pub_left_radps_  = create_publisher<std_msgs::msg::Float64>("/left_wheel/vel_radps",  qos);
      pub_right_radps_ = create_publisher<std_msgs::msg::Float64>("/right_wheel/vel_radps", qos);
    }

    // ── IMU publisher ─────────────────────────────────────────────────────────
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(
      "/xsens_imu", rclcpp::QoS(50).best_effort());

    // ── PGV publishers ────────────────────────────────────────────────────────
    // /pgv/position  — x,y in metres (in pgv_frame_id, typically base_link or map)
    // /pgv/tag_id    — current visible tag ID (0.0 = no tag)
    // /pgv/angle_deg — heading angle reported by PGV in degrees
    pub_pgv_pos_   = create_publisher<geometry_msgs::msg::PointStamped>("/pgv/position",  qos);
    pub_pgv_tag_   = create_publisher<std_msgs::msg::Float64>("/pgv/tag_id",    qos);
    pub_pgv_angle_ = create_publisher<std_msgs::msg::Float64>("/pgv/angle_deg", qos);

    setup_udp();

    running_.store(true);
    rx_thread_   = std::thread([this]{ wheel_rx_loop(); });
    imu_thread_  = std::thread([this]{ imu_rx_loop();   });
    pgv_thread_  = std::thread([this]{ pgv_rx_loop();   });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
    cmd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]{ send_cmd_once(); });

    RCLCPP_INFO(get_logger(),
      "UDP bridge: cmd->%s:%d  wheel_fb<-:%d  imu<-:%d  pgv<-:%d",
      imx7_ip_.c_str(), cmd_port_, fb_port_, imu_port_, pgv_port_);
  }

  ~FlexBotUdpBridge() override {
    running_.store(false);
    if (rx_thread_.joinable())  rx_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();
    if (pgv_thread_.joinable()) pgv_thread_.join();
    if (sock_tx_       >= 0) ::close(sock_tx_);
    if (sock_rx_wheel_ >= 0) ::close(sock_rx_wheel_);
    if (sock_rx_imu_   >= 0) ::close(sock_rx_imu_);
    if (sock_rx_pgv_   >= 0) ::close(sock_rx_pgv_);
  }

private:
  // ── UDP setup ─────────────────────────────────────────────────────────────
  void setup_udp() {
    // TX socket (commands → IMX7)
    sock_tx_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_tx_ < 0) throw std::runtime_error("Failed to create TX socket");

    std::memset(&imx7_addr_, 0, sizeof(imx7_addr_));
    imx7_addr_.sin_family = AF_INET;
    imx7_addr_.sin_port   = htons(cmd_port_);
    if (::inet_pton(AF_INET, imx7_ip_.c_str(), &imx7_addr_.sin_addr) != 1)
      throw std::runtime_error("inet_pton failed for imx7_ip");

    // Helper lambda to create a bound UDP RX socket
    auto make_rx = [](int port) -> int {
      int s = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (s < 0) throw std::runtime_error("Failed to create RX socket");
      int reuse = 1;
      ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      sockaddr_in addr{};
      addr.sin_family      = AF_INET;
      addr.sin_addr.s_addr = INADDR_ANY;
      addr.sin_port        = htons(port);
      if (::bind(s, (sockaddr*)&addr, sizeof(addr)) < 0)
        throw std::runtime_error("bind() failed on port " + std::to_string(port));
      // 20ms recv timeout so threads can exit cleanly
      timeval tv{}; tv.tv_sec = 0; tv.tv_usec = 20000;
      ::setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      return s;
    };

    sock_rx_wheel_ = make_rx(fb_port_);
    sock_rx_imu_   = make_rx(imu_port_);
    sock_rx_pgv_   = make_rx(pgv_port_);
  }

  // ── Command sender (timer callback) ───────────────────────────────────────
  void send_cmd_once() {
    double l = 0.0, r = 0.0;
    { std::lock_guard<std::mutex> lk(cmd_mtx_); l = left_cmd_; r = right_cmd_; }

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

  // ── Wheel feedback receiver thread ────────────────────────────────────────
  void wheel_rx_loop() {
    RpmFeedbackPacket pkt{};
    sockaddr_in src{}; socklen_t srclen = sizeof(src);
    while (running_.load()) {
      ssize_t n = ::recvfrom(sock_rx_wheel_, &pkt, sizeof(pkt), 0, (sockaddr*)&src, &srclen);
      if (n != (ssize_t)sizeof(pkt)) continue;

      std_msgs::msg::Float64 m;
      m.data = (double)pkt.left_rpm;  pub_left_rpm_->publish(m);
      m.data = (double)pkt.right_rpm; pub_right_rpm_->publish(m);

      if (publish_feedback_radps_) {
        m.data = rpm_to_radps((double)pkt.left_rpm);  pub_left_radps_->publish(m);
        m.data = rpm_to_radps((double)pkt.right_rpm); pub_right_radps_->publish(m);
      }
    }
  }

  // ── IMU receiver thread ───────────────────────────────────────────────────
  void imu_rx_loop() {
    UdpImuPacket pkt{};
    sockaddr_in src{}; socklen_t srclen = sizeof(src);
    while (running_.load()) {
      ssize_t n = ::recvfrom(sock_rx_imu_, &pkt, sizeof(pkt), 0, (sockaddr*)&src, &srclen);
      if (n != (ssize_t)sizeof(UdpImuPacket)) continue;
      if (pkt.magic != 0x554D4955u || pkt.version != 1) continue;

      // CRC check
      const uint32_t rx_crc = pkt.crc;
      pkt.crc = 0;
      if (crc32_calc(reinterpret_cast<const uint8_t*>(&pkt),
                     sizeof(UdpImuPacket) - sizeof(uint32_t)) != rx_crc) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "IMU CRC mismatch (seq=%u)", pkt.seq);
        continue;
      }

      double roll = pkt.roll, pitch = pkt.pitch, yaw = pkt.yaw;
      if (imu_rpy_in_deg_) {
        const double d2r = M_PI / 180.0;
        roll *= d2r; pitch *= d2r; yaw *= d2r;
      }

      double qx, qy, qz, qw;
      rpy_to_quat(roll, pitch, yaw, qx, qy, qz, qw);

      sensor_msgs::msg::Imu imu;
      imu.header.stamp    = this->now();
      imu.header.frame_id = imu_frame_id_;
      imu.orientation.x   = qx;
      imu.orientation.y   = qy;
      imu.orientation.z   = qz;
      imu.orientation.w   = qw;
      // No gyro/accel in this packet
      imu.angular_velocity_covariance[0]    = -1.0;
      imu.linear_acceleration_covariance[0] = -1.0;
      pub_imu_->publish(imu);
    }
  }

  // ── PGV receiver thread ───────────────────────────────────────────────────
  void pgv_rx_loop() {
    UdpPgvPacket pkt{};
    sockaddr_in src{}; socklen_t srclen = sizeof(src);

    while (running_.load()) {
      ssize_t n = ::recvfrom(sock_rx_pgv_, &pkt, sizeof(pkt), 0, (sockaddr*)&src, &srclen);
      if (n < 0) continue;
      if (n != (ssize_t)sizeof(UdpPgvPacket)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "PGV packet wrong size: got %zd expected %zu", n, sizeof(UdpPgvPacket));
        continue;
      }

      // Magic check: 'PGV1' = 0x50475631
      if (ntohl(pkt.magic) != 0x50475631u) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "PGV bad magic: 0x%08X", ntohl(pkt.magic));
        continue;
      }

      // Decode — all fields are network byte order (big-endian) from pgv_tx.cpp
      const double x_m       = (double)((int32_t)ntohl((uint32_t)pkt.x_01mm))  * 1e-4; // 0.1mm → m
      const double y_m       = (double)((int32_t)ntohl((uint32_t)pkt.y_01mm))  * 1e-4;
      const double angle_deg = (double)((int16_t)ntohs((uint16_t)pkt.ang_01deg)) * 0.1; // 0.1° → °
      const uint8_t tag_id   = pkt.tag_id;

      auto stamp = this->now();

      // /pgv/position — x,y in metres
      geometry_msgs::msg::PointStamped pos;
      pos.header.stamp    = stamp;
      pos.header.frame_id = pgv_frame_id_;
      pos.point.x = x_m;
      pos.point.y = y_m;
      pos.point.z = 0.0;
      pub_pgv_pos_->publish(pos);

      // /pgv/tag_id
      std_msgs::msg::Float64 tag_msg;
      tag_msg.data = (double)tag_id;
      pub_pgv_tag_->publish(tag_msg);

      // /pgv/angle_deg
      std_msgs::msg::Float64 ang_msg;
      ang_msg.data = angle_deg;
      pub_pgv_angle_->publish(ang_msg);

      RCLCPP_DEBUG(get_logger(),
        "PGV: x=%.4fm y=%.4fm angle=%.1f° tag=%u",
        x_m, y_m, angle_deg, tag_id);

      // Log tag detection at INFO level (throttled)
      if (tag_id != 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "PGV tag detected: id=%u at (%.4f, %.4f) m angle=%.1f°",
          tag_id, x_m, y_m, angle_deg);
      }
    }
  }

  // ── Params ────────────────────────────────────────────────────────────────
  std::string imx7_ip_;
  int cmd_port_, fb_port_, imu_port_, pgv_port_;
  std::string imu_frame_id_, pgv_frame_id_;
  bool imu_rpy_in_deg_, cmds_are_radps_, publish_feedback_radps_;
  double cmd_rate_hz_;

  // ── ROS handles ───────────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_cmd_, sub_right_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_left_rpm_,  pub_right_rpm_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_left_radps_, pub_right_radps_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr     pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_pgv_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_pgv_tag_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_pgv_angle_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  // ── State ─────────────────────────────────────────────────────────────────
  std::mutex cmd_mtx_;
  double left_cmd_{0.0}, right_cmd_{0.0};

  // ── UDP sockets ───────────────────────────────────────────────────────────
  int sock_tx_{-1}, sock_rx_wheel_{-1}, sock_rx_imu_{-1}, sock_rx_pgv_{-1};
  sockaddr_in imx7_addr_{};

  // ── Threads ───────────────────────────────────────────────────────────────
  std::atomic<bool> running_{false};
  std::thread rx_thread_, imu_thread_, pgv_thread_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlexBotUdpBridge>());
  rclcpp::shutdown();
  ret