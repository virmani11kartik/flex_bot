#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

class SerialBridgeNode : public rclcpp::Node {
public:
  SerialBridgeNode() : Node("serial_bridge_node") {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_rate_   = declare_parameter<int>("baud_rate", 115200);

    fd_ = open_serial(serial_port_, baud_rate_);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Cannot open serial port: %s", serial_port_.c_str());
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", serial_port_.c_str(), baud_rate_);

    auto qos = rclcpp::QoS(10).best_effort();

    sub_act_   = create_subscription<std_msgs::msg::Int32>(
      "/actuator/cmd",   qos, [this](const std_msgs::msg::Int32 &m){ act_cb(m);   });
    sub_step_  = create_subscription<std_msgs::msg::Int32>(
      "/stepper/cmd",    qos, [this](const std_msgs::msg::Int32 &m){ step_cb(m);  });
    sub_speed_ = create_subscription<std_msgs::msg::Int32>(
      "/stepper/speed",  qos, [this](const std_msgs::msg::Int32 &m){ speed_cb(m); });

    running_.store(true);
    read_thread_ = std::thread([this]{ read_loop(); });

    RCLCPP_INFO(get_logger(), "serial_bridge_node ready.");
  }

  ~SerialBridgeNode() override {
    running_.store(false);
    if (read_thread_.joinable()) read_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  // ── Serial helpers ──────────────────────────────────────────────────────
  static int open_serial(const std::string &port, int baud) {
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    // Switch to blocking after open
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    termios tty{};
    tcgetattr(fd, &tty);

    speed_t spd = B115200;
    switch (baud) {
      case 9600:   spd = B9600;   break;
      case 57600:  spd = B57600;  break;
      case 115200: spd = B115200; break;
      case 230400: spd = B230400; break;
      default:     spd = B115200; break;
    }
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    cfmakeraw(&tty);           // 8N1, no flow control
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 2;       // 200 ms read timeout

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
  }

  void send(const std::string &line) {
    std::string msg = line + "\n";
    std::lock_guard<std::mutex> lk(mtx_);
    ssize_t n = ::write(fd_, msg.c_str(), msg.size());
    if (n < 0) RCLCPP_ERROR(get_logger(), "Serial write error");
    else       RCLCPP_DEBUG(get_logger(), "-> %s", line.c_str());
  }

  // ── Callbacks ───────────────────────────────────────────────────────────
  void act_cb(const std_msgs::msg::Int32 &m) {
    int v = std::clamp((int)m.data, -1, 1);
    send("CMD:ACT," + std::to_string(v));
  }

  void step_cb(const std_msgs::msg::Int32 &m) {
    int v = std::clamp((int)m.data, -1, 1);
    send("CMD:STEP," + std::to_string(v));
  }

  void speed_cb(const std_msgs::msg::Int32 &m) {
    int v = std::clamp((int)m.data, 100, 3000);
    send("CMD:SPEED," + std::to_string(v));
    RCLCPP_INFO(get_logger(), "-> CMD:SPEED,%d", v);
  }

  // ── Read loop — echoes ESP32 responses to ROS log ───────────────────────
  void read_loop() {
    char buf[256];
    std::string accum;

    while (running_.load()) {
      ssize_t n = ::read(fd_, buf, sizeof(buf) - 1);
      if (n <= 0) continue;

      buf[n] = '\0';
      accum += buf;

      size_t pos;
      while ((pos = accum.find('\n')) != std::string::npos) {
        std::string line = accum.substr(0, pos);
        // strip \r if present
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty())
          RCLCPP_INFO(get_logger(), "ESP32: %s", line.c_str());
        accum.erase(0, pos + 1);
      }
    }
  }

private:
  std::string serial_port_;
  int         baud_rate_{115200};
  int         fd_{-1};

  std::mutex  mtx_;

  std::atomic<bool> running_{false};
  std::thread       read_thread_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_act_, sub_step_, sub_speed_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
