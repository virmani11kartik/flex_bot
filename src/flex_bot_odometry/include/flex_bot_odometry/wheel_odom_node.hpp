#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

class WheelOdomNode : public rclcpp::Node
{
public:
  WheelOdomNode();

private:
  // Sub callbacks
  void leftRpmCb(const std_msgs::msg::Float64::SharedPtr msg);
  void rightRpmCb(const std_msgs::msg::Float64::SharedPtr msg);

  // Main loop
  void onTimer();

  // Helpers
  static double normalizeAngle(double a);

  // 3x3 matrix ops (row-major)
  static void matMul3(const double A[9], const double B[9], double C[9]);
  static void matTranspose3(const double A[9], double AT[9]);

  // Covariance compact format: [σxx, σxy, σxθ, σyy, σyθ, σθθ]
  static void covToMat3(const double cov6[6], double M3[9]);
  static void mat3ToCov(const double M3[9], double cov6[6]);

  // Publish
  void publishOdom(const rclcpp::Time& stamp, double v, double wz);

  // Params
  std::string frame_id_;
  std::string child_frame_id_;
  std::string odom_topic_;
  double wheel_radius_m_;             // r
  double wheel_base_m_;               // b
  double bias_correction_factor_;

  // Probabilistic motion model params (preserve your ESP32 semantics)
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double encoder_noise_per_tick_;
  double publish_rate_hz_;

  // RPM subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_rpm_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_right_rpm_;
  double rpm_left_;
  double rpm_right_;
  bool have_left_;
  bool have_right_;

  // Timing
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  bool first_update_;

  // State
  double x_;
  double y_;
  double theta_;

  // Covariance compact
  double cov6_[6];

  // For PF chaining later (kept)
  double last_delta_rot1_;
  double last_delta_trans_;
  double last_delta_rot2_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  // TF broadcasting (ADDED)
  bool publish_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};