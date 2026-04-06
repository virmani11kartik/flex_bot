#include "flex_bot_odometry/wheel_odom_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


WheelOdomNode::WheelOdomNode()
: Node("wheel_odom_node"),
  wheel_radius_m_(0.15301),
  wheel_base_m_(0.5000),
  bias_correction_factor_(1.0),
  alpha1_(0.1),
  alpha2_(0.01),
  alpha3_(0.01),
  alpha4_(0.1),
  encoder_noise_per_tick_(0.001),
  publish_rate_hz_(50.0),
  rpm_left_(0.0),
  rpm_right_(0.0),
  have_left_(false),
  have_right_(false),
  first_update_(true),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  last_delta_rot1_(0.0),
  last_delta_trans_(0.0),
  last_delta_rot2_(0.0)
{
  // Params
  frame_id_ = declare_parameter<std::string>("frame_id", "odom");
  child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odometry");

  wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", wheel_radius_m_);
  wheel_base_m_ = declare_parameter<double>("wheel_base_m", wheel_base_m_);
  bias_correction_factor_ = declare_parameter<double>("bias_correction_factor", bias_correction_factor_);

  alpha1_ = declare_parameter<double>("alpha1", alpha1_);
  alpha2_ = declare_parameter<double>("alpha2", alpha2_);
  alpha3_ = declare_parameter<double>("alpha3", alpha3_);
  alpha4_ = declare_parameter<double>("alpha4", alpha4_);
  encoder_noise_per_tick_ = declare_parameter<double>("encoder_noise_per_tick", encoder_noise_per_tick_);

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", publish_rate_hz_);

  // Init covariance (matches your ESP32 init)
  cov6_[0] = declare_parameter<double>("init_sigma_xx", 0.01);
  cov6_[1] = declare_parameter<double>("init_sigma_xy", 0.0);
  cov6_[2] = declare_parameter<double>("init_sigma_xth", 0.0);
  cov6_[3] = declare_parameter<double>("init_sigma_yy", 0.01);
  cov6_[4] = declare_parameter<double>("init_sigma_yth", 0.0);
  cov6_[5] = declare_parameter<double>("init_sigma_thth", 0.01);

  // Publisher
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  publish_tf_ = declare_parameter<bool>("publish_tf", false);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscriptions
  // auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();


  sub_left_rpm_ = create_subscription<std_msgs::msg::Float64>(
    "/left_wheel/vel_rpm", qos,
    std::bind(&WheelOdomNode::leftRpmCb, this, std::placeholders::_1));

  sub_right_rpm_ = create_subscription<std_msgs::msg::Float64>(
    "/right_wheel/vel_rpm", qos,
    std::bind(&WheelOdomNode::rightRpmCb, this, std::placeholders::_1));

  last_time_ = this->get_clock()->now();

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&WheelOdomNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "wheel_odom_node up. pub=%s sub=[/left_wheel/vel_rpm,/right_wheel/vel_rpm]",
              odom_topic_.c_str());
  RCLCPP_INFO(get_logger(),
              "Params: r=%.4f m, b=%.4f m, rate=%.1f Hz, alphas=[%.3f %.3f %.3f %.3f], enc_noise=%.6f",
              wheel_radius_m_, wheel_base_m_, publish_rate_hz_,
              alpha1_, alpha2_, alpha3_, alpha4_, encoder_noise_per_tick_);
}

void WheelOdomNode::leftRpmCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  rpm_left_ = msg->data;
  have_left_ = true;
}

void WheelOdomNode::rightRpmCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  rpm_right_ = msg->data;
  have_right_ = true;
}

double WheelOdomNode::normalizeAngle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void WheelOdomNode::matMul3(const double A[9], const double B[9], double C[9])
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += A[i*3 + k] * B[k*3 + j];
      }
      C[i*3 + j] = s;
    }
  }
}

void WheelOdomNode::matTranspose3(const double A[9], double AT[9])
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      AT[j*3 + i] = A[i*3 + j];
    }
  }
}

void WheelOdomNode::covToMat3(const double cov6[6], double M3[9])
{
  M3[0] = cov6[0];
  M3[1] = cov6[1];
  M3[2] = cov6[2];
  M3[3] = cov6[1];
  M3[4] = cov6[3];
  M3[5] = cov6[4];
  M3[6] = cov6[2];
  M3[7] = cov6[4];
  M3[8] = cov6[5];
}

void WheelOdomNode::mat3ToCov(const double M3[9], double cov6[6])
{
  cov6[0] = M3[0];
  cov6[1] = M3[1];
  cov6[2] = M3[2];
  cov6[3] = M3[4];
  cov6[4] = M3[5];
  cov6[5] = M3[8];
}

void WheelOdomNode::publishOdom(const rclcpp::Time& stamp, double v, double wz)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;

  // Pose
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // Twist
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = wz;

  // Pose covariance (6x6): fill x,y,yaw from our 3x3 cov
  for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;

  const double sxx   = cov6_[0];
  const double sxy   = cov6_[1];
  const double sxth  = cov6_[2];
  const double syy   = cov6_[3];
  const double syth  = cov6_[4];
  const double sthth = cov6_[5];

  // indices: [x y z roll pitch yaw]
  odom.pose.covariance[0]  = sxx;
  odom.pose.covariance[1]  = sxy;
  odom.pose.covariance[5]  = sxth;

  odom.pose.covariance[6]  = sxy;
  odom.pose.covariance[7]  = syy;
  odom.pose.covariance[11] = syth;

  odom.pose.covariance[30] = sxth;
  odom.pose.covariance[31] = syth;
  odom.pose.covariance[35] = sthth;

  // Twist covariance 
  for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
  odom.twist.covariance[0]  = 0.0005;  // vx
  odom.twist.covariance[35] = 0.0010;  // wz

  pub_odom_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;  // FIXED: changed from 'now' to 'stamp'
    t.header.frame_id = frame_id_;        // "odom"
    t.child_frame_id = child_frame_id_;   // "base_link"

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q_tf;  // FIXED: renamed to avoid shadowing
    q_tf.setRPY(0.0, 0.0, theta_);
    t.transform.rotation.x = q_tf.x();
    t.transform.rotation.y = q_tf.y();
    t.transform.rotation.z = q_tf.z();
    t.transform.rotation.w = q_tf.w();

    tf_broadcaster_->sendTransform(t);
  }
}

void WheelOdomNode::onTimer()
{
  if (!have_left_ || !have_right_) return;

  const auto now = this->get_clock()->now();
  const double dt = (now - last_time_).seconds();

  if (first_update_) {
    first_update_ = false;
    last_time_ = now;
    return;
  }

  // Guard pauses / time jumps
  if (dt <= 1e-6 || dt > 1.0) {
    last_time_ = now;
    return;
  }

  // RPM -> rad/s
  const double wL = rpm_left_  * (2.0 * M_PI / 60.0);
  const double wR = rpm_right_ * (2.0 * M_PI / 60.0);

  // Use dt to compute wheel angle increments
  const double dThetaL = wL * dt;
  const double dThetaR = wR * dt;

  // Distances
  const double dSLeft  = dThetaL * wheel_radius_m_ * bias_correction_factor_;
  const double dSRight = dThetaR * wheel_radius_m_ * bias_correction_factor_;

  // Robot motion
  double delta_trans = 0.5 * (dSLeft + dSRight);
  const double delta_rot = (wheel_base_m_ > 1e-9) ? ((dSRight - dSLeft) / wheel_base_m_) : 0.0;

  // Split rotation (preserve your ESP32 logic)
  double delta_rot1 = 0.0;
  double delta_rot2 = delta_rot;

  if (std::fabs(delta_rot) > 0.001 && std::fabs(delta_trans) > 0.001) {
    delta_rot1 = delta_rot / 2.0;
    delta_rot2 = delta_rot / 2.0;
  } else if (std::fabs(delta_rot) > 0.001) {
    delta_rot1 = 0.0;
    delta_rot2 = delta_rot;
    delta_trans = 0.0; // pure rotation
  } else {
    delta_rot1 = 0.0;
    delta_rot2 = 0.0;  // pure translation
  }

  last_delta_rot1_  = delta_rot1;
  last_delta_trans_ = delta_trans;
  last_delta_rot2_  = delta_rot2;

  // Deterministic update (midpoint)
  const double old_theta = theta_;
  x_ += delta_trans * std::cos(old_theta + delta_rot / 2.0);
  y_ += delta_trans * std::sin(old_theta + delta_rot / 2.0);
  theta_ = normalizeAngle(theta_ + delta_rot);

  // Probabilistic covariance propagation (keep your abs() style)
  double rot1_var  = alpha1_ * std::fabs(delta_rot1) + alpha2_ * std::fabs(delta_trans);
  double trans_var = alpha3_ * std::fabs(delta_trans) + alpha4_ * (std::fabs(delta_rot1) + std::fabs(delta_rot2));
  double rot2_var  = alpha1_ * std::fabs(delta_rot2) + alpha2_ * std::fabs(delta_trans);

  // Encoder noise add-on (same idea as your ESP32)
  const double total_distance = std::fabs(dSLeft) + std::fabs(dSRight);
  const double encoder_noise = encoder_noise_per_tick_ * total_distance / (2.0 * wheel_radius_m_);
  trans_var += encoder_noise;

  // Jacobians
  const double mid_theta = old_theta + delta_rot1;

  const double G[9] = {
    1.0, 0.0, -delta_trans * std::sin(mid_theta),
    0.0, 1.0,  delta_trans * std::cos(mid_theta),
    0.0, 0.0,  1.0
  };

  const double V[9] = {
    -delta_trans * std::sin(mid_theta),  std::cos(mid_theta), 0.0,
     delta_trans * std::cos(mid_theta),  std::sin(mid_theta), 0.0,
     1.0,                                0.0,               1.0
  };

  const double M[9] = {
    rot1_var, 0.0,     0.0,
    0.0,     trans_var,0.0,
    0.0,     0.0,     rot2_var
  };

  double Sigma[9];
  covToMat3(cov6_, Sigma);

  double GT[9], VT[9];
  matTranspose3(G, GT);
  matTranspose3(V, VT);

  double t1[9], t2[9], t3[9], t4[9];
  matMul3(G, Sigma, t1);
  matMul3(t1, GT, t2);

  matMul3(V, M, t3);
  matMul3(t3, VT, t4);

  for (int i = 0; i < 9; ++i) {
    t2[i] += t4[i];
  }

  mat3ToCov(t2, cov6_);

  // keep diagonal positive
  cov6_[0] = std::max(cov6_[0], 1e-9);
  cov6_[3] = std::max(cov6_[3], 1e-9);
  cov6_[5] = std::max(cov6_[5], 1e-9);

  // Twist from wheel linear v
  const double vL = wheel_radius_m_ * wL;
  const double vR = wheel_radius_m_ * wR;
  const double v  = 0.5 * (vR + vL);
  const double wz = (wheel_base_m_ > 1e-9) ? ((vR - vL) / wheel_base_m_) : 0.0;

  publishOdom(now, v, wz);
  last_time_ = now;
}