#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <vector>

/**
 * WaypointController — Differential Drive
 *
 * State source (priority order):
 *   1. /amcl_pose  (PoseWithCovarianceStamped) — AMCL localization
 *   2. map->base_link TF                        — fallback
 *
 * Outputs:
 *   /left_wheel/cmd_vel   (std_msgs/Float64)  rad/s
 *   /right_wheel/cmd_vel  (std_msgs/Float64)  rad/s
 *
 * Parameters:
 *   wheel_radius    (double, 0.076)   m
 *   wheel_base      (double, 0.50)    m   full track width (left-to-right)
 *   max_wheel_rads  (double, 3.0)     rad/s hardware limit
 *   linear_speed    (double, 0.25)    m/s cruise speed
 *   max_omega       (double, 1.2)     rad/s angular clamp
 *   goal_tolerance  (double, 0.40)    m   advance to next waypoint
 *   final_tolerance (double, 0.10)    m   declare goal reached
 *   heading_kp      (double, 1.5)     angular P gain
 *   lookahead       (double, 0.6)     m   pure-pursuit look-ahead
 *   slow_dist       (double, 0.8)     m   start slowing near goal
 *   min_speed       (double, 0.04)    m/s minimum forward speed
 *   map_frame       (string, "map")
 *   base_frame      (string, "base_link")
 */
class WaypointController : public rclcpp::Node {
public:
    WaypointController() : rclcpp::Node("waypoint_controller") {

        wheel_radius_    = declare_parameter<double>("wheel_radius",    0.076);
        wheel_base_      = declare_parameter<double>("wheel_base",      0.30);
        max_wheel_rads_  = declare_parameter<double>("max_wheel_rads",  3.0);
        linear_speed_    = declare_parameter<double>("linear_speed",    0.25);
        max_omega_       = declare_parameter<double>("max_omega",       1.2);
        goal_tolerance_  = declare_parameter<double>("goal_tolerance",  0.20);
        final_tolerance_ = declare_parameter<double>("final_tolerance", 0.10);
        heading_kp_      = declare_parameter<double>("heading_kp",      2.5);
        lookahead_       = declare_parameter<double>("lookahead",       0.6);
        slow_dist_       = declare_parameter<double>("slow_dist",       0.8);
        min_speed_       = declare_parameter<double>("min_speed",       0.04);
        map_frame_       = declare_parameter<std::string>("map_frame",  "map");
        base_frame_      = declare_parameter<std::string>("base_frame", "base_link");

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Primary pose: AMCL
        amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", rclcpp::QoS(5).reliable(),
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                const auto &p = msg->pose.pose;
                pose_x_   = p.position.x;
                pose_y_   = p.position.y;
                pose_yaw_ = quatToYaw(p.orientation.x, p.orientation.y,
                                      p.orientation.z, p.orientation.w);
                have_pose_ = true;
            });

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/astar/path", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::Path::SharedPtr msg){ onPath(msg); });

        left_pub_   = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/target", 1);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),   // 20 Hz
            [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "WaypointController ready | r=%.3fm L=%.3fm max=%.1frad/s v=%.2fm/s",
            wheel_radius_, wheel_base_, max_wheel_rads_, linear_speed_);
    }

private:
    double wheel_radius_, wheel_base_, max_wheel_rads_;
    double linear_speed_, max_omega_;
    double goal_tolerance_, final_tolerance_;
    double heading_kp_, lookahead_, slow_dist_, min_speed_;
    std::string map_frame_, base_frame_;

    double pose_x_{0}, pose_y_{0}, pose_yaw_{0};
    bool   have_pose_{false};

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t wp_idx_{0};
    bool   active_{false};

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── Utilities ─────────────────────────────────────────────────────────────
    static double quatToYaw(double x, double y, double z, double w) {
        return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
    }

    inline double wrapAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    inline double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    inline double dist2d(double ax, double ay, double bx, double by) {
        return std::hypot(bx - ax, by - ay);
    }

    // ── Diff-drive inverse kinematics ─────────────────────────────────────────
    // Returns {omega_left, omega_right} in rad/s
    std::pair<double,double> diffDriveIK(double v, double omega) {
        const double a = wheel_base_ / 2.0;
        const double r = wheel_radius_;

        double omega_r = (v + omega * a) / r;
        double omega_l = (v - omega * a) / r;

        // Scale both proportionally if either exceeds hardware limit
        double max_abs = std::max(std::fabs(omega_r), std::fabs(omega_l));
        if (max_abs > max_wheel_rads_) {
            double scale = max_wheel_rads_ / max_abs;
            omega_r *= scale;
            omega_l *= scale;
        }
        return {omega_l, omega_r};
    }

    void publishWheelCmds(double v, double omega) {
        auto [omega_l, omega_r] = diffDriveIK(v, omega);
        std_msgs::msg::Float64 l, r;
        l.data = -omega_l;   // negate: hardware positive = backward
        r.data = -omega_r;
        left_pub_->publish(l);
        right_pub_->publish(r);

        RCLCPP_DEBUG(get_logger(), "cmd: v=%.3f ω=%.3f → L=%.3f R=%.3f rad/s",
                     v, omega, omega_l, omega_r);
    }

    void stopRobot() {
        std_msgs::msg::Float64 zero;
        zero.data = 0.0;
        left_pub_->publish(zero);
        right_pub_->publish(zero);
    }

    // Fallback: get pose from TF if AMCL not yet available
    bool tryTfPose() {
        try {
            auto tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero,
                std::chrono::milliseconds(20));
            pose_x_   = tf.transform.translation.x;
            pose_y_   = tf.transform.translation.y;
            const auto &q = tf.transform.rotation;
            pose_yaw_ = quatToYaw(q.x, q.y, q.z, q.w);
            have_pose_ = true;
            return true;
        } catch (...) {
            return false;
        }
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void onPath(nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Empty path received — stopping.");
            stopRobot();
            active_ = false;
            return;
        }
        waypoints_ = msg->poses;
        wp_idx_    = 0;
        active_    = true;
        RCLCPP_INFO(get_logger(), "New path: %zu waypoints.", waypoints_.size());
    }

    // Pure-pursuit: find first waypoint beyond lookahead distance
    size_t lookaheadIndex() {
        for (size_t i = wp_idx_; i < waypoints_.size(); ++i) {
            if (dist2d(pose_x_, pose_y_,
                       waypoints_[i].pose.position.x,
                       waypoints_[i].pose.position.y) >= lookahead_) {
                return i;
            }
        }
        return waypoints_.size() - 1;
    }

    // ── Control loop (20 Hz) ──────────────────────────────────────────────────
    void controlLoop() {
        if (!active_ || waypoints_.empty()) return;

        // Ensure we have a pose
        if (!have_pose_ && !tryTfPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for pose (/amcl_pose or map->base_link TF)");
            return;
        }

        // Advance past already-reached waypoints
        while (wp_idx_ < waypoints_.size() - 1) {
            double d = dist2d(pose_x_, pose_y_,
                              waypoints_[wp_idx_].pose.position.x,
                              waypoints_[wp_idx_].pose.position.y);
            if (d < goal_tolerance_) {
                ++wp_idx_;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
                    "Waypoint %zu/%zu", wp_idx_, waypoints_.size());
            } else break;
        }

        // Check final goal
        double dist_final = dist2d(pose_x_, pose_y_,
                                   waypoints_.back().pose.position.x,
                                   waypoints_.back().pose.position.y);
        if (dist_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            stopRobot();
            active_ = false;
            return;
        }

        // Look-ahead target
        size_t tidx = lookaheadIndex();
        double tx = waypoints_[tidx].pose.position.x;
        double ty = waypoints_[tidx].pose.position.y;
        publishTargetMarker(tx, ty);

        // Heading error
        double desired_yaw = std::atan2(ty - pose_y_, tx - pose_x_);
        double herr        = wrapAngle(desired_yaw - pose_yaw_);

        // Angular velocity — P controller
        double omega = clamp(heading_kp_ * herr, -max_omega_, max_omega_);

        // Linear speed — reduce when turning hard, slow near goal
        double v = linear_speed_;
        if (dist_final < slow_dist_)
            v *= dist_final / slow_dist_;
        v *= std::max(0.0, std::cos(herr));
        v  = std::max(min_speed_, v);

        // Pure rotation if heading error > 90°
        if (std::fabs(herr) > M_PI / 2.0) v = 0.0;

        publishWheelCmds(v, omega);
    }

    void publishTargetMarker(double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp    = now();
        m.ns = "controller"; m.id = 0;
        m.type   = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.25;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        m.color.r = 1.0f; m.color.g = 0.5f;
        m.color.b = 0.0f; m.color.a = 1.0f;
        m.lifetime = rclcpp::Duration::from_seconds(0.15);
        marker_pub_->publish(m);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>());
    rclcpp::shutdown();
    return 0;
}
