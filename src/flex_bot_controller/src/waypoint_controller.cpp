#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <vector>
#include <algorithm>

/**
 * WaypointController
 *
 * Subscribes to /astar/path and drives the robot through each waypoint
 * using pure-pursuit or a simple P-heading controller.
 *
 * Parameters:
 *   map_frame          (string, "map")
 *   base_frame         (string, "base_link")
 *   linear_speed       (double, 0.3)   m/s cruise speed
 *   max_angular_speed  (double, 1.0)   rad/s clamp
 *   goal_tolerance     (double, 0.20)  m  — advance to next waypoint
 *   final_tolerance    (double, 0.10)  m  — stop at last waypoint
 *   heading_kp         (double, 2.0)   angular P gain
 *   lookahead          (double, 0.5)   m  — pure-pursuit look-ahead
 *   use_pure_pursuit   (bool,   true)
 *   slow_dist          (double, 0.8)   m  — start slowing near final goal
 *   min_speed          (double, 0.05)  m/s minimum while moving
 */
class WaypointController : public rclcpp::Node {
public:
    WaypointController() : rclcpp::Node("waypoint_controller") {
        map_frame_         = declare_parameter<std::string>("map_frame",        "map");
        base_frame_        = declare_parameter<std::string>("base_frame",       "base_link");
        linear_speed_      = declare_parameter<double>("linear_speed",          0.3);
        max_angular_speed_ = declare_parameter<double>("max_angular_speed",     1.0);
        goal_tolerance_    = declare_parameter<double>("goal_tolerance",        0.20);
        final_tolerance_   = declare_parameter<double>("final_tolerance",       0.10);
        heading_kp_        = declare_parameter<double>("heading_kp",            2.0);
        lookahead_         = declare_parameter<double>("lookahead",             0.5);
        use_pure_pursuit_  = declare_parameter<bool>  ("use_pure_pursuit",      true);
        slow_dist_         = declare_parameter<double>("slow_dist",             0.8);
        min_speed_         = declare_parameter<double>("min_speed",             0.05);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/astar/path", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::Path::SharedPtr msg){ onPath(msg); });

        cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/target", 1);

        // 20 Hz control loop
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "WaypointController ready — speed=%.2f m/s, lookahead=%.2f m, mode=%s",
            linear_speed_, lookahead_, use_pure_pursuit_ ? "pure-pursuit" : "P-heading");
    }

private:
    std::string map_frame_, base_frame_;
    double linear_speed_, max_angular_speed_;
    double goal_tolerance_, final_tolerance_;
    double heading_kp_, lookahead_;
    bool   use_pure_pursuit_;
    double slow_dist_, min_speed_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t wp_idx_{0};
    bool   active_{false};

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void onPath(nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty path — stopping.");
            stopRobot();
            active_ = false;
            return;
        }
        waypoints_ = msg->poses;
        wp_idx_    = 0;
        active_    = true;
        RCLCPP_INFO(get_logger(), "New path: %zu waypoints.", waypoints_.size());
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    bool getRobotPose(double &rx, double &ry, double &ryaw) {
        try {
            auto tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero,
                std::chrono::milliseconds(50));
            rx   = tf.transform.translation.x;
            ry   = tf.transform.translation.y;
            const auto &q = tf.transform.rotation;
            ryaw = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    inline double dist2d(double ax, double ay, double bx, double by) {
        return std::hypot(bx - ax, by - ay);
    }

    inline double clampVal(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    // Wrap angle to [-pi, pi]
    inline double wrapAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void stopRobot() {
        geometry_msgs::msg::Twist t;
        cmd_pub_->publish(t);
    }

    // ── Pure pursuit: find look-ahead point on path ───────────────────────────
    // Returns index of the look-ahead waypoint
    size_t lookaheadIndex(double rx, double ry) {
        // Start from current wp_idx and walk forward until we exceed lookahead
        for (size_t i = wp_idx_; i < waypoints_.size(); ++i) {
            double d = dist2d(rx, ry,
                              waypoints_[i].pose.position.x,
                              waypoints_[i].pose.position.y);
            if (d >= lookahead_) return i;
        }
        return waypoints_.size() - 1; // last point
    }

    // ── Main control loop ─────────────────────────────────────────────────────
    void controlLoop() {
        if (!active_ || waypoints_.empty()) return;

        double rx, ry, ryaw;
        if (!getRobotPose(rx, ry, ryaw)) return;

        // Advance waypoint index while robot is close enough to current wp
        while (wp_idx_ < waypoints_.size() - 1) {
            double d = dist2d(rx, ry,
                              waypoints_[wp_idx_].pose.position.x,
                              waypoints_[wp_idx_].pose.position.y);
            if (d < goal_tolerance_) {
                ++wp_idx_;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Advanced to waypoint %zu / %zu", wp_idx_, waypoints_.size());
            } else {
                break;
            }
        }

        // Check final goal reached
        const auto &final_wp = waypoints_.back();
        double dist_to_final = dist2d(rx, ry,
                                      final_wp.pose.position.x,
                                      final_wp.pose.position.y);
        if (dist_to_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached! Stopping.");
            stopRobot();
            active_ = false;
            return;
        }

        // Pick target point
        size_t target_idx = use_pure_pursuit_
            ? lookaheadIndex(rx, ry)
            : wp_idx_;

        const auto &target = waypoints_[target_idx];
        double tx = target.pose.position.x;
        double ty = target.pose.position.y;

        publishTargetMarker(tx, ty);

        // Heading error
        double desired_yaw = std::atan2(ty - ry, tx - rx);
        double heading_err = wrapAngle(desired_yaw - ryaw);

        // Angular command (P controller)
        double omega = clampVal(heading_kp_ * heading_err,
                                -max_angular_speed_, max_angular_speed_);

        // Linear speed — slow down near final goal and when turning hard
        double speed = linear_speed_;

        // Slow near goal
        if (dist_to_final < slow_dist_) {
            speed *= (dist_to_final / slow_dist_);
        }

        // Reduce speed when heading error is large (> ~30 deg)
        double heading_factor = std::max(0.0, std::cos(heading_err));
        speed *= heading_factor;
        speed  = std::max(min_speed_, speed);

        // Stop linear motion if heading error is very large (> 90 deg)
        if (std::fabs(heading_err) > M_PI / 2.0) {
            speed = 0.0;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = speed;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);
    }

    // ── Marker: show current target waypoint in RViz ──────────────────────────
    void publishTargetMarker(double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp    = now();
        m.ns  = "controller";
        m.id  = 0;
        m.type   = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.2;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 1.0f;
        m.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_pub_->publish(m);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>());
    rclcpp::shutdown();
    return 0;
}
