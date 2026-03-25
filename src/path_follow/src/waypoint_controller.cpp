#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <obstacle_detector/msg/obstacles.hpp>
#include <obstacle_detector/msg/circle_obstacle.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <limits>

/**
 * WaypointController — Differential Drive + CBF-QP obstacle avoidance
 *
 * CBF-QP layer sits between pure-pursuit and wheel output:
 *   pure_pursuit → (v_nom, omega_nom) → cbfQP → (v_safe, omega_safe) → wheels
 *
 * QP solved with Eigen3 active-set method (no external solver needed).
 *
 * New subscriptions vs original:
 *   /tracked_obstacles   (obstacle_detector/msg/Obstacles)
 *   /positioning/status  (std_msgs/msg/String) — disable CBF during docking
 */
class WaypointController : public rclcpp::Node {
public:
    WaypointController() : rclcpp::Node("waypoint_controller") {

        // ── original params ────────────────────────────────────────────────
        wheel_radius_    = declare_parameter<double>("wheel_radius",    0.076);
        wheel_base_      = declare_parameter<double>("wheel_base",      0.50);
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

        // ── CBF params ────────────────────────────────────────────────────
        // alpha: CBF decay rate. higher = tighter safety, more aggressive swerve
        // r_robot: robot footprint radius for collision margin
        cbf_alpha_       = declare_parameter<double>("cbf_alpha",   1.5);
        r_robot_         = declare_parameter<double>("r_robot",     0.25);
        cbf_enabled_     = declare_parameter<bool>  ("cbf_enabled", true);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── subscribers ───────────────────────────────────────────────────
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

        // obstacle_detector tracked obstacles
        obs_sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
            "/tracked_obstacles", rclcpp::QoS(10).best_effort(),
            [this](obstacle_detector::msg::Obstacles::SharedPtr msg){
                obstacles_ = msg->circles;
            });

        // disable CBF when pgv dock controller has the wheels
        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/positioning/status", 10,
            [this](std_msgs::msg::String::SharedPtr msg){
                docking_ = (msg->data.rfind("DOCKING", 0) == 0 ||
                            msg->data.rfind("DOCKED",  0) == 0);
            });

        // ── publishers ────────────────────────────────────────────────────
        left_pub_   = create_publisher<std_msgs::msg::Float64>(
            "/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>(
            "/right_wheel/cmd_vel", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/controller/target", 1);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "WaypointController+CBF ready | "
            "r=%.3fm L=%.3fm v=%.2fm/s alpha=%.2f r_robot=%.2fm",
            wheel_radius_, wheel_base_, linear_speed_,
            cbf_alpha_, r_robot_);
    }

private:
    // ── params ─────────────────────────────────────────────────────────────
    double wheel_radius_, wheel_base_, max_wheel_rads_;
    double linear_speed_, max_omega_;
    double goal_tolerance_, final_tolerance_;
    double heading_kp_, lookahead_, slow_dist_, min_speed_;
    std::string map_frame_, base_frame_;
    double cbf_alpha_, r_robot_;
    bool   cbf_enabled_;

    // ── state ──────────────────────────────────────────────────────────────
    double pose_x_{0}, pose_y_{0}, pose_yaw_{0};
    bool   have_pose_{false};
    bool   docking_{false};

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t wp_idx_{0};
    bool   active_{false};

    std::vector<obstacle_detector::msg::CircleObstacle> obstacles_;

    // ── ROS handles ────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obs_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── utilities (unchanged from original) ────────────────────────────────
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

    // ── diff drive IK (unchanged) ──────────────────────────────────────────
    std::pair<double,double> diffDriveIK(double v, double omega) {
        const double a = wheel_base_ / 2.0;
        const double r = wheel_radius_;
        double omega_r = (v + omega * a) / r;
        double omega_l = (v - omega * a) / r;
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
    }

    void stopRobot() {
        std_msgs::msg::Float64 zero; zero.data = 0.0;
        left_pub_->publish(zero);
        right_pub_->publish(zero);
    }

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
        } catch (...) { return false; }
    }

    // ── CBF-QP ─────────────────────────────────────────────────────────────
    /**
     * Solve safety filter QP using Eigen3 active-set method.
     *
     * Problem:
     *   min  ||u - u_nom||²
     *   s.t. for each obstacle i:
     *        A_i * u <= b_i    (CBF constraint)
     *        velocity box constraints
     *
     * State:  p = (pose_x_, pose_y_)
     * Control: u = [v, omega]
     * Robot CoM velocity: [v*cos(yaw), v*sin(yaw)]
     *
     * CBF for circle obstacle i:
     *   h_i = ||p - p_i||² - r_safe_i²
     *   ḣ_i = 2(p-p_i)·(v_robot - v_obs_i) >= -alpha * h_i
     *
     * Rearranged as inequality:
     *   -2(dx*cos(yaw) + dy*sin(yaw)) * v  <=  Lfh_i + alpha*h_i
     *   (omega term is zero for CoM translation)
     */
    std::pair<double,double> cbfQP(double v_nom, double omega_nom) {
        if (!cbf_enabled_ || obstacles_.empty()) {
            return {v_nom, omega_nom};
        }

        const double yaw = pose_yaw_;
        const double px  = pose_x_;
        const double py  = pose_y_;

        // u_nom = [v_nom, omega_nom]
        Eigen::Vector2d u_nom(v_nom, omega_nom);

        // Build constraint matrix G*u <= h for all obstacles + box
        // Each obstacle gives 1 row, box gives 4 rows
        const int n_obs = static_cast<int>(obstacles_.size());
        const int n_con = n_obs + 4;   // obstacle CBFs + box constraints

        Eigen::MatrixXd G(n_con, 2);
        Eigen::VectorXd h_vec(n_con);

        for (int i = 0; i < n_obs; ++i) {
            const auto &obs = obstacles_[i];
            const double ox  = obs.center.x;
            const double oy  = obs.center.y;
            const double ovx = obs.velocity.x;
            const double ovy = obs.velocity.y;
            const double r_safe = obs.radius + r_robot_;

            const double dx     = px - ox;
            const double dy     = py - oy;
            const double dist_sq = dx*dx + dy*dy;

            // CBF value h_i = dist² - r_safe²
            const double h_val = dist_sq - r_safe * r_safe;

            // L_g h * u: only v affects CoM translation
            // = 2 * (dx*cos(yaw) + dy*sin(yaw)) * v
            const double Lgh_v     = 2.0 * (dx * std::cos(yaw) +
                                             dy * std::sin(yaw));
            const double Lgh_omega = 0.0;

            // L_f h: drift from obstacle velocity
            // = 2 * (dx*(-ovx) + dy*(-ovy))
            const double Lfh = 2.0 * (dx * (-ovx) + dy * (-ovy));

            // CBF constraint: Lgh*u >= -alpha*h - Lfh
            // → -Lgh*u <= Lfh + alpha*h
            G(i, 0) = -Lgh_v;
            G(i, 1) = -Lgh_omega;
            h_vec(i) = Lfh + cbf_alpha_ * h_val;
        }

        // Box constraints: v in [0, max_v], omega in [-max_omega, max_omega]
        // -v    <= 0           → row n_obs+0
        //  v    <= max_v       → row n_obs+1
        // -omega <= max_omega  → row n_obs+2
        //  omega <= max_omega  → row n_obs+3
        G(n_obs+0, 0) = -1.0; G(n_obs+0, 1) =  0.0; h_vec(n_obs+0) = 0.0;
        G(n_obs+1, 0) =  1.0; G(n_obs+1, 1) =  0.0; h_vec(n_obs+1) = linear_speed_;
        G(n_obs+2, 0) =  0.0; G(n_obs+2, 1) = -1.0; h_vec(n_obs+2) = max_omega_;
        G(n_obs+3, 0) =  0.0; G(n_obs+3, 1) =  1.0; h_vec(n_obs+3) = max_omega_;

        // ── Solve QP via projected gradient (lightweight, no external lib) ──
        // min 0.5||u - u_nom||²  s.t. G*u <= h
        //
        // We use iterative gradient projection:
        //   u = u_nom
        //   for each violated constraint i:
        //     project u onto feasible half-space
        // Converges in a few iterations for small n_obs.

        Eigen::Vector2d u = u_nom;
        const int max_iter = 20;

        for (int iter = 0; iter < max_iter; ++iter) {
            bool any_violated = false;

            for (int i = 0; i < n_con; ++i) {
                Eigen::Vector2d g = G.row(i).transpose();
                double slack = g.dot(u) - h_vec(i);

                if (slack > 1e-6) {   // constraint violated
                    any_violated = true;
                    double g_norm_sq = g.squaredNorm();
                    if (g_norm_sq < 1e-12) continue;

                    // project u back onto constraint boundary
                    u -= (slack / g_norm_sq) * g;
                }
            }

            if (!any_violated) break;
        }

        // Safety check — if QP drove v negative, stop
        if (u(0) < 0.0) u(0) = 0.0;

        return {u(0), u(1)};
    }

    // ── path callback (unchanged) ──────────────────────────────────────────
    void onPath(nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Empty path — stopping.");
            stopRobot();
            active_ = false;
            return;
        }
        waypoints_ = msg->poses;
        wp_idx_    = 0;
        active_    = true;
        RCLCPP_INFO(get_logger(), "New path: %zu waypoints.", waypoints_.size());
    }

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

    // ── control loop ──────────────────────────────────────────────────────
    void controlLoop() {
        // pgv dock controller has the wheels — step back completely
        if (docking_) return;

        if (!active_ || waypoints_.empty()) return;

        if (!have_pose_ && !tryTfPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for pose");
            return;
        }

        // advance past reached waypoints
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

        // check final goal
        double dist_final = dist2d(pose_x_, pose_y_,
                                   waypoints_.back().pose.position.x,
                                   waypoints_.back().pose.position.y);
        if (dist_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            stopRobot();
            active_ = false;
            return;
        }

        // ── pure pursuit → nominal control (unchanged) ──
        size_t tidx = lookaheadIndex();
        double tx = waypoints_[tidx].pose.position.x;
        double ty = waypoints_[tidx].pose.position.y;
        publishTargetMarker(tx, ty);

        double desired_yaw = std::atan2(ty - pose_y_, tx - pose_x_);
        double herr        = wrapAngle(desired_yaw - pose_yaw_);

        double omega = clamp(heading_kp_ * herr, -max_omega_, max_omega_);

        double v = linear_speed_;
        if (dist_final < slow_dist_) v *= dist_final / slow_dist_;
        v *= std::max(0.0, std::cos(herr));
        v  = std::max(min_speed_, v);
        if (std::fabs(herr) > M_PI / 2.0) v = 0.0;

        // ── CBF-QP safety filter ── (only change from original)
        auto [v_safe, omega_safe] = cbfQP(v, omega);

        // log when CBF is actively modifying control
        if (std::fabs(v_safe - v) > 0.01 ||
            std::fabs(omega_safe - omega) > 0.05) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "CBF active: v %.3f→%.3f omega %.3f→%.3f",
                v, v_safe, omega, omega_safe);
        }

        publishWheelCmds(v_safe, omega_safe);
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