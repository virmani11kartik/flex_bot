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
 * CBF improvements over naive version:
 *   1. Directional gating  — only obstacles in forward cone trigger CBF
 *   2. Lateral gating      — obstacles far to the side are ignored
 *   3. Influence distance  — obstacles beyond cbf_influence_dist_ ignored
 *   4. Directional scaling — alpha scaled by how directly ahead obstacle is
 *   5. Omega correction    — Lgh_omega includes lookahead so CBF steers around
 *
 * Parameters (all tunable via ROS params / yaml):
 *   cbf_alpha            (double, 1.0)   CBF decay rate
 *   r_robot              (double, 0.25)  robot footprint radius m
 *   cbf_enabled          (bool,   true)  disable to revert to pure pursuit
 *   cbf_influence_dist   (double, 2.5)   ignore obstacles beyond this m
 *   cbf_forward_cone     (double, 0.2)   cos(angle) threshold for forward cone
 *                                        0.2 = 78°, 0.5 = 60°, 0.0 = 90°
 *   cbf_path_half_width  (double, 0.35)  lateral gate half-width m
 *   cbf_lookahead_t      (double, 0.5)   lookahead time for omega correction s
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

        // ── CBF params ─────────────────────────────────────────────────────
        cbf_alpha_          = declare_parameter<double>("cbf_alpha",           1.0);
        r_robot_            = declare_parameter<double>("r_robot",             0.25);
        cbf_enabled_        = declare_parameter<bool>  ("cbf_enabled",         true);
        cbf_influence_dist_ = declare_parameter<double>("cbf_influence_dist",  2.5);
        cbf_forward_cone_   = declare_parameter<double>("cbf_forward_cone",    0.2);
        cbf_path_half_width_= declare_parameter<double>("cbf_path_half_width", 0.35);
        cbf_lookahead_t_    = declare_parameter<double>("cbf_lookahead_t",     0.5);
        cbf_ttc_threshold_ = declare_parameter<double>("cbf_ttc_threshold", 3.0);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── subscribers ────────────────────────────────────────────────────
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

        obs_sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
            "/tracked_obstacles", rclcpp::QoS(10).best_effort(),
            [this](obstacle_detector::msg::Obstacles::SharedPtr msg){
                obstacles_ = msg->circles;
            });

        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/positioning/status", 10,
            [this](std_msgs::msg::String::SharedPtr msg){
                docking_ = (msg->data.rfind("DOCKING", 0) == 0 ||
                            msg->data.rfind("DOCKED",  0) == 0);
            });
        
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr g){
                current_goal_ = *g;
                have_goal_ = true;
                last_dist_to_goal_ = 999.0;
                last_progress_time_ = now();
            });

        // ── publishers ─────────────────────────────────────────────────────
        left_pub_   = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/target", 1);
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "WaypointController+CBF ready | "
            "r=%.3fm L=%.3fm v=%.2fm/s alpha=%.2f r_robot=%.2fm "
            "cone=%.2f influence=%.1fm",
            wheel_radius_, wheel_base_, linear_speed_,
            cbf_alpha_, r_robot_, cbf_forward_cone_, cbf_influence_dist_);
    }

private:
    // ── params ──────────────────────────────────────────────────────────────
    double wheel_radius_, wheel_base_, max_wheel_rads_;
    double linear_speed_, max_omega_;
    double goal_tolerance_, final_tolerance_;
    double heading_kp_, lookahead_, slow_dist_, min_speed_;
    std::string map_frame_, base_frame_;

    double cbf_alpha_;
    double r_robot_;
    bool   cbf_enabled_;
    double cbf_influence_dist_;
    double cbf_forward_cone_;
    double cbf_path_half_width_;
    double cbf_lookahead_t_;
    double cbf_ttc_threshold_;

    // ── state ────────────────────────────────────────────────────────────────
    double pose_x_{0}, pose_y_{0}, pose_yaw_{0};
    bool   have_pose_{false};
    bool   docking_{false};
    bool   have_goal_{false};
    double last_dist_to_goal_{999.0};
    rclcpp::Time last_progress_time_;
    const double stuck_timeout_s_{1.0};

    geometry_msgs::msg::PoseStamped current_goal_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t wp_idx_{0};
    bool   active_{false};

    std::vector<obstacle_detector::msg::CircleObstacle> obstacles_;

    // ── ROS handles ──────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                           path_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr             obs_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                         status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr               goal_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                           left_pub_, right_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr                  marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr                  goal_pub_;
    rclcpp::TimerBase::SharedPtr                                                   control_timer_;

    // ── utilities ────────────────────────────────────────────────────────────
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

    // ── diff drive IK ────────────────────────────────────────────────────────
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
        l.data = -omega_l;
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

    // ── CBF-QP with directional gating ───────────────────────────────────────
    //
    // Gate 1 — forward cone:
    //   Only obstacles where dot(robot_forward, robot→obs) > cbf_forward_cone_
    //   cbf_forward_cone_ = 0.2 → within 78° of forward heading
    //   obstacles behind or far to side are completely ignored
    //
    // Gate 2 — lateral distance:
    //   Only obstacles whose lateral offset < r_safe + cbf_path_half_width_
    //   obstacle 2m to the side won't be hit → ignore it
    //
    // Gate 3 — influence distance:
    //   Only obstacles within cbf_influence_dist_ meters
    //
    // Directional scaling:
    //   alpha scaled by forward_proj so side obstacles get weaker constraint
    //   directly ahead → full alpha, 60° off → 0.3 * alpha
    //
    // Omega correction:
    //   Lgh_omega includes lookahead term so turning steers away from obstacles
    //   without this CBF only slows down, never steers around
    //
    std::pair<double,double> cbfQP(double v_nom, double omega_nom) {
        if (!cbf_enabled_ || obstacles_.empty())
            return {v_nom, omega_nom};

        const double yaw = pose_yaw_;
        const double px  = pose_x_;
        const double py  = pose_y_;

        // robot forward unit vector in world frame
        const double fx =  std::cos(yaw);
        const double fy =  std::sin(yaw);
        // robot left unit vector (perpendicular)
        const double lx = -std::sin(yaw);
        const double ly =  std::cos(yaw);

        // ── Gate: select only relevant obstacles ──────────────────────────
        struct ActiveObs {
            double ox, oy, ovx, ovy, r_safe;
            double forward_proj;   // how directly ahead (0..1)
        };
        std::vector<ActiveObs> active;

        for (const auto &obs : obstacles_) {
            const double ox  = obs.center.x;
            const double oy  = obs.center.y;
            const double ovx = obs.velocity.x;
            const double ovy = obs.velocity.y;

            const double dx   = ox - px;
            const double dy   = oy - py;
            const double dist = std::hypot(dx, dy) + 1e-6;

            // Gate 3: influence distance
            if (dist > cbf_influence_dist_) continue;

            // forward projection
            const double fwd_proj = (dx * fx + dy * fy) / dist;

            // Gate 1: forward cone
            if (fwd_proj < cbf_forward_cone_) continue;

            // lateral gate
            const double lat_dist = std::fabs(dx * lx + dy * ly);
            const double r_safe   = obs.radius + r_robot_;

            // Gate 2: lateral gate
            if (lat_dist > r_safe + cbf_path_half_width_) continue;

            // Gate 4: TTC gate — obstacle closing speed
            // positive = obstacle moving toward robot, negative = moving away
            const double obs_closing = -(ovx * dx/dist + ovy * dy/dist);
            const double closing_speed = v_nom * fwd_proj + obs_closing;
            const double clearance = std::max(0.0, dist - r_safe);
            const double ttc = (closing_speed > 0.01)
                            ? clearance / closing_speed
                            : 999.0;  // not closing → effectively infinite TTC

            if (ttc > cbf_ttc_threshold_) continue;  // far away and not closing → ignore

            active.push_back({ox, oy, ovx, ovy, r_safe, fwd_proj});
        }

        // no relevant obstacles — return nominal unchanged
        if (active.empty()) return {v_nom, omega_nom};

        const int n_obs = static_cast<int>(active.size());
        const int n_con = n_obs + 4;

        Eigen::MatrixXd G(n_con, 2);
        Eigen::VectorXd h_vec(n_con);

        for (int i = 0; i < n_obs; ++i) {
            const auto &a = active[i];

            // dx, dy: robot minus obstacle (pointing away from obstacle)
            const double dx      = px - a.ox;
            const double dy      = py - a.oy;
            const double dist_sq = dx*dx + dy*dy;

            // CBF: h = dist² - r_safe²
            // h > 0 → safe,  h < 0 → collision
            const double h_val = dist_sq - a.r_safe * a.r_safe;

            // Lgh_v: how forward velocity v changes h
            //   d/dt(dist²) due to v = 2*(robot-obs)·v_robot
            //   v_robot = v * [cos(yaw), sin(yaw)]
            const double Lgh_v = 2.0 * (dx * fx + dy * fy);

            // Lgh_omega: how turning omega changes FUTURE h
            //   Without this term CBF only slows, never steers around obstacle
            //   Lookahead model: robot will be at p + v*t*[cos(yaw), sin(yaw)]
            //   Turning by omega changes heading → changes future direction
            //   Approximate: d(future_dx)/d(omega) = -v*t*sin(yaw),
            //                d(future_dy)/d(omega) =  v*t*cos(yaw)
            //   Lgh_omega = 2*(dx * (-v_nom*cbf_lookahead_t_*sin(yaw))
            //                + dy * ( v_nom*cbf_lookahead_t_*cos(yaw)))
            const double Lgh_omega = 2.0 * (
                dx * (-v_nom * cbf_lookahead_t_ * std::sin(yaw)) +
                dy * ( v_nom * cbf_lookahead_t_ * std::cos(yaw)));

            // Lfh: how obstacle's own velocity changes h
            //   d/dt(dist²) due to obstacle moving = -2*(robot-obs)·v_obs
            const double Lfh = 2.0 * (dx * (-a.ovx) + dy * (-a.ovy));

            // Scale alpha by how directly ahead the obstacle is
            //   directly ahead (fwd_proj=1.0) → full cbf_alpha_
            //   60° off      (fwd_proj=0.5) → 0.5 * cbf_alpha_
            //   at threshold (fwd_proj=0.2) → 0.3 * cbf_alpha_ (minimum)
            const double alpha_scaled = cbf_alpha_ *
                std::max(0.3, a.forward_proj);

            // CBF constraint: Lgh*u >= -alpha*h - Lfh
            // rewritten as:   -Lgh*u <= Lfh + alpha*h
            G(i, 0) = -Lgh_v;
            G(i, 1) = -Lgh_omega;
            h_vec(i) = Lfh + alpha_scaled * h_val;
        }

        // Box constraints
        // -v     <= 0              (v >= 0, no reversing)
        //  v     <= linear_speed_  (speed limit)
        // -omega <= max_omega_     (left turn limit)
        //  omega <= max_omega_     (right turn limit)
        G(n_obs+0, 0) = -1.0; G(n_obs+0, 1) =  0.0; h_vec(n_obs+0) = 0.0;
        G(n_obs+1, 0) =  1.0; G(n_obs+1, 1) =  0.0; h_vec(n_obs+1) = linear_speed_;
        G(n_obs+2, 0) =  0.0; G(n_obs+2, 1) = -1.0; h_vec(n_obs+2) = max_omega_;
        G(n_obs+3, 0) =  0.0; G(n_obs+3, 1) =  1.0; h_vec(n_obs+3) = max_omega_;

        // ── Projected gradient QP solver ──────────────────────────────────
        // min 0.5||u - u_nom||²  s.t. G*u <= h_vec
        //
        // Algorithm: start at u_nom, iteratively project back onto each
        // violated constraint halfspace. Converges in <20 iterations for
        // typical obstacle counts (1-5).
        Eigen::Vector2d u_nom_vec(v_nom, omega_nom);
        Eigen::Vector2d u = u_nom_vec;

        for (int iter = 0; iter < 20; ++iter) {
            bool any_violated = false;
            for (int i = 0; i < n_con; ++i) {
                Eigen::Vector2d g = G.row(i).transpose();
                const double slack = g.dot(u) - h_vec(i);
                if (slack > 1e-6) {
                    any_violated = true;
                    const double g_norm_sq = g.squaredNorm();
                    if (g_norm_sq < 1e-12) continue;
                    u -= (slack / g_norm_sq) * g;
                }
            }
            if (!any_violated) break;
        }

        // clamp v to non-negative
        if (u(0) < 0.0) u(0) = 0.0;

        return {u(0), u(1)};
    }

    // ── path callback ────────────────────────────────────────────────────────
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

    // ── control loop (20 Hz) ─────────────────────────────────────────────────
    void controlLoop() {
        if (docking_) return;
        if (!active_ || waypoints_.empty()) return;

        if (!have_pose_ && !tryTfPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for pose (/amcl_pose or map->base_link TF)");
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
        if (have_goal_ && active_ && !obstacles_.empty()) {
            if (dist_final < last_dist_to_goal_ - 0.05) {
                // making progress
                last_dist_to_goal_  = dist_final;
                last_progress_time_ = now();
            } else {
                double stuck_secs = (now() - last_progress_time_).seconds();
                if (stuck_secs > stuck_timeout_s_) {
                    RCLCPP_WARN(get_logger(),
                        "Stuck for %.1fs with obstacles present — triggering replan",
                        stuck_secs);
                    // republish same goal → A* replans with current dynamic obstacles
                    goal_pub_->publish(current_goal_);
                    last_progress_time_ = now();  // reset timer
                }
            }
        }

        if (dist_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            stopRobot();
            active_ = false;
            return;
        }

        // ── pure pursuit → nominal (v, omega) ────────────────────────────
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

        // ── CBF-QP safety filter ──────────────────────────────────────────
        auto [v_safe, omega_safe] = cbfQP(v, omega);

        if (std::fabs(v_safe - v) > 0.01 ||
            std::fabs(omega_safe - omega) > 0.05) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "CBF active: v %.3f→%.3f  omega %.3f→%.3f",
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