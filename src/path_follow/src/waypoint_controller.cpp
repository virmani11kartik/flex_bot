#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
 * Replanning:
 *   Proactive — obstacle detected on current path → replan immediately
 *   Reactive  — robot stuck for stuck_timeout_s_ → replan as fallback
 *
 * Visualization:
 *   /cbf/visualization (MarkerArray):
 *     Red    disc  — collision boundary  (obs.radius + r_robot)
 *     Orange disc  — CBF active zone     (obs.radius + cbf_influence_dist)
 *     Purple disc  — A* exclusion zone   (obs.radius + astar_clearance_viz)
 *     Yellow arrow — obstacle velocity
 */
class WaypointController : public rclcpp::Node {
public:
    WaypointController() : rclcpp::Node("waypoint_controller") {

        // ── robot geometry ─────────────────────────────────────────────────
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
        cbf_alpha_           = declare_parameter<double>("cbf_alpha",           1.0);
        r_robot_             = declare_parameter<double>("r_robot",             0.25);
        cbf_enabled_         = declare_parameter<bool>  ("cbf_enabled",         true);
        cbf_influence_dist_  = declare_parameter<double>("cbf_influence_dist",  0.5);
        cbf_forward_cone_    = declare_parameter<double>("cbf_forward_cone",    0.2);
        cbf_path_half_width_ = declare_parameter<double>("cbf_path_half_width", 0.30);
        cbf_lookahead_t_     = declare_parameter<double>("cbf_lookahead_t",     0.5);
        cbf_ttc_threshold_   = declare_parameter<double>("cbf_ttc_threshold",   3.0);

        // ── replan params ──────────────────────────────────────────────────
        stuck_timeout_s_     = declare_parameter<double>("stuck_timeout_s",     1.0);
        // astar_clearance_viz must match dynamic_obs_clearance in astar_planner
        // used for: proactive replan intersection check + purple circle viz
        astar_clearance_viz_ = declare_parameter<double>("astar_clearance_viz", 1.9);
        // how many waypoints ahead to check for obstacle intersection
        path_check_horizon_  = declare_parameter<int>   ("path_check_horizon",  30);
        // debounce: minimum seconds between replans
        replan_debounce_s_   = declare_parameter<double>("replan_debounce_s",   1.0);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── subscribers ────────────────────────────────────────────────────
        amcl_sub_ = create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
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
                current_goal_       = *g;
                have_goal_          = true;
                last_dist_to_goal_  = 999.0;
                last_progress_time_ = now();
                last_replan_time_   = now();
            });

        // ── publishers ─────────────────────────────────────────────────────
        left_pub_    = create_publisher<std_msgs::msg::Float64>(
            "/left_wheel/cmd_vel",  1);
        right_pub_   = create_publisher<std_msgs::msg::Float64>(
            "/right_wheel/cmd_vel", 1);
        marker_pub_  = create_publisher<visualization_msgs::msg::Marker>(
            "/controller/target", 1);
        goal_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1);
        cbf_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cbf/visualization", 1);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ controlLoop(); });

        // visualization runs independently — always shows markers
        viz_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this](){ publishCbfMarkers(); });

        RCLCPP_INFO(get_logger(),
            "WaypointController+CBF ready | "
            "v=%.2fm/s alpha=%.2f r_robot=%.2fm influence=%.2fm "
            "astar_clearance=%.2fm",
            linear_speed_, cbf_alpha_, r_robot_,
            cbf_influence_dist_, astar_clearance_viz_);
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

    double stuck_timeout_s_;
    double astar_clearance_viz_;
    int    path_check_horizon_;
    double replan_debounce_s_;

    // ── state ────────────────────────────────────────────────────────────────
    double pose_x_{0}, pose_y_{0}, pose_yaw_{0};
    bool   have_pose_{false};
    bool   docking_{false};
    bool   have_goal_{false};
    double last_dist_to_goal_{999.0};
    rclcpp::Time last_progress_time_;
    rclcpp::Time last_replan_time_;

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr             cbf_viz_pub_;
    rclcpp::TimerBase::SharedPtr                                                   control_timer_;
    rclcpp::TimerBase::SharedPtr                                                   viz_timer_;

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

    // ── proactive replan check ────────────────────────────────────────────────
    // Returns true if any obstacle's purple circle intersects the planned path
    // ahead of the robot within path_check_horizon_ waypoints.
    // Uses astar_clearance_viz_ as the exclusion radius — same value A* uses —
    // so we replan exactly when A* would route around the obstacle.
    bool pathIntersectsObstacle() {
        if (waypoints_.empty() || obstacles_.empty()) return false;

        size_t end = std::min(
            wp_idx_ + static_cast<size_t>(path_check_horizon_),
            waypoints_.size());

        for (const auto& obs : obstacles_) {
            double r_exclusion = obs.radius + astar_clearance_viz_;
            for (size_t i = wp_idx_; i < end; ++i) {
                double d = dist2d(
                    waypoints_[i].pose.position.x,
                    waypoints_[i].pose.position.y,
                    obs.center.x, obs.center.y);
                if (d < r_exclusion) return true;
            }
        }
        return false;
    }

    // ── trigger replan (debounced) ────────────────────────────────────────────
    void triggerReplan(const std::string& reason) {
        double since_last = (now() - last_replan_time_).seconds();
        if (since_last < replan_debounce_s_) return;   // debounce

        RCLCPP_WARN(get_logger(), "Replan triggered: %s", reason.c_str());
        goal_pub_->publish(current_goal_);
        last_replan_time_   = now();
        last_progress_time_ = now();
        last_dist_to_goal_  = 999.0;
    }

    // ── CBF-QP ───────────────────────────────────────────────────────────────
    std::pair<double,double> cbfQP(double v_nom, double omega_nom) {
        if (!cbf_enabled_ || obstacles_.empty())
            return {v_nom, omega_nom};

        const double yaw = pose_yaw_;
        const double px  = pose_x_;
        const double py  = pose_y_;

        const double fx =  std::cos(yaw);
        const double fy =  std::sin(yaw);
        const double lx = -std::sin(yaw);
        const double ly =  std::cos(yaw);

        struct ActiveObs {
            double ox, oy, ovx, ovy, r_safe, forward_proj;
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

            // Gate 1: influence distance
            if (dist > cbf_influence_dist_) continue;

            // Gate 2: forward cone
            const double fwd_proj = (dx * fx + dy * fy) / dist;
            if (fwd_proj < cbf_forward_cone_) continue;

            // Gate 3: lateral gate
            const double lat_dist = std::fabs(dx * lx + dy * ly);
            const double r_safe   = obs.radius + r_robot_;
            if (lat_dist > r_safe + cbf_path_half_width_) continue;

            // Gate 4: TTC gate
            const double obs_closing   = -(ovx * dx/dist + ovy * dy/dist);
            const double closing_speed = v_nom * fwd_proj + obs_closing;
            const double clearance     = std::max(0.0, dist - r_safe);
            const double ttc = (closing_speed > 0.01)
                               ? clearance / closing_speed : 999.0;
            if (ttc > cbf_ttc_threshold_) continue;

            active.push_back({ox, oy, ovx, ovy, r_safe, fwd_proj});
        }

        if (active.empty()) return {v_nom, omega_nom};

        const int n_obs = static_cast<int>(active.size());
        const int n_con = n_obs + 4;

        Eigen::MatrixXd G(n_con, 2);
        Eigen::VectorXd h_vec(n_con);

        for (int i = 0; i < n_obs; ++i) {
            const auto &a = active[i];
            const double dx      = px - a.ox;
            const double dy      = py - a.oy;
            const double dist_sq = dx*dx + dy*dy;
            const double h_val   = dist_sq - a.r_safe * a.r_safe;

            const double Lgh_v = 2.0 * (dx * fx + dy * fy);
            const double Lgh_omega = 2.0 * (
                dx * (-v_nom * cbf_lookahead_t_ * std::sin(yaw)) +
                dy * ( v_nom * cbf_lookahead_t_ * std::cos(yaw)));
            const double Lfh = 2.0 * (dx * (-a.ovx) + dy * (-a.ovy));
            const double alpha_s = cbf_alpha_ * std::max(0.3, a.forward_proj);

            G(i, 0) = -Lgh_v;
            G(i, 1) = -Lgh_omega;
            h_vec(i) = Lfh + alpha_s * h_val;
        }

        G(n_obs+0,0)=-1; G(n_obs+0,1)= 0; h_vec(n_obs+0)=0.0;
        G(n_obs+1,0)= 1; G(n_obs+1,1)= 0; h_vec(n_obs+1)=linear_speed_;
        G(n_obs+2,0)= 0; G(n_obs+2,1)=-1; h_vec(n_obs+2)=max_omega_;
        G(n_obs+3,0)= 0; G(n_obs+3,1)= 1; h_vec(n_obs+3)=max_omega_;

        Eigen::Vector2d u(v_nom, omega_nom);
        for (int iter = 0; iter < 20; ++iter) {
            bool viol = false;
            for (int i = 0; i < n_con; ++i) {
                Eigen::Vector2d g = G.row(i).transpose();
                double slack = g.dot(u) - h_vec(i);
                if (slack > 1e-6) {
                    viol = true;
                    double gn = g.squaredNorm();
                    if (gn < 1e-12) continue;
                    u -= (slack / gn) * g;
                }
            }
            if (!viol) break;
        }
        if (u(0) < 0.0) u(0) = 0.0;
        return {u(0), u(1)};
    }

    // ── path callback ────────────────────────────────────────────────────────
    void onPath(nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Empty path — stopping.");
            stopRobot(); active_ = false; return;
        }
        waypoints_ = msg->poses;
        wp_idx_    = 0;
        active_    = true;
        RCLCPP_INFO(get_logger(), "New path: %zu waypoints.", waypoints_.size());
    }

    size_t lookaheadIndex() {
        for (size_t i = wp_idx_; i < waypoints_.size(); ++i)
            if (dist2d(pose_x_, pose_y_,
                       waypoints_[i].pose.position.x,
                       waypoints_[i].pose.position.y) >= lookahead_)
                return i;
        return waypoints_.size() - 1;
    }

    // ── control loop (20 Hz) ─────────────────────────────────────────────────
    void controlLoop() {
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

        double dist_final = dist2d(pose_x_, pose_y_,
                                   waypoints_.back().pose.position.x,
                                   waypoints_.back().pose.position.y);

        // ── replan logic ─────────────────────────────────────────────────
        if (have_goal_ && active_) {

            // PROACTIVE: obstacle purple circle intersects planned path ahead
            // Triggers before robot even slows down
            if (!obstacles_.empty() && pathIntersectsObstacle()) {
                triggerReplan("obstacle on planned path");
            }
            // REACTIVE fallback: robot stuck despite no path intersection
            // (handles cases where obstacle_detector misses something)
            else {
                if (dist_final < last_dist_to_goal_ - 0.05) {
                    last_dist_to_goal_  = dist_final;
                    last_progress_time_ = now();
                } else if (!obstacles_.empty()) {
                    double stuck = (now() - last_progress_time_).seconds();
                    if (stuck > stuck_timeout_s_) {
                        triggerReplan("robot stuck with obstacles");
                    }
                }
            }
        }

        // check goal reached
        if (dist_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            stopRobot(); active_ = false; return;
        }

        // ── pure pursuit → nominal control ────────────────────────────────
        size_t tidx = lookaheadIndex();
        double tx = waypoints_[tidx].pose.position.x;
        double ty = waypoints_[tidx].pose.position.y;
        publishTargetMarker(tx, ty);

        double desired_yaw = std::atan2(ty - pose_y_, tx - pose_x_);
        double herr        = wrapAngle(desired_yaw - pose_yaw_);
        double omega       = clamp(heading_kp_ * herr, -max_omega_, max_omega_);

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

    // ── target marker ────────────────────────────────────────────────────────
    void publishTargetMarker(double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp    = now();
        m.ns = "controller"; m.id = 0;
        m.type   = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.25;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 1.0f;
        m.lifetime = rclcpp::Duration::from_seconds(0.15);
        marker_pub_->publish(m);
    }

    // ── CBF zone visualization ────────────────────────────────────────────────
    // Runs at 10Hz independently of control loop.
    // Shows all three safety zones for each detected obstacle:
    //   Red    = collision boundary  (obs.radius + r_robot)
    //   Orange = CBF active zone     (obs.radius + cbf_influence_dist)
    //   Purple = A* exclusion zone   (obs.radius + astar_clearance_viz)
    //   Yellow arrow = obstacle velocity
    void publishCbfMarkers() {
        visualization_msgs::msg::MarkerArray arr;

        // clear stale markers from previous tick
        visualization_msgs::msg::Marker del;
        del.header.frame_id = map_frame_;
        del.header.stamp    = now();
        del.ns     = "cbf_zones";
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        visualization_msgs::msg::Marker del2 = del;
        del2.ns = "cbf_velocity";
        arr.markers.push_back(del2);

        int id = 0;
        for (const auto& obs : obstacles_) {

            auto makeDisc = [&](double r,
                                float red, float grn, float blu, float alpha,
                                const std::string& ns)
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id    = map_frame_;
                m.header.stamp       = now();
                m.ns                 = ns;
                m.id                 = id++;
                m.type               = visualization_msgs::msg::Marker::CYLINDER;
                m.action             = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x    = obs.center.x;
                m.pose.position.y    = obs.center.y;
                m.pose.position.z    = 0.01;
                m.pose.orientation.w = 1.0;
                m.scale.x            = r * 2.0;   // diameter
                m.scale.y            = r * 2.0;
                m.scale.z            = 0.01;       // flat disc
                m.color.r = red; m.color.g = grn;
                m.color.b = blu; m.color.a = alpha;
                m.lifetime = rclcpp::Duration::from_seconds(0.2);
                return m;
            };

            // red — collision boundary: robot surface touches obstacle surface
            arr.markers.push_back(
                makeDisc(obs.radius + r_robot_,
                         1.0f, 0.0f, 0.0f, 0.25f, "cbf_zones"));

            // orange — CBF active zone: robot starts slowing here
            arr.markers.push_back(
                makeDisc(obs.radius + cbf_influence_dist_,
                         1.0f, 0.6f, 0.0f, 0.12f, "cbf_zones"));

            // purple — A* exclusion zone: A* never plans nodes inside here
            // astar_clearance_viz must match dynamic_obs_clearance in astar_planner
            arr.markers.push_back(
                makeDisc(obs.radius + astar_clearance_viz_,
                         0.5f, 0.0f, 1.0f, 0.06f, "cbf_zones"));

            // yellow arrow — obstacle velocity direction and magnitude
            double speed = std::hypot(obs.velocity.x, obs.velocity.y);
            if (speed > 0.05) {
                visualization_msgs::msg::Marker arrow;
                arrow.header.frame_id    = map_frame_;
                arrow.header.stamp       = now();
                arrow.ns                 = "cbf_velocity";
                arrow.id                 = id++;
                arrow.type               = visualization_msgs::msg::Marker::ARROW;
                arrow.action             = visualization_msgs::msg::Marker::ADD;
                arrow.pose.position.x    = obs.center.x;
                arrow.pose.position.y    = obs.center.y;
                arrow.pose.position.z    = 0.15;
                double yaw               = std::atan2(obs.velocity.y, obs.velocity.x);
                arrow.pose.orientation.z = std::sin(yaw / 2.0);
                arrow.pose.orientation.w = std::cos(yaw / 2.0);
                arrow.scale.x            = speed * 1.5;  // length ∝ speed
                arrow.scale.y            = 0.06;
                arrow.scale.z            = 0.06;
                arrow.color.r = 1.0f; arrow.color.g = 1.0f;
                arrow.color.b = 0.0f; arrow.color.a = 0.9f;
                arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
                arr.markers.push_back(arrow);
            }
        }

        cbf_viz_pub_->publish(arr);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>());
    rclcpp::shutdown();
    return 0;
}