#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>

/**
 * WaypointController — Differential Drive with Local Obstacle Avoidance
 *
 * State machine:
 *   FOLLOWING   — pure-pursuit along A* path
 *   WAITING     — obstacle in corridor, holding position
 *   REPLANNING  — obstacle persisted > obstacle_wait_s, re-published goal to A*
 *
<<<<<<< HEAD
 * State source (priority order):
 *   1. /amcl_pose  (PoseWithCovarianceStamped) — AMCL localization
 *   2. map->base_link TF                        — fallback
 *
 * Outputs:
 *   /left_wheel/cmd_vel   (std_msgs/Float64)  rad/s
 *   /right_wheel/cmd_vel  (std_msgs/Float64)  rad/s
 *
 * Parameters (existing):
 *   wheel_radius    (double, 0.076)   m
 *   wheel_base      (double, 0.50)    m   full track width
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
 *
 * Parameters (new — obstacle avoidance):
 *   scan_topic           (string, "/scan")
 *   local_map_radius     (double, 2.0)    m  half-size of local grid
 *   local_map_resolution (double, 0.05)   m/cell
 *   obstacle_wait_s      (double, 3.0)    s  wait before replan
 *   corridor_width       (double, 0.35)   m  half robot width for path check
 *   inflation_radius     (double, 0.25)   m  inflate each scan hit
=======
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
>>>>>>> 26c052e (cbf connstraints loosen)
 */
class WaypointController : public rclcpp::Node {
public:
    WaypointController() : rclcpp::Node("waypoint_controller") {

        // ── Existing parameters ──────────────────────────────────────────────
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

<<<<<<< HEAD
        // ── New obstacle-avoidance parameters ────────────────────────────────
        scan_topic_           = declare_parameter<std::string>("scan_topic",           "/scan_fullframe");
        local_map_radius_     = declare_parameter<double>("local_map_radius",          2.0);
        local_map_resolution_ = declare_parameter<double>("local_map_resolution",      0.05);
        obstacle_wait_s_      = declare_parameter<double>("obstacle_wait_s",           3.0);
        corridor_width_       = declare_parameter<double>("corridor_width",            0.35);
        inflation_radius_     = declare_parameter<double>("inflation_radius",          0.25);
=======
        // ── CBF params ─────────────────────────────────────────────────────
        cbf_alpha_          = declare_parameter<double>("cbf_alpha",           1.0);
        r_robot_            = declare_parameter<double>("r_robot",             0.25);
        cbf_enabled_        = declare_parameter<bool>  ("cbf_enabled",         true);
        cbf_influence_dist_ = declare_parameter<double>("cbf_influence_dist",  2.5);
        cbf_forward_cone_   = declare_parameter<double>("cbf_forward_cone",    0.2);
        cbf_path_half_width_= declare_parameter<double>("cbf_path_half_width", 0.35);
        cbf_lookahead_t_    = declare_parameter<double>("cbf_lookahead_t",     0.5);
>>>>>>> 26c052e (cbf connstraints loosen)

        // ── Initialise local occupancy grid ──────────────────────────────────
        initLocalMap();

        // ── TF ───────────────────────────────────────────────────────────────
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

<<<<<<< HEAD
        // ── Subscriptions ────────────────────────────────────────────────────
=======
        // ── subscribers ────────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
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

<<<<<<< HEAD
        // NEW: laser scan subscriber
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ onScan(msg); });

        // ── Publishers ───────────────────────────────────────────────────────
        left_pub_   = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/target", 1);

        // NEW: re-publish goal to A* for replanning
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
=======
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

        // ── publishers ─────────────────────────────────────────────────────
        left_pub_   = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/target", 1);
>>>>>>> 26c052e (cbf connstraints loosen)

        // NEW: publish local map for debugging in RViz
        local_map_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/local_map", 1);

        // ── Control timer (20 Hz) ────────────────────────────────────────────
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
<<<<<<< HEAD
            "WaypointController ready | r=%.3fm L=%.3fm max=%.1frad/s v=%.2fm/s",
            wheel_radius_, wheel_base_, max_wheel_rads_, linear_speed_);
        RCLCPP_INFO(get_logger(),
            "Obstacle avoidance: scan=%s radius=%.1fm res=%.2fm wait=%.1fs",
            scan_topic_.c_str(), local_map_radius_, local_map_resolution_, obstacle_wait_s_);
    }

private:
    // ── State machine ─────────────────────────────────────────────────────────
    enum class State { FOLLOWING, WAITING, REPLANNING };
    State state_{State::FOLLOWING};

    // ── Existing parameters ───────────────────────────────────────────────────
=======
            "WaypointController+CBF ready | "
            "r=%.3fm L=%.3fm v=%.2fm/s alpha=%.2f r_robot=%.2fm "
            "cone=%.2f influence=%.1fm",
            wheel_radius_, wheel_base_, linear_speed_,
            cbf_alpha_, r_robot_, cbf_forward_cone_, cbf_influence_dist_);
    }

private:
    // ── params ──────────────────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
    double wheel_radius_, wheel_base_, max_wheel_rads_;
    double linear_speed_, max_omega_;
    double goal_tolerance_, final_tolerance_;
    double heading_kp_, lookahead_, slow_dist_, min_speed_;
    std::string map_frame_, base_frame_;

<<<<<<< HEAD
    // ── New parameters ────────────────────────────────────────────────────────
    std::string scan_topic_;
    double local_map_radius_;
    double local_map_resolution_;
    double obstacle_wait_s_;
    double corridor_width_;
    double inflation_radius_;

    // ── Pose state ────────────────────────────────────────────────────────────
=======
    double cbf_alpha_;
    double r_robot_;
    bool   cbf_enabled_;
    double cbf_influence_dist_;
    double cbf_forward_cone_;
    double cbf_path_half_width_;
    double cbf_lookahead_t_;

    // ── state ────────────────────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
    double pose_x_{0}, pose_y_{0}, pose_yaw_{0};
    bool   have_pose_{false};
    bool   docking_{false};

    // ── Path state ────────────────────────────────────────────────────────────
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t wp_idx_{0};
    bool   active_{false};
    geometry_msgs::msg::PoseStamped current_goal_; // stored for replanning

    // ── Local occupancy grid (map frame) ──────────────────────────────────────
    // A flat 2D grid.  origin_x_/origin_y_ = bottom-left corner in map frame.
    // Cells store the last time they were marked occupied (nanoseconds).
    // 0 = free.
    struct LocalMap {
        std::vector<rclcpp::Time> cells; // size = width * height
        int    width{0}, height{0};
        double resolution{0.05};
        double origin_x{0}, origin_y{0}; // bottom-left corner, map frame
    } lmap_;

<<<<<<< HEAD
    std::string laser_frame_{"laser"};  // filled from scan header

    // ── Timing for state transitions ──────────────────────────────────────────
    rclcpp::Time obstacle_detected_time_;
    bool         obstacle_timer_running_{false};

    // ── ROS handles ───────────────────────────────────────────────────────────
=======
    // ── ROS handles ──────────────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
<<<<<<< HEAD
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr   path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr         left_pub_, right_pub_;
=======
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr    path_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obs_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  status_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    left_pub_, right_pub_;
>>>>>>> 26c052e (cbf connstraints loosen)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_map_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

<<<<<<< HEAD
    // =========================================================================
    // Local map helpers
    // =========================================================================

    void initLocalMap() {
        int cells = (int)std::ceil(2.0 * local_map_radius_ / local_map_resolution_);
        lmap_.width      = cells;
        lmap_.height     = cells;
        lmap_.resolution = local_map_resolution_;
        lmap_.cells.assign((size_t)(cells * cells), rclcpp::Time(0, 0, RCL_ROS_TIME));
        lmap_.origin_x   = 0.0;
        lmap_.origin_y   = 0.0;
        RCLCPP_INFO(get_logger(), "Local map initialised: %d x %d cells (%.1fm radius)",
                    cells, cells, local_map_radius_);
    }

    // Re-centre the local map around the robot's current position.
    // Called each control cycle so the map always covers the area ahead.
    void recentreLocalMap() {
        lmap_.origin_x = pose_x_ - local_map_radius_;
        lmap_.origin_y = pose_y_ - local_map_radius_;
    }

    // World → local map cell index.  Returns false if outside grid.
    bool worldToLocal(double wx, double wy, int &cx, int &cy) const {
        cx = (int)std::floor((wx - lmap_.origin_x) / lmap_.resolution);
        cy = (int)std::floor((wy - lmap_.origin_y) / lmap_.resolution);
        return cx >= 0 && cy >= 0 && cx < lmap_.width && cy < lmap_.height;
    }

    // Mark a cell and inflate a radius around it.
    void markCell(int cx, int cy, const rclcpp::Time &stamp) {
        int R = (int)std::ceil(inflation_radius_ / lmap_.resolution);
        for (int dy = -R; dy <= R; ++dy) {
            for (int dx = -R; dx <= R; ++dx) {
                if (dx*dx + dy*dy > R*R) continue;
                int nx = cx + dx, ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= lmap_.width || ny >= lmap_.height) continue;
                lmap_.cells[(size_t)(ny * lmap_.width + nx)] = stamp;
            }
        }
    }

    // Ray-cast from (x0,y0) to (x1,y1) in world frame, clearing cells along
    // the ray and marking the endpoint.  This is the key to handling dynamic
    // obstacles: if the lidar can see through a cell, it is free.
    void raycastAndMark(double x0, double y0, double x1, double y1,
                        const rclcpp::Time &stamp) {
        // Bresenham in local map coordinates
        int lx0, ly0, lx1, ly1;
        if (!worldToLocal(x0, y0, lx0, ly0)) return;
        if (!worldToLocal(x1, y1, lx1, ly1)) {
            // Endpoint outside grid — still clear the ray up to the boundary
            lx1 = std::max(0, std::min(lmap_.width  - 1, lx1));
            ly1 = std::max(0, std::min(lmap_.height - 1, ly1));
        }

        int dx =  std::abs(lx1 - lx0), sx = lx0 < lx1 ? 1 : -1;
        int dy = -std::abs(ly1 - ly0), sy = ly0 < ly1 ? 1 : -1;
        int err = dx + dy;
        int x = lx0, y = ly0;

        while (true) {
            bool at_endpoint = (x == lx1 && y == ly1);
            if (at_endpoint) {
                // Only mark endpoint if it is a valid hit (inside grid)
                int ex, ey;
                if (worldToLocal(x1, y1, ex, ey)) {
                    markCell(ex, ey, stamp);
                }
                break;
            }
            // Clear cells along the ray (free space)
            lmap_.cells[(size_t)(y * lmap_.width + x)] = rclcpp::Time(0, 0, RCL_ROS_TIME);

            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    }

    // Is cell (cx,cy) currently occupied?
    // A cell is occupied if it was marked within the last obstacle_wait_s_ * 2 seconds.
    // (We use a generous window here — the replan timeout is the real gate.)
    bool cellOccupied(int cx, int cy) const {
        if (cx < 0 || cy < 0 || cx >= lmap_.width || cy >= lmap_.height) return false;
        const auto &t = lmap_.cells[(size_t)(cy * lmap_.width + cx)];
        if (t.nanoseconds() == 0) return false;
        double age = (now() - t).seconds();
        return age < (obstacle_wait_s_ * 2.0);
    }

    // =========================================================================
    // Obstacle checks
    // =========================================================================

    // Check a straight corridor from robot to lookahead target for occupied cells.
    // corridor_width_ is the half-width perpendicular to the path.
    bool pathClearAhead(double tx, double ty) const {
        double dx = tx - pose_x_;
        double dy = ty - pose_y_;
        double seg_len = std::hypot(dx, dy);
        if (seg_len < 1e-3) return true;

        // Unit vector along path and perpendicular
        double ux = dx / seg_len, uy = dy / seg_len;
        double px = -uy,          py =  ux;  // perpendicular

        int steps = (int)(seg_len / lmap_.resolution) + 1;
        int w_steps = (int)(corridor_width_ / lmap_.resolution) + 1;

        for (int s = 0; s <= steps; ++s) {
            double t = (double)s / steps;
            double cx_w = pose_x_ + t * dx;
            double cy_w = pose_y_ + t * dy;

            for (int w = -w_steps; w <= w_steps; ++w) {
                double wx = cx_w + w * lmap_.resolution * px;
                double wy = cy_w + w * lmap_.resolution * py;
                int lcx, lcy;
                if (!worldToLocal(wx, wy, lcx, lcy)) continue;
                if (cellOccupied(lcx, lcy)) return false;
            }
        }
        return true;
    }

    // =========================================================================
    // Scan callback — update local map via ray casting
    // =========================================================================

    void onScan(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!have_pose_) return;

        laser_frame_ = msg->header.frame_id;

        // Get transform from laser frame to map frame
        geometry_msgs::msg::TransformStamped laser_to_map;
        try {
            laser_to_map = tf_buffer_->lookupTransform(
                map_frame_, laser_frame_,
                tf2_ros::fromMsg(msg->header.stamp),
                std::chrono::milliseconds(50));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Scan TF failed: %s", ex.what());
            return;
        }

        // Laser origin in map frame
        double ox = laser_to_map.transform.translation.x;
        double oy = laser_to_map.transform.translation.y;
        double oyaw = quatToYaw(
            laser_to_map.transform.rotation.x,
            laser_to_map.transform.rotation.y,
            laser_to_map.transform.rotation.z,
            laser_to_map.transform.rotation.w);

        rclcpp::Time stamp = msg->header.stamp;

        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
            float r = msg->ranges[i];

            // Skip invalid ranges
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
                angle += msg->angle_increment;
                continue;
            }

            // Hit point in map frame
            double hit_angle = oyaw + angle;
            double hx = ox + r * std::cos(hit_angle);
            double hy = oy + r * std::sin(hit_angle);

            raycastAndMark(ox, oy, hx, hy, stamp);
        }

        publishLocalMapMarker();
    }

    // =========================================================================
    // Utilities (unchanged from original)
    // =========================================================================

=======
    // ── utilities ────────────────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
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

<<<<<<< HEAD
    // ── Diff-drive IK (unchanged) ─────────────────────────────────────────────
=======
    // ── diff drive IK ────────────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
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
<<<<<<< HEAD
        }
=======

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

            // vector from robot to obstacle
            const double dx = ox - px;
            const double dy = oy - py;
            const double dist = std::hypot(dx, dy) + 1e-6;

            // Gate 3: influence distance
            if (dist > cbf_influence_dist_) continue;

            // forward projection: +1 = directly ahead, -1 = directly behind
            const double fwd_proj = (dx * fx + dy * fy) / dist;

            // Gate 1: forward cone — ignore behind and far-side obstacles
            if (fwd_proj < cbf_forward_cone_) continue;

            // lateral distance: how far to the side the obstacle is
            const double lat_dist = std::fabs(dx * lx + dy * ly);
            const double r_safe   = obs.radius + r_robot_;

            // Gate 2: lateral gate — only if obstacle is in robot's swept path
            if (lat_dist > r_safe + cbf_path_half_width_) continue;

            active.push_back({ox, oy,
                              obs.velocity.x, obs.velocity.y,
                              r_safe, fwd_proj});
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
>>>>>>> 26c052e (cbf connstraints loosen)
    }

    // ── Pure-pursuit lookahead index (unchanged) ──────────────────────────────
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

<<<<<<< HEAD
    // =========================================================================
    // Path callback
    // =========================================================================

    void onPath(nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Empty path received — stopping.");
            stopRobot();
            active_ = false;
            state_  = State::FOLLOWING;
            return;
        }
        waypoints_ = msg->poses;
        wp_idx_    = 0;
        active_    = true;
        state_     = State::FOLLOWING;
        obstacle_timer_running_ = false;

        // Store the final goal pose for replanning
        current_goal_ = msg->poses.back();

        RCLCPP_INFO(get_logger(), "New path: %zu waypoints. State → FOLLOWING", waypoints_.size());
    }

    // =========================================================================
    // Control loop (20 Hz) — state machine
    // =========================================================================

=======
    // ── control loop (20 Hz) ─────────────────────────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
    void controlLoop() {
        if (docking_) return;
        if (!active_ || waypoints_.empty()) return;

        if (!have_pose_ && !tryTfPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for pose (/amcl_pose or map->base_link TF)");
            return;
        }

        // Re-centre local map around robot every cycle
        recentreLocalMap();

        // ── Advance past reached waypoints ────────────────────────────────────
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

        // ── Check final goal ──────────────────────────────────────────────────
        double dist_final = dist2d(pose_x_, pose_y_,
                                   waypoints_.back().pose.position.x,
                                   waypoints_.back().pose.position.y);
        if (dist_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            stopRobot();
            active_ = false;
            state_  = State::FOLLOWING;
            return;
        }

<<<<<<< HEAD
        // ── Look-ahead target ─────────────────────────────────────────────────
=======
        // ── pure pursuit → nominal (v, omega) ────────────────────────────
>>>>>>> 26c052e (cbf connstraints loosen)
        size_t tidx = lookaheadIndex();
        double tx = waypoints_[tidx].pose.position.x;
        double ty = waypoints_[tidx].pose.position.y;
        publishTargetMarker(tx, ty);

        // =====================================================================
        // STATE MACHINE
        // =====================================================================

        switch (state_) {

        // ── FOLLOWING ────────────────────────────────────────────────────────
        case State::FOLLOWING: {
            if (!pathClearAhead(tx, ty)) {
                // Obstacle detected — transition to WAITING
                state_ = State::WAITING;
                obstacle_detected_time_ = now();
                obstacle_timer_running_ = true;
                stopRobot();
                RCLCPP_WARN(get_logger(),
                    "Obstacle detected ahead! State → WAITING (%.1fs timeout)",
                    obstacle_wait_s_);
                return;
            }

<<<<<<< HEAD
            // Path is clear — normal pure-pursuit
            double desired_yaw = std::atan2(ty - pose_y_, tx - pose_x_);
            double herr        = wrapAngle(desired_yaw - pose_yaw_);
            double omega       = clamp(heading_kp_ * herr, -max_omega_, max_omega_);

            double v = linear_speed_;
            if (dist_final < slow_dist_)
                v *= dist_final / slow_dist_;
            v *= std::max(0.0, std::cos(herr));
            v  = std::max(min_speed_, v);
            if (std::fabs(herr) > M_PI / 2.0) v = 0.0;

            publishWheelCmds(v, omega);
            break;
=======
        // ── CBF-QP safety filter ──────────────────────────────────────────
        auto [v_safe, omega_safe] = cbfQP(v, omega);

        if (std::fabs(v_safe - v) > 0.01 ||
            std::fabs(omega_safe - omega) > 0.05) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "CBF active: v %.3f→%.3f  omega %.3f→%.3f",
                v, v_safe, omega, omega_safe);
>>>>>>> 26c052e (cbf connstraints loosen)
        }

        // ── WAITING ──────────────────────────────────────────────────────────
        case State::WAITING: {
            stopRobot();

            if (pathClearAhead(tx, ty)) {
                // Obstacle gone — resume following from same wp_idx_
                state_ = State::FOLLOWING;
                obstacle_timer_running_ = false;
                RCLCPP_INFO(get_logger(), "Path cleared! State → FOLLOWING (resuming wp %zu)", wp_idx_);
                return;
            }

            // Check timeout
            double waited = (now() - obstacle_detected_time_).seconds();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Waiting for obstacle to clear... (%.1f / %.1f s)", waited, obstacle_wait_s_);

            if (waited >= obstacle_wait_s_) {
                // Obstacle persisted — trigger replan
                state_ = State::REPLANNING;
                RCLCPP_WARN(get_logger(),
                    "Obstacle did not clear after %.1fs. State → REPLANNING", obstacle_wait_s_);
                triggerReplan();
            }
            break;
        }

        // ── REPLANNING ───────────────────────────────────────────────────────
        case State::REPLANNING: {
            // Hold still while waiting for A* to return a new path.
            // onPath() will reset state to FOLLOWING when new path arrives.
            stopRobot();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Replanning... waiting for new path from A*");
            break;
        }

        } // switch
    }

    // =========================================================================
    // Replan: re-publish the stored goal so A* runs again from current pose
    // =========================================================================

    void triggerReplan() {
        // A* uses TF for start pose, so we only need to re-publish the goal.
        // astar_search.cpp's onGoal() callback re-triggers planPath() automatically.
        geometry_msgs::msg::PoseStamped goal_msg = current_goal_;
        goal_msg.header.stamp = now();
        goal_pub_->publish(goal_msg);
        RCLCPP_INFO(get_logger(), "Re-published goal (%.2f, %.2f) for A* replan",
                    goal_msg.pose.position.x, goal_msg.pose.position.y);
    }

    // =========================================================================
    // Debug visualisation
    // =========================================================================

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

    // Publish occupied local map cells as POINTS marker for RViz
    void publishLocalMapMarker() {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp    = now();
        m.ns = "local_map"; m.id = 1;
        m.type   = visualization_msgs::msg::Marker::POINTS;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = lmap_.resolution * 0.9;
        m.scale.y = lmap_.resolution * 0.9;
        m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 0.6f;
        m.lifetime = rclcpp::Duration::from_seconds(0.2);

        for (int cy = 0; cy < lmap_.height; ++cy) {
            for (int cx = 0; cx < lmap_.width; ++cx) {
                if (!cellOccupied(cx, cy)) continue;
                geometry_msgs::msg::Point p;
                p.x = lmap_.origin_x + (cx + 0.5) * lmap_.resolution;
                p.y = lmap_.origin_y + (cy + 0.5) * lmap_.resolution;
                p.z = 0.05;
                m.points.push_back(p);
            }
        }
        local_map_pub_->publish(m);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointController>());
    rclcpp::shutdown();
    return 0;
}