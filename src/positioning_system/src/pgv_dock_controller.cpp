#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <optional>

// nlohmann/json — header only, install: sudo apt install nlohmann-json3-dev
#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * PgvDockController
 *
 * State machine:
 *   IDLE       — waiting for a dock command
 *   NAVIGATING — A* + waypoint controller is driving toward tag map pose
 *                (we just monitor distance to goal)
 *   DOCKING    — robot is within dock_switch_dist of goal, PGV takes over
 *   DOCKED     — alignment complete, wheels stopped, event published
 *
 * Topics:
 *   SUB  /pgv/position        (geometry_msgs/PointStamped)
 *   SUB  /pgv/tag_id          (std_msgs/Float64)
 *   SUB  /pgv/angle_deg       (std_msgs/Float64)
 *   SUB  /amcl_pose           (geometry_msgs/PoseWithCovarianceStamped)
 *   SUB  /positioning/go_to_tag (std_msgs/String)  — "5" triggers go to tag 5
 *   PUB  /goal_pose           (geometry_msgs/PoseStamped) — sends nav goal
 *   PUB  /left_wheel/cmd_vel  (std_msgs/Float64)
 *   PUB  /right_wheel/cmd_vel (std_msgs/Float64)
 *   PUB  /positioning/docked  (std_msgs/String)  — "tag:5" on dock complete
 *   PUB  /positioning/status  (std_msgs/String)  — human-readable state
 *
 * Parameters:
 *   markers_file       (string) — path to markers.json
 *   dock_switch_dist_m   (double, 0.5)  — switch to PGV docking within this dist
 *   dock_x_thresh_mm     (double, 20.0) — |x offset| < this to be docked
 *   dock_y_thresh_mm     (double, 20.0) — |y offset| < this to be docked
 *   dock_angle_thresh_deg(double, 2.0)  — |angle error| < this to be docked
 *   dock_linear_speed    (double, 0.06) — m/s during docking
 *   dock_angular_speed   (double, 0.3)  — rad/s during docking
 *   wheel_radius         (double, 0.076)
 *   wheel_base           (double, 0.30)
 *   max_wheel_rads       (double, 3.0)
 */
class PgvDockController : public rclcpp::Node {
public:
    PgvDockController() : rclcpp::Node("pgv_dock_controller") {

        // ── Parameters ────────────────────────────────────────────────────────
        markers_file_      = declare_parameter<std::string>("markers_file", "");
        dock_switch_dist_    = declare_parameter<double>("dock_switch_dist_m",    0.5);
        dock_x_thresh_       = declare_parameter<double>("dock_x_thresh_mm",     20.0);
        dock_y_thresh_       = declare_parameter<double>("dock_y_thresh_mm",     20.0);
        dock_angle_thresh_   = declare_parameter<double>("dock_angle_thresh_deg", 2.0);
        dock_linear_speed_   = declare_parameter<double>("dock_linear_speed",    0.06);
        dock_angular_speed_  = declare_parameter<double>("dock_angular_speed",   0.3);
        wheel_radius_        = declare_parameter<double>("wheel_radius",          0.076);
        wheel_base_          = declare_parameter<double>("wheel_base",            0.30);
        max_wheel_rads_      = declare_parameter<double>("max_wheel_rads",        3.0);

        loadmarkers();

        // ── Subscribers ───────────────────────────────────────────────────────
        pgv_pos_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/pgv/position", 10,
            [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
                pgv_x_m_ = m->point.x;
                pgv_y_m_ = m->point.y;
            });

        pgv_tag_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/pgv/tag_id", 10,
            [this](std_msgs::msg::Float64::SharedPtr m) {
                pgv_tag_id_ = static_cast<int>(m->data);
            });

        pgv_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/pgv/angle_deg", 10,
            [this](std_msgs::msg::Float64::SharedPtr m) {
                pgv_angle_deg_ = m->data;
            });

        amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                amcl_x_   = m->pose.pose.position.x;
                amcl_y_   = m->pose.pose.position.y;
                have_amcl_ = true;
            });

        cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/positioning/go_to_tag", 10,
            [this](std_msgs::msg::String::SharedPtr m) { onGoToTag(m->data); });

        // ── Publishers ────────────────────────────────────────────────────────
        goal_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
        left_pub_   = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        docked_pub_ = create_publisher<std_msgs::msg::String>("/positioning/docked",   1);
        status_pub_ = create_publisher<std_msgs::msg::String>("/positioning/status",   1);

        // ── Control loop 20 Hz ────────────────────────────────────────────────
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]{ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "PgvDockController ready. %zu tags loaded. dock_thresh=%.0fmm/%.0fmm/%.1f°",
            markers_.size(), dock_x_thresh_, dock_y_thresh_, dock_angle_thresh_);
    }

private:
    // ── Waypoint entry ────────────────────────────────────────────────────────
    struct TagWaypoint {
        int    tag_id;
        double map_x, map_y, map_yaw_deg;
        double pgv_dock_angle_deg;   // desired PGV angle at docked pose
    };

    // ── State machine ─────────────────────────────────────────────────────────
    enum class State { IDLE, NAVIGATING, DOCKING, DOCKED };
    State state_{State::IDLE};

    // ── Live data ─────────────────────────────────────────────────────────────
    double pgv_x_m_{0}, pgv_y_m_{0}, pgv_angle_deg_{0};
    int    pgv_tag_id_{0};
    double amcl_x_{0}, amcl_y_{0};
    bool   have_amcl_{false};

    // ── Current mission ───────────────────────────────────────────────────────
    std::optional<TagWaypoint> target_;

    // ── Params ────────────────────────────────────────────────────────────────
    std::string markers_file_;
    double dock_switch_dist_, dock_x_thresh_, dock_y_thresh_, dock_angle_thresh_;
    double dock_linear_speed_, dock_angular_speed_;
    double wheel_radius_, wheel_base_, max_wheel_rads_;

    std::map<int, TagWaypoint> markers_;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pgv_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pgv_tag_sub_, pgv_angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docked_pub_, status_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── Utilities ─────────────────────────────────────────────────────────────
    inline double wrapAngle(double a) {
        while (a >  180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    inline double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    void publishWheels(double v, double omega) {
        const double a = wheel_base_ / 2.0;
        double omega_r = (v + omega * a) / wheel_radius_;
        double omega_l = (v - omega * a) / wheel_radius_;
        double max_abs = std::max(std::fabs(omega_r), std::fabs(omega_l));
        if (max_abs > max_wheel_rads_) {
            double scale = max_wheel_rads_ / max_abs;
            omega_r *= scale;
            omega_l *= scale;
        }
        std_msgs::msg::Float64 l, r;
        l.data = omega_l; r.data = omega_r;
        left_pub_->publish(l);
        right_pub_->publish(r);
    }

    void stopWheels() {
        std_msgs::msg::Float64 z; z.data = 0.0;
        left_pub_->publish(z);
        right_pub_->publish(z);
    }

    void publishStatus(const std::string& msg) {
        std_msgs::msg::String s; s.data = msg;
        status_pub_->publish(s);
        RCLCPP_INFO(get_logger(), "[STATUS] %s", msg.c_str());
    }

    // ── Load markers.json ───────────────────────────────────────────────────
    void loadmarkers() {
        if (markers_file_.empty()) {
            RCLCPP_WARN(get_logger(), "markers_file param not set!");
            return;
        }
        std::ifstream f(markers_file_);
        if (!f.is_open()) {
            RCLCPP_WARN(get_logger(), "Cannot open markers file: %s", markers_file_.c_str());
            return;
        }
        json j;
        try { f >> j; } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "JSON parse error: %s", e.what());
            return;
        }
        for (auto& [key, val] : j.items()) {
            TagWaypoint wp;
            wp.tag_id             = val["tag_id"].get<int>();
            wp.map_x              = val["map_x"].get<double>();
            wp.map_y              = val["map_y"].get<double>();
            wp.map_yaw_deg        = val["map_yaw_deg"].get<double>();
            wp.pgv_dock_angle_deg = val["pgv_dock_angle_deg"].get<double>();
            markers_[wp.tag_id] = wp;
            RCLCPP_INFO(get_logger(), "  Tag %d → map(%.3f, %.3f) dock_angle=%.1f°",
                        wp.tag_id, wp.map_x, wp.map_y, wp.pgv_dock_angle_deg);
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu markers.", markers_.size());
    }

    // ── Command handler ───────────────────────────────────────────────────────
    void onGoToTag(const std::string& tag_str) {
        int tag_id = 0;
        try { tag_id = std::stoi(tag_str); }
        catch (...) {
            RCLCPP_ERROR(get_logger(), "Invalid tag ID: '%s'", tag_str.c_str());
            return;
        }

        auto it = markers_.find(tag_id);
        if (it == markers_.end()) {
            RCLCPP_ERROR(get_logger(), "Tag %d not registered in markers.json", tag_id);
            publishStatus("ERROR: tag " + tag_str + " not registered");
            return;
        }

        target_ = it->second;
        state_  = State::NAVIGATING;

        // Send nav goal to A* + waypoint controller
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp    = now();
        goal.header.frame_id = "map";
        goal.pose.position.x = target_->map_x;
        goal.pose.position.y = target_->map_y;
        goal.pose.position.z = 0.0;
        // Encode desired yaw into orientation
        const double yaw_rad = target_->map_yaw_deg * M_PI / 180.0;
        goal.pose.orientation.z = std::sin(yaw_rad / 2.0);
        goal.pose.orientation.w = std::cos(yaw_rad / 2.0);
        goal_pub_->publish(goal);

        publishStatus("NAVIGATING to tag " + tag_str +
                      " → map(" + std::to_string(target_->map_x).substr(0,6) +
                      ", " + std::to_string(target_->map_y).substr(0,6) + ")");
    }

    // ── Main control loop ─────────────────────────────────────────────────────
    void controlLoop() {
        switch (state_) {
            case State::IDLE:
                break;

            case State::NAVIGATING:
                navigatingTick();
                break;

            case State::DOCKING:
                dockingTick();
                break;

            case State::DOCKED:
                break;
        }
    }

    // NAVIGATING — monitor distance, switch to DOCKING when close enough
    void navigatingTick() {
        if (!have_amcl_ || !target_) return;

        double dx   = target_->map_x - amcl_x_;
        double dy   = target_->map_y - amcl_y_;
        double dist = std::hypot(dx, dy);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "NAVIGATING → tag %d | dist=%.3fm", target_->tag_id, dist);

        if (dist < dock_switch_dist_) {
            RCLCPP_INFO(get_logger(),
                "Within %.2fm of tag %d — switching to PGV docking mode",
                dock_switch_dist_, target_->tag_id);

            // Tell waypoint controller to stop by publishing empty path
            // (waypoint_controller stops on empty path)
            publishStatus("DOCKING tag " + std::to_string(target_->tag_id));
            state_ = State::DOCKING;
        }
    }

    // DOCKING — use PGV deviations to align precisely
    void dockingTick() {
        if (!target_) return;

        // Wait for the correct tag to be visible
        if (pgv_tag_id_ != target_->tag_id) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "DOCKING: waiting for tag %d (seeing %d)",
                target_->tag_id, pgv_tag_id_);
            stopWheels();
            return;
        }

        // PGV deviations in mm
        const double x_mm    = pgv_x_m_ * 1000.0;
        const double y_mm    = pgv_y_m_ * 1000.0;
        const double ang_err = wrapAngle(pgv_angle_deg_ - target_->pgv_dock_angle_deg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
            "DOCKING tag %d | x=%+.1fmm y=%+.1fmm angle_err=%+.2f°",
            target_->tag_id, x_mm, y_mm, ang_err);

        // Check if docked
        if (std::fabs(x_mm)  < dock_x_thresh_ &&
            std::fabs(y_mm)  < dock_y_thresh_ &&
            std::fabs(ang_err) < dock_angle_thresh_) {
            stopWheels();
            state_ = State::DOCKED;
            std::string event = "tag:" + std::to_string(target_->tag_id);
            std_msgs::msg::String msg; msg.data = event;
            docked_pub_->publish(msg);
            publishStatus("DOCKED at " + event);
            RCLCPP_INFO(get_logger(), "✓ Docked at tag %d", target_->tag_id);
            return;
        }

        // ── Docking control ───────────────────────────────────────────────────
        // The PGV frame: x = lateral deviation, y = forward deviation
        // We drive to reduce y (forward), then correct x (lateral via rotation),
        // then correct angle.

        double v     = 0.0;
        double omega = 0.0;

        // Priority 1: correct heading angle
        if (std::fabs(ang_err) > dock_angle_thresh_) {
            omega = clamp(-ang_err * 0.02, -dock_angular_speed_, dock_angular_speed_);
        }

        // Priority 2: drive forward/backward to reduce y deviation
        if (std::fabs(y_mm) > dock_y_thresh_) {
            // y_mm > 0 means robot is behind tag centre → drive forward
            v = clamp(y_mm * 0.0001, -dock_linear_speed_, dock_linear_speed_);
        }

        // Priority 3: correct lateral via slight rotation if y is close
        if (std::fabs(y_mm) < dock_y_thresh_ * 2.0 && std::fabs(x_mm) > dock_x_thresh_) {
            // x_mm > 0 means robot is to the right of tag → turn left (positive omega)
            omega += clamp(-x_mm * 0.005, -dock_angular_speed_, dock_angular_speed_);
        }

        publishWheels(v, omega);
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    double wheel_radius_, wheel_base_, max_wheel_rads_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PgvDockController>());
    rclcpp::shutdown();
    return 0;
}
