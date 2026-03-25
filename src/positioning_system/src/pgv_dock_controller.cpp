#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <optional>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * PgvDockController
 *
 * State machine:
 *   IDLE       — waiting for a dock command
 *   NAVIGATING — A* + waypoint controller driving toward tag map pose
 *   DOCKING    — within dock_switch_dist, PGV takes over
 *   DOCKED     — aligned, wheels stopped, event published
 *
 * Topics subscribed:
 *   /pgv/position              (geometry_msgs/PointStamped)
 *   /pgv/tag_id                (std_msgs/Float64)
 *   /pgv/angle_deg             (std_msgs/Float64)
 *   /amcl_pose                 (geometry_msgs/PoseWithCovarianceStamped)
 *   /positioning/go_to_tag     (std_msgs/String)  — "5" → go to tag 5
 *
 * Topics published:
 *   /goal_pose                 (geometry_msgs/PoseStamped)
 *   /left_wheel/cmd_vel        (std_msgs/Float64)
 *   /right_wheel/cmd_vel       (std_msgs/Float64)
 *   /positioning/docked        (std_msgs/String)  — "tag:5" on dock complete
 *   /positioning/status        (std_msgs/String)  — human-readable state
 *   /positioning/markers_viz   (visualization_msgs/MarkerArray) — RViz display
 *
 * Parameters:
 *   markers_file         (string) — path to markers.json
 *   dock_switch_dist_m   (double, 0.5)
 *   dock_x_thresh_mm     (double, 20.0)
 *   dock_y_thresh_mm     (double, 20.0)
 *   dock_angle_thresh_deg(double, 2.0)
 *   dock_linear_speed    (double, 0.06)
 *   dock_angular_speed   (double, 0.3)
 *   wheel_radius         (double, 0.076)
 *   wheel_base           (double, 0.30)
 *   max_wheel_rads       (double, 3.0)
 */
class PgvDockController : public rclcpp::Node {
public:
    PgvDockController() : rclcpp::Node("pgv_dock_controller") {

        std::string default_markers = ament_index_cpp::get_package_share_directory(
            "positioning_system") + "/config/markers.json";
        markers_file_ = declare_parameter<std::string>("markers_file", default_markers);
        dock_switch_dist_    = declare_parameter<double>("dock_switch_dist_m",        0.5);
        dock_x_thresh_       = declare_parameter<double>("dock_x_thresh_mm",         20.0);
        dock_y_thresh_       = declare_parameter<double>("dock_y_thresh_mm",         20.0);
        dock_angle_thresh_   = declare_parameter<double>("dock_angle_thresh_deg",     2.0);
        dock_linear_speed_   = declare_parameter<double>("dock_linear_speed",        0.06);
        dock_angular_speed_  = declare_parameter<double>("dock_angular_speed",       0.3);
        wheel_radius_        = declare_parameter<double>("wheel_radius",              0.076);
        wheel_base_          = declare_parameter<double>("wheel_base",                0.30);
        max_wheel_rads_      = declare_parameter<double>("max_wheel_rads",            3.0);

        loadMarkers();

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
                amcl_x_    = m->pose.pose.position.x;
                amcl_y_    = m->pose.pose.position.y;
                have_amcl_ = true;
            });

        cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/positioning/go_to_tag", 10,
            [this](std_msgs::msg::String::SharedPtr m) { onGoToTag(m->data); });

        // ── Publishers ────────────────────────────────────────────────────────
        goal_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
        left_pub_    = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_   = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        docked_pub_  = create_publisher<std_msgs::msg::String>("/positioning/docked",   1);
        status_pub_  = create_publisher<std_msgs::msg::String>("/positioning/status",   1);

        // Latched MarkerArray so RViz sees markers on connect
        marker_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/positioning/markers_viz",
            rclcpp::QoS(1).transient_local().reliable());

        // Publish markers immediately and every 5s (in case RViz restarts)
        publishRvizMarkers();
        create_wall_timer(std::chrono::seconds(5),
            [this]{ publishRvizMarkers(); });

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]{ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "PgvDockController ready. %zu markers loaded. thresh=%.0fmm/%.0fmm/%.1f°",
            markers_.size(), dock_x_thresh_, dock_y_thresh_, dock_angle_thresh_);
    }

private:
    // ── Tag marker entry ──────────────────────────────────────────────────────
    struct TagMarker {
        int    tag_id;
        double map_x, map_y, map_yaw_deg;
        double pgv_dock_angle_deg;
    };

    enum class State { IDLE, NAVIGATING, DOCKING, DOCKED };
    State state_{State::IDLE};

    // Live PGV + AMCL data
    double pgv_x_m_{0}, pgv_y_m_{0}, pgv_angle_deg_{0};
    int    pgv_tag_id_{0};
    double amcl_x_{0}, amcl_y_{0};
    bool   have_amcl_{false};

    std::optional<TagMarker> target_;

    // Params
    std::string markers_file_;
    double dock_switch_dist_, dock_x_thresh_, dock_y_thresh_, dock_angle_thresh_;
    double dock_linear_speed_, dock_angular_speed_;
    double wheel_radius_, wheel_base_, max_wheel_rads_;

    std::map<int, TagMarker> markers_;

    // ROS handles
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pgv_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pgv_tag_sub_, pgv_angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docked_pub_, status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_viz_pub_;
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
            double s = max_wheel_rads_ / max_abs;
            omega_r *= s; omega_l *= s;
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

    // ── Load markers.json ─────────────────────────────────────────────────────
    void loadMarkers() {
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
            TagMarker mk;
            mk.tag_id             = val["tag_id"].get<int>();
            mk.map_x              = val["map_x"].get<double>();
            mk.map_y              = val["map_y"].get<double>();
            mk.map_yaw_deg        = val["map_yaw_deg"].get<double>();
            mk.pgv_dock_angle_deg = val["pgv_dock_angle_deg"].get<double>();
            markers_[mk.tag_id]   = mk;
            RCLCPP_INFO(get_logger(), "  Marker tag=%d → map(%.3f, %.3f) dock_angle=%.1f°",
                        mk.tag_id, mk.map_x, mk.map_y, mk.pgv_dock_angle_deg);
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu markers.", markers_.size());
    }

    // ── Publish RViz MarkerArray ───────────────────────────────────────────────
    void publishRvizMarkers() {
        if (markers_.empty()) return;

        visualization_msgs::msg::MarkerArray arr;
        int id = 0;

        for (auto& [tag_id, mk] : markers_) {
            const auto stamp = now();

            // Cyan cylinder at tag location
            visualization_msgs::msg::Marker cyl;
            cyl.header.frame_id = "map";
            cyl.header.stamp    = stamp;
            cyl.ns     = "tag_markers";
            cyl.id     = id++;
            cyl.type   = visualization_msgs::msg::Marker::CYLINDER;
            cyl.action = visualization_msgs::msg::Marker::ADD;
            cyl.pose.position.x  = mk.map_x;
            cyl.pose.position.y  = mk.map_y;
            cyl.pose.position.z  = 0.05;
            cyl.pose.orientation.w = 1.0;
            cyl.scale.x = 0.3;
            cyl.scale.y = 0.3;
            cyl.scale.z = 0.1;
            cyl.color.r = 0.0f; cyl.color.g = 0.8f;
            cyl.color.b = 1.0f; cyl.color.a = 0.8f;
            cyl.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(cyl);

            // White text label "TAG 5"
            visualization_msgs::msg::Marker txt;
            txt.header = cyl.header;
            txt.ns     = "tag_labels";
            txt.id     = id++;
            txt.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::msg::Marker::ADD;
            txt.pose.position.x  = mk.map_x;
            txt.pose.position.y  = mk.map_y;
            txt.pose.position.z  = 0.45;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 0.22;
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;
            txt.text    = "TAG " + std::to_string(tag_id);
            txt.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(txt);

            // Yellow arrow showing docking heading
            visualization_msgs::msg::Marker arrow;
            arrow.header = cyl.header;
            arrow.ns     = "tag_arrows";
            arrow.id     = id++;
            arrow.type   = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            const double yaw = mk.map_yaw_deg * M_PI / 180.0;
            arrow.pose.position.x  = mk.map_x;
            arrow.pose.position.y  = mk.map_y;
            arrow.pose.position.z  = 0.05;
            arrow.pose.orientation.z = std::sin(yaw / 2.0);
            arrow.pose.orientation.w = std::cos(yaw / 2.0);
            arrow.scale.x = 0.4;   // length
            arrow.scale.y = 0.06;  // shaft width
            arrow.scale.z = 0.06;
            arrow.color.r = 1.0f; arrow.color.g = 0.8f;
            arrow.color.b = 0.0f; arrow.color.a = 1.0f;
            arrow.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(arrow);
        }

        marker_viz_pub_->publish(arr);
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
            RCLCPP_ERROR(get_logger(), "Tag %d not in markers.json", tag_id);
            publishStatus("ERROR: tag " + tag_str + " not registered");
            return;
        }

        target_ = it->second;
        state_  = State::NAVIGATING;

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp    = now();
        goal.header.frame_id = "map";
        goal.pose.position.x = target_->map_x;
        goal.pose.position.y = target_->map_y;
        goal.pose.position.z = 0.0;
        const double yaw_rad = target_->map_yaw_deg * M_PI / 180.0;
        goal.pose.orientation.z = std::sin(yaw_rad / 2.0);
        goal.pose.orientation.w = std::cos(yaw_rad / 2.0);
        goal_pub_->publish(goal);

        publishStatus("NAVIGATING to tag " + tag_str +
            " → map(" + std::to_string(target_->map_x).substr(0,6) +
            ", "      + std::to_string(target_->map_y).substr(0,6) + ")");
    }

    // ── Control loop ─────────────────────────────────────────────────────────
    void controlLoop() {
        switch (state_) {
            case State::IDLE:      break;
            case State::NAVIGATING: navigatingTick(); break;
            case State::DOCKING:   dockingTick();    break;
            case State::DOCKED:    break;
        }
    }

    void navigatingTick() {
        if (!have_amcl_ || !target_) return;
        double dist = std::hypot(target_->map_x - amcl_x_, target_->map_y - amcl_y_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "NAVIGATING → tag %d | dist=%.3fm", target_->tag_id, dist);
        if (dist < dock_switch_dist_) {
            publishStatus("DOCKING tag " + std::to_string(target_->tag_id));
            state_ = State::DOCKING;
        }
    }

    void dockingTick() {
        if (!target_) return;

        if (pgv_tag_id_ != target_->tag_id) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "DOCKING: waiting for tag %d (seeing %d)",
                target_->tag_id, pgv_tag_id_);
            stopWheels();
            return;
        }

        const double x_mm    = pgv_x_m_ * 1000.0;
        const double y_mm    = pgv_y_m_ * 1000.0;
        const double ang_err = wrapAngle(pgv_angle_deg_ - target_->pgv_dock_angle_deg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
            "DOCKING tag %d | x=%+.1fmm y=%+.1fmm angle_err=%+.2f°",
            target_->tag_id, x_mm, y_mm, ang_err);

        // Docked?
        if (std::fabs(x_mm)    < dock_x_thresh_ &&
            std::fabs(y_mm)    < dock_y_thresh_ &&
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

        double v = 0.0, omega = 0.0;

        // P1: correct heading
        if (std::fabs(ang_err) > dock_angle_thresh_)
            omega = clamp(-ang_err * 0.02, -dock_angular_speed_, dock_angular_speed_);

        // P2: reduce forward/back deviation
        if (std::fabs(y_mm) > dock_y_thresh_)
            v = clamp(y_mm * 0.0001, -dock_linear_speed_, dock_linear_speed_);

        // P3: correct lateral when y is close
        if (std::fabs(y_mm) < dock_y_thresh_ * 2.0 && std::fabs(x_mm) > dock_x_thresh_)
            omega += clamp(-x_mm * 0.005, -dock_angular_speed_, dock_angular_speed_);

        publishWheels(v, omega);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PgvDockController>());
    rclcpp::shutdown();
    return 0;
}
