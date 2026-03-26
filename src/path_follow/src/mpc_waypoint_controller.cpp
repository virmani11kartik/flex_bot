#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <obstacle_detector/msg/obstacles.hpp>
#include <obstacle_detector/msg/circle_obstacle.hpp>

#include <Eigen/Dense>
#include <osqp/osqp.h>
#include <cmath>
#include <vector>
#include <limits>
#include <memory>


/**
 * 
cd ~
git clone https://github.com/osqp/osqp.git
cd osqp
git checkout v1.0.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
sudo cmake --install .
sudo ldconfig
 */

/**
 * MpcWaypointController — Differential Drive with MPC + CBF hard constraints
 *
 * Architecture:
 *   A* path (reference) + tracked obstacles → MPC-QP → (v, omega) → wheels
 *
 * MPC formulation (N=20 horizon, dt=0.1s = 2s lookahead):
 *
 *   State:   [x, y, theta]  at each of N steps
 *   Control: [v, omega]     at each of N steps
 *   Decision variables: u = [v_0, w_0, v_1, w_1, ..., v_{N-1}, w_{N-1}]  (2N vars)
 *
 *   Minimize:
 *     sum_{t=0}^{N-1}  w_pos   * ||p(t) - p_ref(t)||^2
 *                    + w_theta * angle_error(theta(t), theta_ref(t))^2
 *                    + w_v     * v(t)^2
 *                    + w_dv    * (v(t) - v(t-1))^2
 *                    + w_dw    * (w(t) - w(t-1))^2
 *
 *   Subject to:
 *     [1] Linearised diff-drive dynamics (propagated from current state)
 *     [2] Control box:  0 <= v(t) <= v_max,  -w_max <= omega(t) <= w_max
 *     [3] CBF hard constraint per obstacle per timestep:
 *             h_i(t) = ||p(t) - p_obs_i(t)||^2 - r_safe_i^2
 *             Lgh_v * v(t) + Lgh_w * omega(t) >= -alpha * h_i(t) - Lfh_i(t)
 *             (only for obstacles in forward cone and within influence distance)
 *
 *   Fallback: if QP infeasible → run pure CBF filter (same as waypoint_controller)
 *
 * Topics subscribed:
 *   /amcl_pose                (geometry_msgs/PoseWithCovarianceStamped)
 *   /astar/path               (nav_msgs/Path)
 *   /tracked_obstacles        (obstacle_detector/Obstacles)
 *   /positioning/status       (std_msgs/String)  — docking handoff
 *
 * Topics published:
 *   /left_wheel/cmd_vel       (std_msgs/Float64)  rad/s
 *   /right_wheel/cmd_vel      (std_msgs/Float64)  rad/s
 *   /controller/target        (visualization_msgs/Marker)
 *   /mpc/status               (std_msgs/String)   — "MPC" or "CBF_FALLBACK"
 *
 * Parameters:
 *   All robot geometry params identical to waypoint_controller.yaml
 *   Additional MPC params below.
 */

// ── small helpers ────────────────────────────────────────────────────────────
static inline double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
static inline double dist2d(double ax, double ay, double bx, double by) {
    return std::hypot(bx - ax, by - ay);
}
static double quatToYaw(double x, double y, double z, double w) {
    return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

// ── OSQP dense-to-CSC helper ─────────────────────────────────────────────────
// OSQP requires upper-triangular CSC format for P, full CSC for A.
struct CscMatrix {
    std::vector<OSQPFloat> data;
    std::vector<OSQPInt>   row_ind;
    std::vector<OSQPInt>   col_ptr;
    OSQPInt m{0}, n{0}, nnz{0};
};

// Build upper-triangular CSC from dense Eigen matrix (symmetric)
static CscMatrix toCscUpper(const Eigen::MatrixXd& M) {
    CscMatrix csc;
    int n = M.cols();
    csc.m = csc.n = n;
    csc.col_ptr.resize(n + 1, 0);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i <= j; ++i)
            if (std::abs(M(i,j)) > 1e-12) {
                csc.data.push_back(static_cast<OSQPFloat>(M(i,j)));
                csc.row_ind.push_back(static_cast<OSQPInt>(i));
                csc.col_ptr[j+1]++;
            }
    for (int j = 0; j < n; ++j)
        csc.col_ptr[j+1] += csc.col_ptr[j];
    csc.nnz = static_cast<OSQPInt>(csc.data.size());
    return csc;
}

// Build full CSC from dense Eigen matrix (constraint matrix)
static CscMatrix toCscFull(const Eigen::MatrixXd& M) {
    CscMatrix csc;
    csc.m = M.rows(); csc.n = M.cols();
    csc.col_ptr.resize(csc.n + 1, 0);
    for (int j = 0; j < M.cols(); ++j)
        for (int i = 0; i < M.rows(); ++i)
            if (std::abs(M(i,j)) > 1e-12) {
                csc.data.push_back(static_cast<OSQPFloat>(M(i,j)));
                csc.row_ind.push_back(static_cast<OSQPInt>(i));
                csc.col_ptr[j+1]++;
            }
    for (int j = 0; j < M.cols(); ++j)
        csc.col_ptr[j+1] += csc.col_ptr[j];
    csc.nnz = static_cast<OSQPInt>(csc.data.size());
    return csc;
}

// ── main node ────────────────────────────────────────────────────────────────
class MpcWaypointController : public rclcpp::Node {
public:
    MpcWaypointController() : rclcpp::Node("mpc_waypoint_controller") {

        // ── robot geometry (same as waypoint_controller) ──────────────────
        wheel_radius_   = declare_parameter<double>("wheel_radius",    0.0765);
        wheel_base_     = declare_parameter<double>("wheel_base",      0.50);
        max_wheel_rads_ = declare_parameter<double>("max_wheel_rads",  3.0);
        v_max_          = declare_parameter<double>("linear_speed",    0.15);
        w_max_          = declare_parameter<double>("max_omega",       0.8);
        goal_tolerance_ = declare_parameter<double>("goal_tolerance",  0.20);
        final_tolerance_= declare_parameter<double>("final_tolerance", 0.10);
        lookahead_      = declare_parameter<double>("lookahead",       0.6);
        slow_dist_      = declare_parameter<double>("slow_dist",       0.8);
        min_speed_      = declare_parameter<double>("min_speed",       0.04);
        map_frame_      = declare_parameter<std::string>("map_frame",  "map");
        base_frame_     = declare_parameter<std::string>("base_frame", "base_link");

        // ── MPC params ────────────────────────────────────────────────────
        N_   = declare_parameter<int>   ("mpc_horizon",      20);    // steps
        dt_  = declare_parameter<double>("mpc_dt",           0.10);  // s
        w_pos_   = declare_parameter<double>("mpc_w_pos",    5.0);   // path tracking
        w_theta_ = declare_parameter<double>("mpc_w_theta",  1.0);   // heading
        w_v_     = declare_parameter<double>("mpc_w_v",      0.1);   // speed penalty
        w_dv_    = declare_parameter<double>("mpc_w_dv",     0.5);   // accel smoothness
        w_dw_    = declare_parameter<double>("mpc_w_dw",     0.3);   // turn smoothness

        // ── CBF params (used both in MPC constraints and fallback) ────────
        cbf_alpha_          = declare_parameter<double>("cbf_alpha",          1.0);
        r_robot_            = declare_parameter<double>("r_robot",            0.25);
        cbf_influence_dist_ = declare_parameter<double>("cbf_influence_dist", 1.5);
        cbf_forward_cone_   = declare_parameter<double>("cbf_forward_cone",   0.2);
        cbf_path_half_width_= declare_parameter<double>("cbf_path_half_width",0.30);
        cbf_lookahead_t_    = declare_parameter<double>("cbf_lookahead_t",    0.5);

        // ── stuck detection ───────────────────────────────────────────────
        stuck_timeout_s_    = declare_parameter<double>("stuck_timeout_s",    3.0);
        max_replans_        = declare_parameter<int>   ("max_replans",        3);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── subscribers ───────────────────────────────────────────────────
        amcl_sub_ = create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", rclcpp::QoS(5).reliable(),
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                const auto &p = m->pose.pose;
                pose_x_   = p.position.x;
                pose_y_   = p.position.y;
                pose_yaw_ = quatToYaw(p.orientation.x, p.orientation.y,
                                      p.orientation.z, p.orientation.w);
                have_pose_ = true;
            });

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/astar/path", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::Path::SharedPtr m){ onPath(m); });

        obs_sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
            "/tracked_obstacles", rclcpp::QoS(10).best_effort(),
            [this](obstacle_detector::msg::Obstacles::SharedPtr m){
                obstacles_.clear();
                for (const auto &c : m->circles) {
                    double speed = std::hypot(c.velocity.x, c.velocity.y);
                    double d     = dist2d(c.center.x, c.center.y, pose_x_, pose_y_);
                    // filter: human size, moving or very close
                    if (c.true_radius < 0.08 || c.true_radius > 0.40) continue;
                    if (speed < 0.08 && d > 1.2) continue;
                    obstacles_.push_back(c);
                }
            });

        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/positioning/status", 10,
            [this](std_msgs::msg::String::SharedPtr m){
                docking_ = (m->data.rfind("DOCKING",0)==0 ||
                            m->data.rfind("DOCKED", 0)==0);
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr g){
                current_goal_          = *g;
                have_goal_             = true;
                last_dist_to_goal_     = 999.0;
                last_progress_time_    = now();
                replan_count_          = 0;
                waiting_for_clearance_ = false;
            });

        // ── publishers ────────────────────────────────────────────────────
        left_pub_   = create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel",  1);
        right_pub_  = create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/controller/target", 1);
        goal_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
        status_pub_ = create_publisher<std_msgs::msg::String>("/mpc/status", 1);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "MpcWaypointController ready | N=%d dt=%.2fs v_max=%.2f w_max=%.2f",
            N_, dt_, v_max_, w_max_);
    }

private:
    // ── params ───────────────────────────────────────────────────────────────
    double wheel_radius_, wheel_base_, max_wheel_rads_;
    double v_max_, w_max_;
    double goal_tolerance_, final_tolerance_;
    double lookahead_, slow_dist_, min_speed_;
    std::string map_frame_, base_frame_;

    int    N_;
    double dt_;
    double w_pos_, w_theta_, w_v_, w_dv_, w_dw_;

    double cbf_alpha_, r_robot_;
    double cbf_influence_dist_, cbf_forward_cone_;
    double cbf_path_half_width_, cbf_lookahead_t_;

    double stuck_timeout_s_;
    int    max_replans_;

    // ── state ────────────────────────────────────────────────────────────────
    double pose_x_{0}, pose_y_{0}, pose_yaw_{0};
    bool   have_pose_{false}, docking_{false};

    bool   have_goal_{false};
    double last_dist_to_goal_{999.0};
    rclcpp::Time last_progress_time_;
    int    replan_count_{0};
    bool   waiting_for_clearance_{false};
    rclcpp::Time wait_start_time_;
    geometry_msgs::msg::PoseStamped current_goal_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t wp_idx_{0};
    bool   active_{false};

    // last applied control (for smoothness penalties)
    double last_v_{0}, last_w_{0};

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            status_pub_;
    rclcpp::TimerBase::SharedPtr                                                   control_timer_;

    // ── diff drive IK ────────────────────────────────────────────────────────
    std::pair<double,double> diffDriveIK(double v, double omega) {
        const double a = wheel_base_ / 2.0;
        double omega_r = (v + omega * a) / wheel_radius_;
        double omega_l = (v - omega * a) / wheel_radius_;
        double mx = std::max(std::fabs(omega_r), std::fabs(omega_l));
        if (mx > max_wheel_rads_) { omega_r *= max_wheel_rads_/mx; omega_l *= max_wheel_rads_/mx; }
        return {omega_l, omega_r};
    }

    void publishWheelCmds(double v, double omega) {
        auto [ol, or_] = diffDriveIK(v, omega);
        std_msgs::msg::Float64 l, r;
        l.data = -ol; r.data = -or_;   // hardware: positive = backward
        left_pub_->publish(l); right_pub_->publish(r);
        last_v_ = v; last_w_ = omega;
    }

    void stopRobot() {
        std_msgs::msg::Float64 z; z.data = 0.0;
        left_pub_->publish(z); right_pub_->publish(z);
        last_v_ = last_w_ = 0.0;
    }

    bool tryTfPose() {
        try {
            auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_,
                tf2::TimePointZero, std::chrono::milliseconds(20));
            pose_x_   = tf.transform.translation.x;
            pose_y_   = tf.transform.translation.y;
            const auto &q = tf.transform.rotation;
            pose_yaw_ = quatToYaw(q.x, q.y, q.z, q.w);
            have_pose_ = true;
            return true;
        } catch (...) { return false; }
    }

    // ── path callbacks ────────────────────────────────────────────────────────
    void onPath(nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) { stopRobot(); active_ = false; return; }
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

    // ── predict robot state N steps forward ──────────────────────────────────
    // Returns predicted [x, y, theta] at each step given control sequence u
    // Uses linearised Euler integration of diff drive kinematics
    std::vector<std::array<double,3>> predictStates(
        double x0, double y0, double th0,
        const std::vector<double>& v_seq,
        const std::vector<double>& w_seq) const
    {
        std::vector<std::array<double,3>> states(N_+1);
        states[0] = {x0, y0, th0};
        for (int t = 0; t < N_; ++t) {
            double x  = states[t][0];
            double y  = states[t][1];
            double th = states[t][2];
            double v  = v_seq[t];
            double w  = w_seq[t];
            states[t+1][0] = x  + v * std::cos(th) * dt_;
            states[t+1][1] = y  + v * std::sin(th) * dt_;
            states[t+1][2] = th + w * dt_;
        }
        return states;
    }

    // ── get reference pose at horizon step t ─────────────────────────────────
    std::pair<double,double> refPose(int t) {
        // find waypoint at distance t*dt*v_max ahead along path
        double dist_ahead = t * dt_ * v_max_;
        double acc = 0.0;
        for (size_t i = wp_idx_; i < waypoints_.size()-1; ++i) {
            double seg = dist2d(waypoints_[i].pose.position.x,
                                waypoints_[i].pose.position.y,
                                waypoints_[i+1].pose.position.x,
                                waypoints_[i+1].pose.position.y);
            if (acc + seg >= dist_ahead) {
                double frac = (dist_ahead - acc) / std::max(seg, 1e-6);
                double rx = waypoints_[i].pose.position.x +
                            frac*(waypoints_[i+1].pose.position.x - waypoints_[i].pose.position.x);
                double ry = waypoints_[i].pose.position.y +
                            frac*(waypoints_[i+1].pose.position.y - waypoints_[i].pose.position.y);
                return {rx, ry};
            }
            acc += seg;
        }
        // past end of path → return final waypoint
        return {waypoints_.back().pose.position.x,
                waypoints_.back().pose.position.y};
    }

    // ── CBF constraint coefficients for one obstacle at one timestep ──────────
    // Returns false if obstacle is outside gates (constraint not needed)
    struct CbfRow {
        double coeff_v;    // coefficient of v in constraint
        double coeff_w;    // coefficient of omega in constraint
        double rhs;        // right-hand side
    };

    bool buildCbfRow(double px, double py, double pth,
                     double v_lin,
                     const obstacle_detector::msg::CircleObstacle& obs,
                     CbfRow& row) const
    {
        const double ox  = obs.center.x;
        const double oy  = obs.center.y;
        const double ovx = obs.velocity.x;
        const double ovy = obs.velocity.y;

        const double fx = std::cos(pth);
        const double fy = std::sin(pth);
        const double lx = -std::sin(pth);
        const double ly =  std::cos(pth);

        const double dx   = ox - px;
        const double dy   = oy - py;
        const double dist = std::hypot(dx, dy) + 1e-6;

        // Gate 1: influence distance
        if (dist > cbf_influence_dist_) return false;

        // Gate 2: forward cone
        const double fwd_proj = (dx*fx + dy*fy) / dist;
        if (fwd_proj < cbf_forward_cone_) return false;

        // Gate 3: lateral gate
        const double lat_dist = std::fabs(dx*lx + dy*ly);
        const double r_safe   = obs.radius + r_robot_;
        if (lat_dist > r_safe + cbf_path_half_width_) return false;

        // CBF value h = dist^2 - r_safe^2
        const double rdx     = px - ox;   // robot minus obstacle
        const double rdy     = py - oy;
        const double dist_sq = rdx*rdx + rdy*rdy;
        const double h_val   = dist_sq - r_safe*r_safe;

        // Lie derivatives
        // Lgh_v:  how forward speed v changes h
        const double Lgh_v = 2.0*(rdx*fx + rdy*fy);

        // Lgh_w:  lookahead correction so omega steers around obstacle
        const double Lgh_w = 2.0*(rdx*(-v_lin*cbf_lookahead_t_*std::sin(pth)) +
                                  rdy*( v_lin*cbf_lookahead_t_*std::cos(pth)));

        // Lfh: obstacle own velocity contribution
        const double Lfh = 2.0*(rdx*(-ovx) + rdy*(-ovy));

        // Scale alpha by directionality
        const double alpha_s = cbf_alpha_ * std::max(0.3, fwd_proj);

        // CBF constraint: Lgh_v*v + Lgh_w*w >= -alpha*h - Lfh
        // Rewritten:     -Lgh_v*v - Lgh_w*w <= Lfh + alpha*h
        row.coeff_v = -Lgh_v;
        row.coeff_w = -Lgh_w;
        row.rhs     = Lfh + alpha_s * h_val;
        return true;
    }

    // ── MPC solve via OSQP ────────────────────────────────────────────────────
    //
    // Decision variables:  u = [v_0, w_0, v_1, w_1, ..., v_{N-1}, w_{N-1}]
    // Size: 2*N
    //
    // Cost:  0.5 * u^T P u + q^T u
    //
    // Constraints:
    //   l <= A*u <= u_bound
    //   Rows: [box constraints] + [CBF constraints per step per obstacle]
    //
    // Returns {v0, w0} — first control action.
    // Sets infeasible=true if no solution found.
    //
    std::pair<double,double> solveMPC(bool& infeasible) {
        const int n_vars = 2 * N_;   // [v0,w0, v1,w1, ...]

        // ── build cost matrix P (quadratic) and q (linear) ────────────────
        // P is block diagonal with contributions from:
        //   w_pos * (dp/du)^T (dp/du) for each step  [position tracking]
        //   w_theta * (dth/du)^T (dth/du)            [heading tracking]
        //   w_v * I_v                                 [speed penalty]
        //   w_dv * delta_v penalty                    [smoothness]
        //   w_dw * delta_w penalty                    [smoothness]

        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_vars, n_vars);
        Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

        // Propagate linearised Jacobians dp/dv_t and dp/dw_t
        // x(t) = x0 + sum_{k=0}^{t-1} v_k * cos(th_k) * dt
        // For linearisation we freeze theta at current heading
        // (good approximation for short horizons at low speed)
        const double cth = std::cos(pose_yaw_);
        const double sth = std::sin(pose_yaw_);

        for (int t = 1; t <= N_; ++t) {
            auto [rx, ry] = refPose(t);

            // Predicted position (with zero control as linearisation point)
            double px_pred = pose_x_ + t * 0.0 * cth * dt_;  // linearised about v=0
            double py_pred = pose_y_;

            // Jacobian: d(x_t)/d(v_k) = cos(th)*dt for k < t, 0 otherwise
            //           d(x_t)/d(w_k) ≈ 0 (theta coupling, ignored in linear approx)
            // Full nonlinear would need successive linearisation — this is
            // sufficient for the short horizon and slow speed of this robot.
            for (int k = 0; k < t && k < N_; ++k) {
                int iv = 2*k;   // index of v_k in u

                // position cost: w_pos * ||p_pred - p_ref||^2
                // gradient contribution to P and q:
                double ex = px_pred - rx;
                double ey = py_pred - ry;

                // d(x_t)/d(v_k) = cos(th)*dt,  d(y_t)/d(v_k) = sin(th)*dt
                double dxdv = cth * dt_;
                double dydv = sth * dt_;

                P(iv, iv) += 2.0 * w_pos_ * (dxdv*dxdv + dydv*dydv);
                q(iv)     += 2.0 * w_pos_ * (ex*dxdv + ey*dydv);
            }

            // Speed penalty: w_v * v_t^2
            if (t-1 < N_) {
                int iv = 2*(t-1);
                P(iv, iv) += 2.0 * w_v_;
            }
        }

        // Smoothness penalties: w_dv*(v_t - v_{t-1})^2, w_dw*(w_t - w_{t-1})^2
        for (int t = 0; t < N_; ++t) {
            int iv = 2*t, iw = 2*t+1;
            P(iv, iv) += 2.0 * w_dv_;
            P(iw, iw) += 2.0 * w_dw_;
            if (t > 0) {
                P(iv,   iv-2) -= 2.0 * w_dv_;
                P(iv-2, iv)   -= 2.0 * w_dv_;
                P(iw,   iw-2) -= 2.0 * w_dw_;
                P(iw-2, iw)   -= 2.0 * w_dw_;
            } else {
                // first step: penalise change from last applied control
                q(iv) -= 2.0 * w_dv_ * last_v_;
                q(iw) -= 2.0 * w_dw_ * last_w_;
            }
        }

        // ── build constraint matrix A, lower l, upper u ───────────────────
        // Row structure:
        //   [0 .. 2N-1]          box constraints: v_t in [0, v_max], w_t in [-w_max, w_max]
        //   [2N .. 2N+n_cbf-1]   CBF constraints (variable count)

        // First pass: collect CBF rows
        struct CbfEntry { int t; double coeff_v, coeff_w, rhs; };
        std::vector<CbfEntry> cbf_entries;

        // We need predicted states to compute CBF constraints at each step.
        // Use a nominal rollout with v=v_max/2, w=0 as linearisation point.
        {
            double px = pose_x_, py = pose_y_, pth = pose_yaw_;
            for (int t = 0; t < N_; ++t) {
                double v_lin = v_max_ * 0.5;
                for (const auto& obs : obstacles_) {
                    CbfRow row;
                    if (buildCbfRow(px, py, pth, v_lin, obs, row)) {
                        cbf_entries.push_back({t, row.coeff_v, row.coeff_w, row.rhs});
                    }
                }
                // propagate nominal state
                px  += v_lin * std::cos(pth) * dt_;
                py  += v_lin * std::sin(pth) * dt_;
                // theta unchanged for nominal
            }
        }

        const int n_box = 2 * N_;
        const int n_cbf = static_cast<int>(cbf_entries.size());
        const int n_con = n_box + n_cbf;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_con, n_vars);
        Eigen::VectorXd l = Eigen::VectorXd::Constant(n_con, -1e10);
        Eigen::VectorXd u = Eigen::VectorXd::Constant(n_con,  1e10);

        // Box constraints: one row per control variable
        for (int t = 0; t < N_; ++t) {
            int iv = 2*t, iw = 2*t+1;
            // v_t in [min_speed_, v_max_]
            A(iv, iv) = 1.0;  l(iv) = min_speed_;  u(iv) = v_max_;
            // w_t in [-w_max_, w_max_]
            A(iw, iw) = 1.0;  l(iw) = -w_max_;     u(iw) = w_max_;
        }

        // CBF constraints: -Lgh_v*v_t - Lgh_w*w_t <= rhs
        // → A_cbf * u <= rhs
        // → l=-inf, u=rhs  (upper bound)
        for (int ci = 0; ci < n_cbf; ++ci) {
            int row = n_box + ci;
            int t   = cbf_entries[ci].t;
            int iv  = 2*t, iw = 2*t+1;
            A(row, iv) = cbf_entries[ci].coeff_v;
            A(row, iw) = cbf_entries[ci].coeff_w;
            l(row) = -1e10;
            u(row) = cbf_entries[ci].rhs;
        }

        // ── convert to CSC and call OSQP ─────────────────────────────────
        auto P_csc = toCscUpper(P);
        auto A_csc = toCscFull(A);

        // Convert to OSQPFloat vectors
        std::vector<OSQPFloat> q_osqp(n_vars), l_osqp(n_con), u_osqp(n_con);
        for (int i=0; i<n_vars; ++i) q_osqp[i] = static_cast<OSQPFloat>(q(i));
        for (int i=0; i<n_con;  ++i) { l_osqp[i] = static_cast<OSQPFloat>(l(i));
                                        u_osqp[i] = static_cast<OSQPFloat>(u(i)); }

        OSQPSolver*  solver  = nullptr;
        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose        = 0;
        settings.warm_starting  = 1;
        settings.max_iter       = 1000;
        settings.eps_abs        = 1e-4;
        settings.eps_rel        = 1e-4;
        settings.time_limit     = 0.04;   // 40ms — well within 50ms loop

        OSQPCscMatrix P_mat, A_mat;

        P_mat.m   = P_csc.m;   P_mat.n  = P_csc.n;  P_mat.nzmax = P_csc.nnz;
        P_mat.x   = P_csc.data.data();
        P_mat.i   = P_csc.row_ind.data();
        P_mat.p   = P_csc.col_ptr.data();
        P_mat.nz  = -1;  // CSC format

        A_mat.m   = A_csc.m;   A_mat.n  = A_csc.n;  A_mat.nzmax = A_csc.nnz;
        A_mat.x   = A_csc.data.data();
        A_mat.i   = A_csc.row_ind.data();
        A_mat.p   = A_csc.col_ptr.data();
        A_mat.nz  = -1;

        OSQPInt exit_code = osqp_setup(&solver, &P_mat, q_osqp.data(),
                                       &A_mat, l_osqp.data(), u_osqp.data(),
                                       n_con, n_vars, &settings);

        if (exit_code != 0 || solver == nullptr) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "OSQP setup failed (%d) — using CBF fallback", (int)exit_code);
            infeasible = true;
            return {0.0, 0.0};
        }

        osqp_solve(solver);
        const OSQPInfo* info = solver->info;

        double v0 = 0.0, w0 = 0.0;
        if (info->status_val == OSQP_SOLVED ||
            info->status_val == OSQP_SOLVED_INACCURATE) {
            const OSQPFloat* sol = solver->solution->x;
            v0 = clamp(static_cast<double>(sol[0]), 0.0,    v_max_);
            w0 = clamp(static_cast<double>(sol[1]), -w_max_, w_max_);
            infeasible = false;
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "OSQP status: %s — CBF fallback", info->status);
            infeasible = true;
        }

        osqp_cleanup(solver);
        return {v0, w0};
    }

    // ── CBF fallback (same logic as waypoint_controller) ─────────────────────
    // Used when MPC QP is infeasible (obstacle fully blocking all options)
    std::pair<double,double> cbfFallback(double v_nom, double w_nom) {
        if (obstacles_.empty()) return {v_nom, w_nom};

        const double fx = std::cos(pose_yaw_);
        const double fy = std::sin(pose_yaw_);
        const double lx = -std::sin(pose_yaw_);
        const double ly =  std::cos(pose_yaw_);

        struct AO { double ox,oy,ovx,ovy,r_safe,fwd; };
        std::vector<AO> active;

        for (const auto& obs : obstacles_) {
            const double dx = obs.center.x - pose_x_;
            const double dy = obs.center.y - pose_y_;
            const double d  = std::hypot(dx,dy)+1e-6;
            if (d > cbf_influence_dist_) continue;
            const double fwd = (dx*fx+dy*fy)/d;
            if (fwd < cbf_forward_cone_) continue;
            const double lat = std::fabs(dx*lx+dy*ly);
            const double rs  = obs.radius + r_robot_;
            if (lat > rs + cbf_path_half_width_) continue;
            active.push_back({obs.center.x, obs.center.y,
                              obs.velocity.x, obs.velocity.y, rs, fwd});
        }
        if (active.empty()) return {v_nom, w_nom};

        int n_obs = active.size();
        int n_con = n_obs + 4;
        Eigen::MatrixXd G(n_con, 2);
        Eigen::VectorXd h(n_con);

        for (int i=0; i<n_obs; ++i) {
            const auto& a = active[i];
            double dx = pose_x_-a.ox, dy = pose_y_-a.oy;
            double ds = dx*dx+dy*dy;
            double hv = ds - a.r_safe*a.r_safe;
            double Lgv = 2.0*(dx*fx+dy*fy);
            double Lgw = 2.0*(dx*(-v_nom*cbf_lookahead_t_*std::sin(pose_yaw_))+
                              dy*( v_nom*cbf_lookahead_t_*std::cos(pose_yaw_)));
            double Lfh = 2.0*(dx*(-a.ovx)+dy*(-a.ovy));
            double alpha_s = cbf_alpha_*std::max(0.3, a.fwd);
            G(i,0)=-Lgv; G(i,1)=-Lgw; h(i)=Lfh+alpha_s*hv;
        }
        G(n_obs,0)=-1; G(n_obs,1)=0;   h(n_obs)=0.0;
        G(n_obs+1,0)=1; G(n_obs+1,1)=0; h(n_obs+1)=v_max_;
        G(n_obs+2,0)=0; G(n_obs+2,1)=-1;h(n_obs+2)=w_max_;
        G(n_obs+3,0)=0; G(n_obs+3,1)=1; h(n_obs+3)=w_max_;

        Eigen::Vector2d u(v_nom, w_nom);
        for (int iter=0; iter<20; ++iter) {
            bool viol=false;
            for (int i=0; i<n_con; ++i) {
                Eigen::Vector2d g=G.row(i).transpose();
                double slack=g.dot(u)-h(i);
                if (slack>1e-6) {
                    viol=true;
                    double gn=g.squaredNorm();
                    if (gn<1e-12) continue;
                    u -= (slack/gn)*g;
                }
            }
            if (!viol) break;
        }
        if (u(0)<0) u(0)=0;
        return {u(0), u(1)};
    }

    // ── publish status ────────────────────────────────────────────────────────
    void pubStatus(const std::string& s) {
        std_msgs::msg::String msg; msg.data = s;
        status_pub_->publish(msg);
    }

    // ── main control loop (20 Hz) ─────────────────────────────────────────────
    void controlLoop() {
        if (docking_) return;
        if (!active_ || waypoints_.empty()) return;

        if (!have_pose_ && !tryTfPose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for pose");
            return;
        }

        // advance past reached waypoints
        while (wp_idx_ < waypoints_.size()-1) {
            double d = dist2d(pose_x_, pose_y_,
                              waypoints_[wp_idx_].pose.position.x,
                              waypoints_[wp_idx_].pose.position.y);
            if (d < goal_tolerance_) ++wp_idx_;
            else break;
        }

        double dist_final = dist2d(pose_x_, pose_y_,
                                   waypoints_.back().pose.position.x,
                                   waypoints_.back().pose.position.y);

        // ── stuck detection ──────────────────────────────────────────────
        if (have_goal_ && active_ && !obstacles_.empty()) {
            if (waiting_for_clearance_) {
                stopRobot();
                // check if path clear
                bool clear = true;
                for (const auto& obs : obstacles_) {
                    double sp = std::hypot(obs.velocity.x, obs.velocity.y);
                    if (sp < 0.08) continue;
                    for (size_t i = wp_idx_; i < std::min(wp_idx_+20, waypoints_.size()); ++i) {
                        double d = dist2d(waypoints_[i].pose.position.x,
                                          waypoints_[i].pose.position.y,
                                          obs.center.x, obs.center.y);
                        if (d < obs.radius + r_robot_ + 0.3) { clear=false; break; }
                    }
                    if (!clear) break;
                }
                double waited = (now()-wait_start_time_).seconds();
                if (clear || waited > 10.0) {
                    goal_pub_->publish(current_goal_);
                    waiting_for_clearance_ = false;
                    replan_count_ = 0;
                    last_progress_time_ = now();
                }
                return;
            }

            if (dist_final < last_dist_to_goal_ - 0.05) {
                last_dist_to_goal_  = dist_final;
                last_progress_time_ = now();
                replan_count_       = 0;
            } else {
                double stuck = (now()-last_progress_time_).seconds();
                if (stuck > stuck_timeout_s_) {
                    if (replan_count_ < max_replans_) {
                        ++replan_count_;
                        RCLCPP_WARN(get_logger(), "Stuck %.1fs — replan %d/%d",
                            stuck, replan_count_, max_replans_);
                        goal_pub_->publish(current_goal_);
                        last_progress_time_ = now();
                    } else {
                        stopRobot();
                        waiting_for_clearance_ = true;
                        wait_start_time_       = now();
                        replan_count_          = 0;
                    }
                }
            }
        }

        if (dist_final < final_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            stopRobot(); active_ = false; return;
        }

        // ── publish lookahead marker ─────────────────────────────────────
        size_t tidx = lookaheadIndex();
        double tx = waypoints_[tidx].pose.position.x;
        double ty = waypoints_[tidx].pose.position.y;
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_; m.header.stamp = now();
            m.ns="mpc_target"; m.id=0;
            m.type=visualization_msgs::msg::Marker::SPHERE;
            m.action=visualization_msgs::msg::Marker::ADD;
            m.pose.position.x=tx; m.pose.position.y=ty; m.pose.position.z=0.25;
            m.pose.orientation.w=1.0;
            m.scale.x=m.scale.y=m.scale.z=0.3;
            m.color.r=0.0f; m.color.g=0.6f; m.color.b=1.0f; m.color.a=1.0f;
            m.lifetime=rclcpp::Duration::from_seconds(0.15);
            marker_pub_->publish(m);
        }

        // ── solve MPC ────────────────────────────────────────────────────
        bool infeasible = false;
        auto [v_mpc, w_mpc] = solveMPC(infeasible);

        if (!infeasible) {
            // MPC solution found — apply first control action
            pubStatus("MPC");
            RCLCPP_DEBUG(get_logger(), "MPC: v=%.3f w=%.3f", v_mpc, w_mpc);
            publishWheelCmds(v_mpc, w_mpc);
        } else {
            // MPC infeasible — fall back to pure CBF
            pubStatus("CBF_FALLBACK");

            // compute nominal from pure pursuit
            double desired_yaw = std::atan2(ty-pose_y_, tx-pose_x_);
            double herr = wrapAngle(desired_yaw - pose_yaw_);
            double v_nom = linear_speed_nom();
            if (dist_final < slow_dist_) v_nom *= dist_final / slow_dist_;
            v_nom *= std::max(0.0, std::cos(herr));
            v_nom  = std::max(min_speed_, v_nom);
            if (std::fabs(herr) > M_PI/2.0) v_nom = 0.0;
            double w_nom = clamp(2.5 * herr, -w_max_, w_max_);

            auto [v_safe, w_safe] = cbfFallback(v_nom, w_nom);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "CBF fallback: v=%.3f w=%.3f", v_safe, w_safe);
            publishWheelCmds(v_safe, w_safe);
        }
    }

    double linear_speed_nom() const { return v_max_; }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcWaypointController>());
    rclcpp::shutdown();
    return 0;
}