#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <obstacle_detector/msg/obstacles.hpp>
#include <obstacle_detector/msg/circle_obstacle.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <limits>

// ── quat helpers ─────────────────────────────────────────────────────────────
double quat_to_yaw(const geometry_msgs::msg::Quaternion& q_in) {
    const double n = std::sqrt(q_in.x*q_in.x + q_in.y*q_in.y +
                               q_in.z*q_in.z + q_in.w*q_in.w);
    if (n == 0.0) return 0.0;
    const double x = q_in.x/n, y = q_in.y/n, z = q_in.z/n, w = q_in.w/n;
    return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0; q.y = 0.0;
    q.z = std::sin(0.5*yaw);
    q.w = std::cos(0.5*yaw);
    return q;
}

// ── main node ─────────────────────────────────────────────────────────────────
class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : rclcpp::Node("astar_planner") {
        RCLCPP_INFO(get_logger(), "A* Planner node initialized");

        use_prm_             = declare_parameter<bool>  ("use_prm",              true);
        heuristic_weight_    = declare_parameter<double>("heuristic_weight",      1.0);
        inflation_radius_m_  = declare_parameter<double>("inflation_radius_m",    0.2);
        allow_diagonal_      = declare_parameter<bool>  ("allow_diagonal",        true);
        use_tf_start_        = declare_parameter<bool>  ("use_tf_start",          true);
        map_frame_           = declare_parameter<std::string>("map_frame",        "map");
        base_frame_          = declare_parameter<std::string>("base_frame",       "base_link");

        // ── dynamic obstacle clearance ────────────────────────────────────
        // Must satisfy: dynamic_obs_clearance > cbf_influence_dist + cbf_path_half_width
        // Default: 1.9 > 1.5 + 0.30 = 1.80 ✓
        // This ensures A* routes the robot completely outside the CBF influence
        // zone so the robot follows the replanned path at full speed.
        dynamic_obs_clearance_ = declare_parameter<double>("dynamic_obs_clearance", 1.9);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this](){
            if (!use_tf_start_) return;
            try {
                auto tf = tf_buffer_->lookupTransform(
                    map_frame_, base_frame_, tf2::TimePointZero);
                start_pose_tf_.header = tf.header;
                start_pose_tf_.pose.position.x  = tf.transform.translation.x;
                start_pose_tf_.pose.position.y  = tf.transform.translation.y;
                start_pose_tf_.pose.position.z  = 0.0;
                start_pose_tf_.pose.orientation = tf.transform.rotation;
                have_start_tf_ = true;
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Could not get transform: %s", ex.what());
                have_start_tf_ = false;
            }
        });

        rclcpp::QoS latched(1);
        latched.reliable().transient_local();

        path_pub_     = create_publisher<nav_msgs::msg::Path>("/astar/path", latched);
        explored_pub_ = create_publisher<visualization_msgs::msg::Marker>("/astar/explored", 1);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){ onMap(m); });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr g){ onGoal(g); });

        start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr s){ onStart(s); });

        // ── dynamic obstacles (moving people) ────────────────────────────
        // Only accept circles that are actively moving — stationary detections
        // are likely wall fragments and are already handled by the static map.
        obs_sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
            "/tracked_obstacles", rclcpp::QoS(10).best_effort(),
            [this](obstacle_detector::msg::Obstacles::SharedPtr msg){
                dynamic_obstacles_.clear();
                for (const auto& c : msg->circles) {
                    // double speed = std::hypot(c.velocity.x, c.velocity.y);
                    // if (speed < 0.08) continue;   // stationary → skip
                    if (c.true_radius < 0.08 || c.true_radius > 0.40) continue; // not human
                    dynamic_obstacles_.push_back(c);
                }
            });

        if (use_prm_) {
            prm_nodes_sub_ = create_subscription<visualization_msgs::msg::Marker>(
                "/prm/nodes", rclcpp::QoS(1).transient_local().reliable(),
                [this](visualization_msgs::msg::Marker::SharedPtr m){ onPrmNodes(m); });

            prm_adj_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
                "/prm/adjacency", rclcpp::QoS(1).transient_local().reliable(),
                [this](std_msgs::msg::Int32MultiArray::SharedPtr a){ onPrmAdjacency(a); });
        }

        RCLCPP_INFO(get_logger(),
            "A* Planner configured: use_prm=%s heuristic=%.2f dyn_clearance=%.2fm",
            use_prm_ ? "true" : "false", heuristic_weight_, dynamic_obs_clearance_);
    }

private:
    // ── params ───────────────────────────────────────────────────────────────
    bool        use_prm_;
    double      heuristic_weight_;
    double      inflation_radius_m_;
    bool        allow_diagonal_;
    bool        use_tf_start_;
    std::string map_frame_, base_frame_;

    // dynamic obstacle clearance — must be > cbf_influence_dist + cbf_path_half_width
    double dynamic_obs_clearance_{1.9};

    // ── state ────────────────────────────────────────────────────────────────
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<uint8_t>         inflated_;
    bool have_map_ = false;

    geometry_msgs::msg::PoseStamped start_, goal_;
    bool have_start_ = false, have_goal_ = false;

    struct PrmNode { double x, y; };
    std::vector<PrmNode>          prm_nodes_;
    std::vector<std::vector<int>> prm_adj_;
    bool have_prm_ = false;

    std::vector<obstacle_detector::msg::CircleObstacle> dynamic_obstacles_;

    // ── ROS handles ──────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool have_start_tf_{false};
    geometry_msgs::msg::PoseStamped start_pose_tf_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr              map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr           goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr           prm_nodes_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr            prm_adj_sub_;
    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr         obs_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  explored_pub_;

    // ── pose source ───────────────────────────────────────────────────────────
    bool getStartPose(geometry_msgs::msg::PoseStamped& out) {
        if (use_tf_start_) {
            if (!have_start_tf_) return false;
            out = start_pose_tf_;
            return true;
        }
        if (!have_start_) return false;
        out = start_;
        return true;
    }

    // ── dynamic obstacle gate ─────────────────────────────────────────────────
    // Returns true if PRM node (nx, ny) is inside the clearance zone of any
    // moving obstacle.
    //
    // clearance = obs.radius + dynamic_obs_clearance_
    //
    // dynamic_obs_clearance_ is set larger than cbf_influence_dist so that
    // any path A* finds is completely outside the CBF influence zone.
    // This ensures the robot follows the replanned path at full speed
    // without CBF activating.
    //
    // Rule to maintain in yaml:
    //   dynamic_obs_clearance > cbf_influence_dist + cbf_path_half_width
    //   e.g.  1.9             >      1.5           +      0.30          ✓
    bool nodeBlockedByDynamicObs(double nx, double ny) const {
        for (const auto& obs : dynamic_obstacles_) {
            double dx   = nx - obs.center.x;
            double dy   = ny - obs.center.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < obs.radius + dynamic_obs_clearance_) return true;
        }
        return false;
    }

    // ── map utilities ─────────────────────────────────────────────────────────
    inline bool worldToMap(double wx, double wy, int& mx, int& my) const {
        double ox = map_.info.origin.position.x;
        double oy = map_.info.origin.position.y;
        double r  = map_.info.resolution;
        mx = (int)std::floor((wx - ox) / r);
        my = (int)std::floor((wy - oy) / r);
        return mx >= 0 && my >= 0 &&
               (unsigned)mx < map_.info.width &&
               (unsigned)my < map_.info.height;
    }

    inline void mapToWorld(int mx, int my, double& wx, double& wy) const {
        wx = map_.info.origin.position.x + (mx + 0.5) * map_.info.resolution;
        wy = map_.info.origin.position.y + (my + 0.5) * map_.info.resolution;
    }

    inline int coordToIndex(int x, int y) const {
        return y * (int)map_.info.width + x;
    }

    inline void indexToCoord(int idx, int& x, int& y) const {
        x = idx % (int)map_.info.width;
        y = idx / (int)map_.info.width;
    }

    inline bool isOccupied(int mx, int my) const {
        if (mx < 0 || my < 0 ||
            (unsigned)mx >= map_.info.width ||
            (unsigned)my >= map_.info.height) return true;
        return inflated_[my * map_.info.width + mx] > 0;
    }

    void inflateMap() {
        inflated_.assign(map_.info.width * map_.info.height, 0);
        int R = (int)std::ceil(inflation_radius_m_ / map_.info.resolution);
        int W = (int)map_.info.width, H = (int)map_.info.height;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int8_t v = map_.data[y * W + x];
                if (v < 0 || v >= 65) {
                    for (int dy = -R; dy <= R; ++dy) {
                        int yy = y + dy;
                        if (yy < 0 || yy >= H) continue;
                        int dxmax = (int)std::floor(
                            std::sqrt((double)R*R - dy*dy));
                        int x0 = std::max(0, x - dxmax);
                        int x1 = std::min(W-1, x + dxmax);
                        std::fill(inflated_.begin() + yy*W + x0,
                                  inflated_.begin() + yy*W + x1 + 1, 1);
                    }
                }
            }
        }
    }

    // ── grid A* ───────────────────────────────────────────────────────────────
    struct GridNode {
        int x, y;
        double yaw, g_cost, h_cost;
        int parent_idx;
        double f_cost() const { return g_cost + h_cost; }
        bool operator>(const GridNode& o) const { return f_cost() > o.f_cost(); }
    };

    double heuristic(int x1, int y1, int x2, int y2) const {
        double dx = (x2-x1) * map_.info.resolution;
        double dy = (y2-y1) * map_.info.resolution;
        return std::sqrt(dx*dx + dy*dy) * heuristic_weight_;
    }

    std::vector<std::pair<int,int>> getNeighbors(int x, int y) const {
        static const std::vector<std::pair<int,int>> moves = {
            {-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
        std::vector<std::pair<int,int>> nbrs;
        int max_n = allow_diagonal_ ? 8 : 4;
        for (int i = 0; i < max_n; ++i) {
            int nx = x + moves[i].first;
            int ny = y + moves[i].second;
            if (nx >= 0 && ny >= 0 &&
                (unsigned)nx < map_.info.width &&
                (unsigned)ny < map_.info.height &&
                !isOccupied(nx, ny))
                nbrs.emplace_back(nx, ny);
        }
        return nbrs;
    }

    nav_msgs::msg::Path gridAStar(
        int sx, int sy, double syaw,
        int gx, int gy, double gyaw)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = map_.header.frame_id;
        path.header.stamp    = now();
        (void)gyaw;

        std::priority_queue<GridNode, std::vector<GridNode>,
                            std::greater<GridNode>> open;
        std::unordered_map<int, GridNode> all;
        std::unordered_set<int>           closed;
        std::vector<int>                  explored;

        int si = coordToIndex(sx, sy);
        GridNode sn; sn.x=sx; sn.y=sy; sn.yaw=syaw;
        sn.g_cost=0; sn.h_cost=heuristic(sx,sy,gx,gy); sn.parent_idx=-1;
        open.push(sn); all[si]=sn;

        while (!open.empty()) {
            auto cur = open.top(); open.pop();
            int ci = coordToIndex(cur.x, cur.y);
            if (closed.count(ci)) continue;
            closed.insert(ci); explored.push_back(ci);

            if (cur.x == gx && cur.y == gy) {
                std::vector<std::pair<int,int>> coords;
                int idx = ci;
                while (idx != -1) {
                    int x, y; indexToCoord(idx, x, y);
                    coords.emplace_back(x, y);
                    idx = all[idx].parent_idx;
                }
                std::reverse(coords.begin(), coords.end());
                for (auto& c : coords) {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = path.header;
                    mapToWorld(c.first, c.second,
                               ps.pose.position.x, ps.pose.position.y);
                    ps.pose.orientation = yaw_to_quat(cur.yaw);
                    path.poses.push_back(ps);
                }
                publishExplored(explored);
                RCLCPP_INFO(get_logger(), "Grid A*: %zu waypoints, %zu cells",
                    path.poses.size(), explored.size());
                return path;
            }

            for (auto& nb : getNeighbors(cur.x, cur.y)) {
                int nx=nb.first, ny=nb.second;
                int ni = coordToIndex(nx, ny);
                if (closed.count(ni)) continue;
                double step = (nx!=cur.x && ny!=cur.y)
                              ? map_.info.resolution*1.414
                              : map_.info.resolution;
                double tg = cur.g_cost + step;
                if (!all.count(ni) || tg < all[ni].g_cost) {
                    GridNode nn; nn.x=nx; nn.y=ny;
                    nn.g_cost=tg; nn.h_cost=heuristic(nx,ny,gx,gy);
                    nn.parent_idx=ci;
                    all[ni]=nn; open.push(nn);
                }
            }
        }
        publishExplored(explored);
        RCLCPP_WARN(get_logger(), "Grid A* failed, explored %zu", explored.size());
        return path;
    }

    // ── PRM A* ────────────────────────────────────────────────────────────────
    double euclideanDistance(double x1, double y1, double x2, double y2) const {
        return std::hypot(x2-x1, y2-y1);
    }

    int findNearestNode(double x, double y) const {
        int    best = -1;
        double bd   = std::numeric_limits<double>::max();
        for (size_t i = 0; i < prm_nodes_.size(); ++i) {
            double d = euclideanDistance(x, y, prm_nodes_[i].x, prm_nodes_[i].y);
            if (d < bd) { bd = d; best = (int)i; }
        }
        return best;
    }

    nav_msgs::msg::Path prmAStar() {
        nav_msgs::msg::Path path;
        path.header.frame_id = map_.header.frame_id;
        path.header.stamp    = now();

        if (!have_prm_) {
            RCLCPP_WARN(get_logger(), "PRM not available");
            return path;
        }

        geometry_msgs::msg::PoseStamped sp;
        if (!getStartPose(sp)) return path;

        double sx  = sp.pose.position.x,  sy  = sp.pose.position.y;
        double syaw= quat_to_yaw(sp.pose.orientation);
        double gx  = goal_.pose.position.x, gy = goal_.pose.position.y;
        double gyaw= quat_to_yaw(goal_.pose.orientation);
        (void)syaw; (void)gyaw;

        int sn = findNearestNode(sx, sy);
        int gn = findNearestNode(gx, gy);
        if (sn < 0 || gn < 0) {
            RCLCPP_WARN(get_logger(), "No valid PRM start/goal nodes");
            return path;
        }

        struct PNode {
            int idx, parent;
            double g_cost, h_cost;
            double f_cost() const { return g_cost + h_cost; }
            bool operator>(const PNode& o) const { return f_cost() > o.f_cost(); }
        };

        std::priority_queue<PNode, std::vector<PNode>, std::greater<PNode>> open;
        std::unordered_map<int, PNode> all;
        std::unordered_set<int>        closed;

        PNode start_pn;
        start_pn.idx    = sn;
        start_pn.parent = -1;
        start_pn.g_cost = 0;
        start_pn.h_cost = euclideanDistance(
            prm_nodes_[sn].x, prm_nodes_[sn].y,
            prm_nodes_[gn].x, prm_nodes_[gn].y) * heuristic_weight_;
        open.push(start_pn); all[sn] = start_pn;

        while (!open.empty()) {
            auto cur = open.top(); open.pop();
            if (closed.count(cur.idx)) continue;
            closed.insert(cur.idx);

            if (cur.idx == gn) {
                // reconstruct path
                std::vector<int> indices;
                int idx = cur.idx;
                while (idx != -1) {
                    indices.push_back(idx);
                    idx = all[idx].parent;
                }
                std::reverse(indices.begin(), indices.end());

                // start pose
                geometry_msgs::msg::PoseStamped ps0;
                ps0.header = path.header;
                ps0.pose.position.x  = sx; ps0.pose.position.y = sy;
                ps0.pose.orientation = goal_.pose.orientation;
                path.poses.push_back(ps0);

                // PRM node poses
                for (int i : indices) {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = path.header;
                    ps.pose.position.x  = prm_nodes_[i].x;
                    ps.pose.position.y  = prm_nodes_[i].y;
                    ps.pose.orientation = goal_.pose.orientation;
                    path.poses.push_back(ps);
                }

                // goal pose
                geometry_msgs::msg::PoseStamped pg;
                pg.header = path.header;
                pg.pose.position.x  = gx; pg.pose.position.y = gy;
                pg.pose.orientation = goal_.pose.orientation;
                path.poses.push_back(pg);

                RCLCPP_INFO(get_logger(),
                    "PRM A*: %zu waypoints, explored %zu nodes",
                    path.poses.size(), closed.size());
                return path;
            }

            for (int nb : prm_adj_[cur.idx]) {
                if (closed.count(nb)) continue;

                // ── dynamic obstacle gate ─────────────────────────────────
                // Skip PRM nodes inside clearance zone of moving obstacles.
                // clearance = obs.radius + dynamic_obs_clearance_
                // ensures replanned path is outside CBF influence zone
                // → robot follows new path at full speed, CBF inactive.
                if (nodeBlockedByDynamicObs(prm_nodes_[nb].x, prm_nodes_[nb].y))
                    continue;

                double ec  = euclideanDistance(
                    prm_nodes_[cur.idx].x, prm_nodes_[cur.idx].y,
                    prm_nodes_[nb].x,      prm_nodes_[nb].y);
                double tg  = cur.g_cost + ec;

                if (!all.count(nb) || tg < all[nb].g_cost) {
                    PNode nn;
                    nn.idx    = nb;
                    nn.parent = cur.idx;
                    nn.g_cost = tg;
                    nn.h_cost = euclideanDistance(
                        prm_nodes_[nb].x, prm_nodes_[nb].y,
                        prm_nodes_[gn].x, prm_nodes_[gn].y) * heuristic_weight_;
                    all[nb] = nn; open.push(nn);
                }
            }
        }

        RCLCPP_WARN(get_logger(), "PRM A* failed, explored %zu", closed.size());
        return path;
    }

    // ── path densification ────────────────────────────────────────────────────
    nav_msgs::msg::Path interpolatePath(
        const nav_msgs::msg::Path& input, double step_m = 0.1)
    {
        nav_msgs::msg::Path out;
        out.header = input.header;
        if (input.poses.size() < 2) return input;

        for (size_t i = 0; i < input.poses.size()-1; ++i) {
            double x0 = input.poses[i].pose.position.x;
            double y0 = input.poses[i].pose.position.y;
            double x1 = input.poses[i+1].pose.position.x;
            double y1 = input.poses[i+1].pose.position.y;
            double len = std::hypot(x1-x0, y1-y0);
            int steps = std::max(1, (int)(len / step_m));
            for (int s = 0; s < steps; ++s) {
                double t = (double)s / steps;
                geometry_msgs::msg::PoseStamped ps;
                ps.header = out.header;
                ps.pose.position.x = x0 + t*(x1-x0);
                ps.pose.position.y = y0 + t*(y1-y0);
                ps.pose.orientation.w = 1.0;
                out.poses.push_back(ps);
            }
        }
        out.poses.push_back(input.poses.back());
        return out;
    }

    // ── visualisation ─────────────────────────────────────────────────────────
    void publishExplored(const std::vector<int>& explored) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_.header.frame_id;
        m.header.stamp    = now();
        m.ns = "astar"; m.id = 0;
        m.type   = visualization_msgs::msg::Marker::POINTS;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = m.scale.y = 0.05;
        m.color.r=0; m.color.g=1; m.color.b=0; m.color.a=0.5f;
        for (int idx : explored) {
            int x, y; indexToCoord(idx, x, y);
            geometry_msgs::msg::Point p;
            mapToWorld(x, y, p.x, p.y); p.z = 0.05;
            m.points.push_back(p);
        }
        explored_pub_->publish(m);
    }

    // ── plan entry point ─────────────────────────────────────────────────────
    void planPath() {
        if (!have_map_ || !have_goal_) {
            RCLCPP_WARN(get_logger(), "Missing: map=%s goal=%s",
                have_map_?"✓":"✗", have_goal_?"✓":"✗");
            return;
        }
        geometry_msgs::msg::PoseStamped sp;
        if (!getStartPose(sp)) {
            RCLCPP_WARN(get_logger(), "Start pose not available"); return;
        }
        if (use_prm_ && !have_prm_) {
            RCLCPP_WARN(get_logger(), "PRM not ready"); return;
        }

        auto t0 = now();
        nav_msgs::msg::Path path;

        if (use_prm_) {
            path = prmAStar();
        } else {
            int sx, sy, gx, gy;
            double syaw = quat_to_yaw(sp.pose.orientation);
            double gyaw = quat_to_yaw(goal_.pose.orientation);
            if (!worldToMap(sp.pose.position.x, sp.pose.position.y, sx, sy) ||
                !worldToMap(goal_.pose.position.x, goal_.pose.position.y, gx, gy)) {
                RCLCPP_ERROR(get_logger(), "Start or goal outside map"); return;
            }
            if (isOccupied(sx, sy) || isOccupied(gx, gy)) {
                RCLCPP_ERROR(get_logger(), "Start or goal occupied"); return;
            }
            path = gridAStar(sx, sy, syaw, gx, gy, gyaw);
        }

        double ms = (now() - t0).seconds() * 1000.0;
        if (!path.poses.empty()) {
            auto dense = interpolatePath(path, 0.10);
            path_pub_->publish(dense);
            RCLCPP_INFO(get_logger(), "Published %zu poses (%.1fms)",
                dense.poses.size(), ms);
        } else {
            RCLCPP_WARN(get_logger(), "No path found (%.1fms)", ms);
        }
    }

    // ── callbacks ─────────────────────────────────────────────────────────────
    void onMap(nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        if (have_map_) return;
        map_ = *m; have_map_ = true;
        inflateMap();
        RCLCPP_INFO(get_logger(), "Map received (%dx%d res=%.3f inflation=%.2fm)",
            map_.info.width, map_.info.height,
            map_.info.resolution, inflation_radius_m_);
        planPath();
    }

    void onStart(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr s) {
        start_.header = s->header;
        start_.pose   = s->pose.pose;
        have_start_   = true;
        RCLCPP_INFO(get_logger(), "Start set: (%.2f, %.2f)",
            start_.pose.position.x, start_.pose.position.y);
        planPath();
    }

    void onGoal(geometry_msgs::msg::PoseStamped::SharedPtr g) {
        goal_      = *g;
        have_goal_ = true;
        RCLCPP_INFO(get_logger(), "Goal set: (%.2f, %.2f, %.2f°)",
            goal_.pose.position.x, goal_.pose.position.y,
            quat_to_yaw(goal_.pose.orientation) * 180.0/M_PI);
        planPath();
    }

    void onPrmNodes(visualization_msgs::msg::Marker::SharedPtr m) {
        prm_nodes_.clear();
        prm_nodes_.reserve(m->points.size());
        for (auto& p : m->points) prm_nodes_.push_back({p.x, p.y});
        RCLCPP_INFO(get_logger(), "PRM: %zu nodes received", prm_nodes_.size());
        if (have_prm_ && prm_adj_.size() == prm_nodes_.size()) planPath();
    }

    void onPrmAdjacency(std_msgs::msg::Int32MultiArray::SharedPtr a) {
        prm_adj_.clear();
        size_t i = 0;
        while (i < a->data.size()) {
            int n = a->data[i++];
            std::vector<int> nb;
            for (int k = 0; k < n && i < a->data.size(); ++k)
                nb.push_back(a->data[i++]);
            prm_adj_.push_back(nb);
        }
        have_prm_ = true;
        RCLCPP_INFO(get_logger(), "PRM: adjacency for %zu nodes", prm_adj_.size());
        if (prm_adj_.size() == prm_nodes_.size()) planPath();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}