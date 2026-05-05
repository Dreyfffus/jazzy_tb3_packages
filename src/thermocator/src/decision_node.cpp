#include "thermocator/decision_node.hpp"

#include <algorithm>
#include <limits>

namespace thermocator {

DecisionNode::DecisionNode()
    : Node("decision_node") {
    declare_parameter("hot_threshold", 60.0);
    declare_parameter("frontier_min_distance", 0.5);  // meters
    declare_parameter("investigation_duration", 3.0); // seconds at goal
    declare_parameter("control_rate", 1.0);           // Hz
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("robot_frame", std::string("base_footprint"));

    hot_threshold_ = get_parameter("hot_threshold").as_double();
    frontier_min_distance_ = get_parameter("frontier_min_distance").as_double();
    investigation_duration_ = get_parameter("investigation_duration").as_double();
    control_rate_ = get_parameter("control_rate").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(1);
    qos.transient_local().reliable();

    thermal_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/thermal_map", qos,
        std::bind(&DecisionNode::thermalMapCallback, this,
                  std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    const auto period =
        std::chrono::duration<double>(1.0 / control_rate_);

    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DecisionNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "DecisionNode ready — hot_threshold: %.1f  min_frontier_dist: %.2fm",
                hot_threshold_, frontier_min_distance_);
}

void DecisionNode::thermalMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    thermal_map_ = msg;
    map_received_ = true;
}

void DecisionNode::controlLoop() {
    switch (state_) {
    case ExplorationState::IDLE:
        handleIdle();
        break;
    case ExplorationState::SCANNING:
        handleScanning();
        break;
    case ExplorationState::NAVIGATING:
        handleNavigating();
        break;
    case ExplorationState::INVESTIGATING:
        handleInvestigating();
        break;
    case ExplorationState::COMPLETE:
        handleComplete();
        break;
    }
}

void DecisionNode::handleIdle() {
    if (!map_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for thermal map ...");
        return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for Nav2 action server ...");
        return;
    }

    RCLCPP_INFO(get_logger(), "Thermal map and Nav2 ready — starting exploration");
    state_ = ExplorationState::SCANNING;
}

void DecisionNode::handleScanning() {
    const auto pose = getRobotPose();
    if (!pose.has_value()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot get robot pose — skipping scan");
        return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_copy = thermal_map_;
    }

    auto frontiers = detectFrontiers(*map_copy, pose->first, pose->second);

    if (frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "No frontiers found — exploration complete");
        state_ = ExplorationState::COMPLETE;
        return;
    }

    const auto best = selectBestFrontier(frontiers, pose->first, pose->second);

    RCLCPP_INFO(get_logger(),
                "Selected frontier at (%.2f, %.2f) score=%.2f dist=%.2f",
                best.world_x, best.world_y, best.heat_score, best.distance);

    current_goal_x_ = best.world_x;
    current_goal_y_ = best.world_y;
    goal_active_ = false;
    goal_succeeded_ = false;
    goal_failed_ = false;

    sendGoal(best.world_x, best.world_y);
    state_ = ExplorationState::NAVIGATING;
}

void DecisionNode::handleNavigating() {
    if (goal_failed_) {
        RCLCPP_WARN(get_logger(),
                    "Goal failed — returning to scanning");
        goal_failed_ = false;
        state_ = ExplorationState::SCANNING;
        return;
    }

    if (goal_succeeded_) {
        RCLCPP_INFO(get_logger(),
                    "Reached frontier — investigating for %.1fs",
                    investigation_duration_);
        investigation_start_ = now();
        goal_succeeded_ = false;
        state_ = ExplorationState::INVESTIGATING;
        return;
    }

    // Still navigating — check if the goal has become unsafe
    // (thermal map updated and goal cell is now lethal)
    nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_copy = thermal_map_;
    }

    const auto &info = map_copy->info;
    const int col = static_cast<int>(
        std::floor((current_goal_x_ - info.origin.position.x) / info.resolution));
    const int row = static_cast<int>(
        std::floor((current_goal_y_ - info.origin.position.y) / info.resolution));

    if (col >= 0 && col < static_cast<int>(info.width) &&
        row >= 0 && row < static_cast<int>(info.height)) {
        const std::size_t idx =
            static_cast<std::size_t>(row) * info.width + col;
        const int8_t value = map_copy->data[idx];

        // Goal cell became dangerously hot — abort and rescan
        if (value > static_cast<int8_t>(hot_threshold_)) {
            RCLCPP_WARN(get_logger(),
                        "Goal cell became hot (value=%d) — cancelling and rescanning", value);
            cancelGoal();
            state_ = ExplorationState::SCANNING;
        }
    }
}

void DecisionNode::handleInvestigating() {
    const auto elapsed =
        (now() - investigation_start_).seconds();

    if (elapsed >= investigation_duration_) {
        RCLCPP_INFO(get_logger(),
                    "Investigation complete — rescanning for frontiers");
        state_ = ExplorationState::SCANNING;
    }
}

void DecisionNode::handleComplete() {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Exploration complete — no more heat zone frontiers");

    // Periodically recheck in case the thermal map updates
    // and new frontiers appear as the robot moves
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!thermal_map_)
            return;
    }

    const auto pose = getRobotPose();
    if (!pose.has_value())
        return;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_copy = thermal_map_;
    }

    auto frontiers = detectFrontiers(*map_copy, pose->first, pose->second);
    if (!frontiers.empty()) {
        RCLCPP_INFO(get_logger(),
                    "New frontiers detected — resuming exploration");
        state_ = ExplorationState::SCANNING;
    }
}

std::vector<Frontier> DecisionNode::detectFrontiers(
    const nav_msgs::msg::OccupancyGrid &grid,
    double robot_x, double robot_y) const {
    std::vector<Frontier> frontiers;
    const auto &info = grid.info;

    for (uint32_t row = 1; row < info.height - 1; ++row) {
        for (uint32_t col = 1; col < info.width - 1; ++col) {
            if (!isFrontierCell(grid, static_cast<int>(row), static_cast<int>(col)))
                continue;

            const double wx =
                info.origin.position.x + (col + 0.5) * info.resolution;
            const double wy =
                info.origin.position.y + (row + 0.5) * info.resolution;

            const double dx = wx - robot_x;
            const double dy = wy - robot_y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            // Skip frontiers too close — robot is already investigating them
            if (dist < frontier_min_distance_)
                continue;

            // Heat score — average of neighboring hot cell values
            // Higher score = stronger heat gradient = more interesting boundary
            double heat_sum = 0.0;
            int heat_count = 0;
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    const std::size_t idx =
                        static_cast<std::size_t>(row + dr) * info.width + (col + dc);
                    const int8_t v = grid.data[idx];
                    if (v > 0) {
                        heat_sum += static_cast<double>(v);
                        ++heat_count;
                    }
                }
            }
            const double heat_score =
                heat_count > 0 ? heat_sum / heat_count : 0.0;

            frontiers.push_back({wx, wy, heat_score, dist});
        }
    }

    return frontiers;
}

bool DecisionNode::isFrontierCell(
    const nav_msgs::msg::OccupancyGrid &grid,
    int row, int col) const {
    const auto &info = grid.info;
    const std::size_t idx =
        static_cast<std::size_t>(row) * info.width + col;

    const int8_t cell_value = grid.data[idx];

    // The cell itself must be cold or unknown — not hot
    // A frontier is a safe cell bordering a hot zone
    const double hot_norm =
        static_cast<double>(hot_threshold_) / 100.0 * 100.0;

    if (cell_value > static_cast<int8_t>(hot_norm))
        return false;

    // Check 4-connected neighbours for at least one hot cell
    const int neighbours[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (const auto &n : neighbours) {
        const int nr = row + n[0];
        const int nc = col + n[1];

        if (nr < 0 || nr >= static_cast<int>(info.height) ||
            nc < 0 || nc >= static_cast<int>(info.width))
            continue;

        const std::size_t nidx =
            static_cast<std::size_t>(nr) * info.width + nc;
        const int8_t nv = grid.data[nidx];

        if (nv > static_cast<int8_t>(hot_norm))
            return true;
    }

    return false;
}

Frontier DecisionNode::selectBestFrontier(
    std::vector<Frontier> &frontiers,
    double robot_x, double robot_y) const {
    // Score = heat_score / distance
    // Prefers high-heat nearby frontiers over distant weak ones
    // distance is already stored in the Frontier struct

    return *std::max_element(frontiers.begin(), frontiers.end(),
                             [](const Frontier &a, const Frontier &b) {
                                 const double score_a = a.heat_score / (a.distance + 1e-6);
                                 const double score_b = b.heat_score / (b.distance + 1e-6);
                                 return score_a < score_b;
                             });
}

void DecisionNode::sendGoal(double x, double y) {
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = map_frame_;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;

    // Face the goal direction — orientation derived from robot→goal vector
    // is handled by Nav2, so we set a neutral quaternion (identity)
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&DecisionNode::goalResponseCallback, this,
                  std::placeholders::_1);

    send_goal_options.feedback_callback =
        std::bind(&DecisionNode::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
        std::bind(&DecisionNode::resultCallback, this,
                  std::placeholders::_1);

    nav_client_->async_send_goal(goal, send_goal_options);
    goal_active_ = true;

    RCLCPP_INFO(get_logger(),
                "Sent navigation goal to (%.2f, %.2f)", x, y);
}

void DecisionNode::cancelGoal() {
    if (current_goal_handle_) {
        nav_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_.reset();
    }
    goal_active_ = false;
}

void DecisionNode::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_WARN(get_logger(), "Goal was rejected by Nav2");
        goal_active_ = false;
        goal_failed_ = true;
        return;
    }

    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
}

void DecisionNode::feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
        feedback) {
    RCLCPP_DEBUG(get_logger(),
                 "Distance remaining: %.2fm",
                 feedback->distance_remaining);
}

void DecisionNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    goal_active_ = false;

    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Navigation succeeded");
        goal_succeeded_ = true;
        break;

    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Navigation aborted");
        goal_failed_ = true;
        break;

    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Navigation cancelled");
        goal_failed_ = true;
        break;

    default:
        RCLCPP_WARN(get_logger(), "Unknown navigation result");
        goal_failed_ = true;
        break;
    }
}

std::optional<std::pair<double, double>> DecisionNode::getRobotPose() const {
    try {
        const auto transform = tf_buffer_->lookupTransform(
            map_frame_, robot_frame_,
            rclcpp::Time(0),
            rclcpp::Duration::from_seconds(0.1));

        return std::make_pair(
            transform.transform.translation.x,
            transform.transform.translation.y);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "TF lookup failed: %s", ex.what());
        return std::nullopt;
    }
}

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
