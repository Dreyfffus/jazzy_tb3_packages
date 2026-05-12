#include "thermocator/decision_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace thermocator {

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

DecisionNode::DecisionNode() : Node("decision_node") {

    declare_parameter("heat_detection_threshold", 20.0);
    declare_parameter("frontier_min_distance", 0.5);
    declare_parameter("investigation_duration", 5.0);
    declare_parameter("control_rate", 1.0);
    declare_parameter("scoring_radius", 1.5);
    declare_parameter("w_unknown_hot", 3.0);
    declare_parameter("w_dist_hottest", 0.5);
    declare_parameter("w_cold_penalty", 0.2);
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("robot_frame", std::string("base_footprint"));
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    heat_detection_threshold_ = get_parameter("heat_detection_threshold").as_double();
    frontier_min_distance_ = get_parameter("frontier_min_distance").as_double();
    investigation_duration_ = get_parameter("investigation_duration").as_double();
    control_rate_ = get_parameter("control_rate").as_double();
    scoring_radius_ = get_parameter("scoring_radius").as_double();
    w_unknown_hot_ = get_parameter("w_unknown_hot").as_double();
    w_dist_hottest_ = get_parameter("w_dist_hottest").as_double();
    w_cold_penalty_ = get_parameter("w_cold_penalty").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();

    thermal_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/thermal_map", latched_qos,
        std::bind(&DecisionNode::thermalMapCallback, this, std::placeholders::_1));

    spatial_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", latched_qos,
        std::bind(&DecisionNode::spatialMapCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DecisionNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "DecisionNode ready -- heat_threshold: %.1f  scoring_radius: %.2fm  "
                "weights: unknown_hot=%.2f  dist_hottest=%.2f  cold=%.2f",
                heat_detection_threshold_, scoring_radius_,
                w_unknown_hot_, w_dist_hottest_, w_cold_penalty_);
}

// ----------------------------------------------------------------------------
// Map callbacks
// ----------------------------------------------------------------------------

void DecisionNode::thermalMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    thermal_map_ = msg;
    thermal_map_received_ = true;
}

void DecisionNode::spatialMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    spatial_map_ = msg;
    spatial_map_received_ = true;
}

// ----------------------------------------------------------------------------
// State machine
// ----------------------------------------------------------------------------

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
    if (!thermal_map_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for /thermal_map ...");
        return;
    }
    if (!spatial_map_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for /map ...");
        return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for Nav2 action server ...");
        return;
    }
    RCLCPP_INFO(get_logger(), "All systems ready -- starting exploration");
    state_ = ExplorationState::SCANNING;
}

void DecisionNode::handleScanning() {
    const auto pose = getRobotPose();
    if (!pose.has_value()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot get robot pose -- skipping scan");
        return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        thermal_copy = thermal_map_;
        spatial_copy = spatial_map_;
    }

    std::vector<Frontier> frontiers;

    if (hasHeatData(*thermal_copy)) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Phase 2 -- thermal mapping mode");
        frontiers = detectThermalFrontiers(
            *spatial_copy, *thermal_copy, pose->first, pose->second);

        if (frontiers.empty()) {
            RCLCPP_INFO(get_logger(),
                        "No thermal frontiers -- falling back to spatial exploration");
            frontiers = detectSpatialFrontiers(
                *spatial_copy, pose->first, pose->second);
        }
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Phase 1 -- spatial exploration mode");
        frontiers = detectSpatialFrontiers(
            *spatial_copy, pose->first, pose->second);
    }

    if (frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "No frontiers found -- exploration complete");
        state_ = ExplorationState::COMPLETE;
        return;
    }

    const auto best = selectBestFrontier(frontiers);

    RCLCPP_INFO(get_logger(),
                "Goal: (%.2f, %.2f)  score=%.3f  "
                "unknown_hot=%.2f  mean_heat=%.1f  "
                "dist_hottest=%.2f  cold_penalty=%.2f",
                best.world_x, best.world_y, best.final_score,
                best.unknown_hot_neighbors, best.mean_neighbor_heat,
                best.distance_to_hottest, best.cold_penalty);

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
        RCLCPP_WARN(get_logger(), "Goal failed -- returning to scanning");
        goal_failed_ = false;
        state_ = ExplorationState::SCANNING;
        return;
    }
    if (goal_succeeded_) {
        RCLCPP_INFO(get_logger(),
                    "Reached goal -- investigating for %.1fs", investigation_duration_);
        investigation_start_ = now();
        goal_succeeded_ = false;
        state_ = ExplorationState::INVESTIGATING;
    }
}

void DecisionNode::handleInvestigating() {
    const double elapsed = (now() - investigation_start_).seconds();
    if (elapsed >= investigation_duration_) {
        RCLCPP_INFO(get_logger(), "Investigation done -- rescanning");
        state_ = ExplorationState::SCANNING;
    }
}

void DecisionNode::handleComplete() {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                         "Exploration complete -- monitoring for new frontiers");

    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!spatial_map_)
            return;
        spatial_copy = spatial_map_;
    }

    const auto pose = getRobotPose();
    if (!pose.has_value())
        return;

    const auto frontiers = detectSpatialFrontiers(
        *spatial_copy, pose->first, pose->second);

    if (!frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "New frontiers detected -- resuming");
        state_ = ExplorationState::SCANNING;
    }
}

// ----------------------------------------------------------------------------
// Phase detection
// ----------------------------------------------------------------------------

bool DecisionNode::hasHeatData(
    const nav_msgs::msg::OccupancyGrid &thermal) const {
    const auto thresh = static_cast<int8_t>(heat_detection_threshold_);
    for (const auto &v : thermal.data) {
        if (v >= thresh)
            return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
// Phase 1 -- spatial frontiers
// Free cells adjacent to unknown cells, scored by proximity
// ----------------------------------------------------------------------------

std::vector<Frontier> DecisionNode::detectSpatialFrontiers(
    const nav_msgs::msg::OccupancyGrid &spatial,
    double robot_x, double robot_y) const {
    std::vector<Frontier> frontiers;
    const auto &info = spatial.info;
    const int nb[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    for (uint32_t row = 1; row < info.height - 1; ++row) {
        for (uint32_t col = 1; col < info.width - 1; ++col) {

            const size_t idx = static_cast<size_t>(row) * info.width + col;
            if (spatial.data[idx] != 0)
                continue;

            bool adj_unknown = false;
            for (const auto &n : nb) {
                const size_t nidx =
                    static_cast<size_t>(row + n[0]) * info.width + (col + n[1]);
                if (spatial.data[nidx] == -1) {
                    adj_unknown = true;
                    break;
                }
            }
            if (!adj_unknown)
                continue;

            const double wx =
                info.origin.position.x + (col + 0.5) * info.resolution;
            const double wy =
                info.origin.position.y + (row + 0.5) * info.resolution;
            const double dx = wx - robot_x;
            const double dy = wy - robot_y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < frontier_min_distance_)
                continue;

            Frontier f;
            f.world_x = wx;
            f.world_y = wy;
            f.distance = dist;
            f.final_score = 1.0 / (dist + 1e-6);
            frontiers.push_back(f);
        }
    }
    return frontiers;
}

// ----------------------------------------------------------------------------
// Phase 2 -- thermal frontiers
// Free spatial cells scored by unknown-adjacent-to-hot, heat proximity,
// and cold penalty
// ----------------------------------------------------------------------------

std::vector<Frontier> DecisionNode::detectThermalFrontiers(
    const nav_msgs::msg::OccupancyGrid &spatial,
    const nav_msgs::msg::OccupancyGrid &thermal,
    double robot_x, double robot_y) const {
    // Find the globally hottest known cell once
    double hottest_wx = robot_x;
    double hottest_wy = robot_y;
    {
        int8_t max_heat = 0;
        const auto &ti = thermal.info;
        for (uint32_t r = 0; r < ti.height; ++r) {
            for (uint32_t c = 0; c < ti.width; ++c) {
                const size_t idx = static_cast<size_t>(r) * ti.width + c;
                if (thermal.data[idx] > max_heat) {
                    max_heat = thermal.data[idx];
                    hottest_wx = ti.origin.position.x + (c + 0.5) * ti.resolution;
                    hottest_wy = ti.origin.position.y + (r + 0.5) * ti.resolution;
                }
            }
        }
    }

    std::vector<Frontier> frontiers;
    const auto &si = spatial.info;
    const auto &ti = thermal.info;
    const auto thresh = static_cast<int8_t>(heat_detection_threshold_);
    const int anb[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    const int scan = static_cast<int>(scoring_radius_ / ti.resolution);

    for (uint32_t row = 1; row < si.height - 1; ++row) {
        for (uint32_t col = 1; col < si.width - 1; ++col) {

            const size_t sidx = static_cast<size_t>(row) * si.width + col;
            if (spatial.data[sidx] != 0)
                continue;

            const double wx =
                si.origin.position.x + (col + 0.5) * si.resolution;
            const double wy =
                si.origin.position.y + (row + 0.5) * si.resolution;
            const double dx = wx - robot_x;
            const double dy = wy - robot_y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < frontier_min_distance_)
                continue;

            // Locate candidate in thermal grid
            const int tc =
                static_cast<int>((wx - ti.origin.position.x) / ti.resolution);
            const int tr =
                static_cast<int>((wy - ti.origin.position.y) / ti.resolution);

            double unknown_hot_score = 0.0;
            double heat_sum = 0.0;
            int heat_count = 0;
            double cold_penalty = 0.0;

            for (int dr = -scan; dr <= scan; ++dr) {
                for (int dc = -scan; dc <= scan; ++dc) {
                    const int nr = tr + dr;
                    const int nc = tc + dc;

                    if (nr < 0 || nr >= static_cast<int>(ti.height) ||
                        nc < 0 || nc >= static_cast<int>(ti.width))
                        continue;

                    const float cell_dist =
                        std::sqrt(static_cast<float>(dr * dr + dc * dc)) * ti.resolution;
                    if (cell_dist > scoring_radius_)
                        continue;

                    const size_t tidx =
                        static_cast<size_t>(nr) * ti.width + nc;
                    const int8_t tv = thermal.data[tidx];

                    if (tv == -1) {
                        // Unknown cell -- extremely interesting if adjacent to hot cell
                        for (const auto &n : anb) {
                            const int ar = nr + n[0];
                            const int ac = nc + n[1];
                            if (ar < 0 || ar >= static_cast<int>(ti.height) ||
                                ac < 0 || ac >= static_cast<int>(ti.width))
                                continue;
                            const size_t aidx =
                                static_cast<size_t>(ar) * ti.width + ac;
                            if (thermal.data[aidx] >= thresh) {
                                unknown_hot_score +=
                                    static_cast<double>(thermal.data[aidx]) /
                                    (static_cast<double>(cell_dist) + 1e-3);
                                break;
                            }
                        }
                    } else if (tv >= thresh) {
                        // Known hot cell
                        heat_sum += static_cast<double>(tv);
                        ++heat_count;
                    } else if (tv >= 0) {
                        // Known cold cell -- penalize
                        cold_penalty += 1.0;
                    }
                }
            }

            // Skip candidates with no thermal interest
            if (unknown_hot_score < 1e-6 && heat_count == 0)
                continue;

            const double mean_heat =
                heat_count > 0 ? heat_sum / static_cast<double>(heat_count) : 0.0;

            const double dhx = wx - hottest_wx;
            const double dhy = wy - hottest_wy;
            const double dist_to_hottest = std::sqrt(dhx * dhx + dhy * dhy);

            // Score: reward unknown-adjacent-to-hot, penalise distance from
            // hottest region and confirmed cold areas
            const double score =
                w_unknown_hot_ * unknown_hot_score * (mean_heat / 100.0 + 1.0) - w_dist_hottest_ * dist_to_hottest - w_cold_penalty_ * cold_penalty;

            Frontier f;
            f.world_x = wx;
            f.world_y = wy;
            f.distance = dist;
            f.unknown_hot_neighbors = unknown_hot_score;
            f.mean_neighbor_heat = mean_heat;
            f.distance_to_hottest = dist_to_hottest;
            f.cold_penalty = cold_penalty;
            f.final_score = score;
            frontiers.push_back(f);
        }
    }
    return frontiers;
}

// ----------------------------------------------------------------------------
// Selection
// ----------------------------------------------------------------------------

Frontier DecisionNode::selectBestFrontier(
    std::vector<Frontier> &frontiers) const {
    return *std::max_element(
        frontiers.begin(), frontiers.end(),
        [](const Frontier &a, const Frontier &b) {
            return a.final_score < b.final_score;
        });
}

// ----------------------------------------------------------------------------
// Navigation
// ----------------------------------------------------------------------------

void DecisionNode::sendGoal(double x, double y) {
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = map_frame_;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;

    auto opts =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    opts.goal_response_callback =
        std::bind(&DecisionNode::goalResponseCallback, this,
                  std::placeholders::_1);

    opts.feedback_callback =
        std::bind(&DecisionNode::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);

    opts.result_callback =
        std::bind(&DecisionNode::resultCallback, this,
                  std::placeholders::_1);

    nav_client_->async_send_goal(goal, opts);
    goal_active_ = true;
    RCLCPP_INFO(get_logger(), "Goal sent to (%.2f, %.2f)", x, y);
}

void DecisionNode::cancelGoal() {
    if (current_goal_handle_) {
        nav_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_.reset();
    }
    goal_active_ = false;
}

// ----------------------------------------------------------------------------
// Action client callbacks
// ----------------------------------------------------------------------------

void DecisionNode::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr &handle) {
    if (!handle) {
        RCLCPP_WARN(get_logger(), "Goal rejected by Nav2");
        goal_active_ = false;
        goal_failed_ = true;
        return;
    }
    current_goal_handle_ = handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
}

void DecisionNode::feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_DEBUG(get_logger(),
                 "Distance remaining: %.2fm", feedback->distance_remaining);
}

void DecisionNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
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

// ----------------------------------------------------------------------------
// TF
// ----------------------------------------------------------------------------

std::optional<std::pair<double, double>> DecisionNode::getRobotPose() const {
    try {
        const auto t = tf_buffer_->lookupTransform(
            map_frame_, robot_frame_,
            rclcpp::Time(0),
            rclcpp::Duration::from_seconds(0.1));
        return std::make_pair(
            t.transform.translation.x,
            t.transform.translation.y);
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup failed: %s", e.what());
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
