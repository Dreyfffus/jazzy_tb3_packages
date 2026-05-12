#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace thermocator {

enum class ExplorationState {
    IDLE,
    SCANNING,
    NAVIGATING,
    INVESTIGATING,
    COMPLETE
};

struct Frontier {
    double world_x = 0.0;
    double world_y = 0.0;
    double distance = 0.0;
    double unknown_hot_neighbors = 0.0;
    double mean_neighbor_heat = 0.0;
    double distance_to_hottest = 0.0;
    double cold_penalty = 0.0;
    double final_score = 0.0;
};

class DecisionNode : public rclcpp::Node {
  public:
    explicit DecisionNode();
    ~DecisionNode() override = default;

  private:
    // Callbacks
    void thermalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void spatialMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // Control loop
    void controlLoop();
    void handleIdle();
    void handleScanning();
    void handleNavigating();
    void handleInvestigating();
    void handleComplete();

    // Phase detection
    bool hasHeatData(const nav_msgs::msg::OccupancyGrid &thermal) const;

    // Frontier detection
    std::vector<Frontier> detectSpatialFrontiers(
        const nav_msgs::msg::OccupancyGrid &spatial,
        double robot_x, double robot_y) const;

    std::vector<Frontier> detectThermalFrontiers(
        const nav_msgs::msg::OccupancyGrid &spatial,
        const nav_msgs::msg::OccupancyGrid &thermal,
        double robot_x, double robot_y) const;

    Frontier selectBestFrontier(std::vector<Frontier> &frontiers) const;

    // Navigation
    void sendGoal(double x, double y);
    void cancelGoal();

    // Action client callbacks
    void goalResponseCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::SharedPtr &handle);

    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void resultCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    // TF
    std::optional<std::pair<double, double>> getRobotPose() const;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr spatial_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Maps
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_map_;
    bool thermal_map_received_ = false;
    bool spatial_map_received_ = false;

    // State
    ExplorationState state_ = ExplorationState::IDLE;
    std::atomic<bool> goal_active_{false};
    std::atomic<bool> goal_succeeded_{false};
    std::atomic<bool> goal_failed_{false};
    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;
    rclcpp::Time investigation_start_;

    // Parameters
    double heat_detection_threshold_;
    double frontier_min_distance_;
    double investigation_duration_;
    double control_rate_;
    double scoring_radius_;
    double w_unknown_hot_;
    double w_dist_hottest_;
    double w_cold_penalty_;
    std::string map_frame_;
    std::string robot_frame_;
};

} // namespace thermocator
