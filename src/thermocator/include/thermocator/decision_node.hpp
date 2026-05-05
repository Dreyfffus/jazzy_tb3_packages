#pragma once

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace thermocator {

enum class ExplorationState {
    IDLE,          // Waiting for thermal map to initialize
    SCANNING,      // Searching for heat zone frontiers
    NAVIGATING,    // Moving toward a frontier goal
    INVESTIGATING, // Robot has reached goal, taking readings
    COMPLETE       // No more frontiers to explore
};

struct Frontier {
    double world_x;
    double world_y;
    double heat_score; // Higher = more interesting boundary
    double distance;   // Distance from robot at time of detection
};

class DecisionNode : public rclcpp::Node {
  public:
    explicit DecisionNode();
    ~DecisionNode() override = default;

  private:
    void thermalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void controlLoop();

    void handleIdle();
    void handleScanning();
    void handleNavigating();
    void handleInvestigating();
    void handleComplete();

    std::vector<Frontier> detectFrontiers(
        const nav_msgs::msg::OccupancyGrid &grid,
        double robot_x, double robot_y) const;

    // Returns true if a cell is a frontier —
    // borders a hot zone but is itself cold or unknown
    bool isFrontierCell(
        const nav_msgs::msg::OccupancyGrid &grid,
        int row, int col) const;

    // Scores and sorts frontier candidates
    Frontier selectBestFrontier(
        std::vector<Frontier> &frontiers,
        double /*robot_x*/, double /*robot_y*/) const;

    void sendGoal(double x, double y);
    void cancelGoal();

    // Action client callbacks
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle);

    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void resultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    std::optional<std::pair<double, double>> getRobotPose() const;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
        current_goal_handle_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    ExplorationState state_ = ExplorationState::IDLE;

    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map_;
    bool map_received_ = false;

    bool goal_active_ = false;
    bool goal_succeeded_ = false;
    bool goal_failed_ = false;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    rclcpp::Time investigation_start_;

    double hot_threshold_;
    double frontier_min_distance_;
    double investigation_duration_;
    double control_rate_;
    std::string map_frame_;
    std::string robot_frame_;
};

} // namespace thermocator
