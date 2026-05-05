#pragma once
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <mutex>
#include <string>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace thermocator {
class ThermalLayer : public nav2_costmap_2d::Layer {
  public:
    ThermalLayer() = default;
    ~ThermalLayer() override = default;

    void onInitialize() override;
    void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y,
        double *max_x, double *max_y) override;
    void updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j,
        int max_i, int max_j) override;
    void reset() override;
    bool isClearable() override { return false; }

  private:
    void ThermalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    uint8_t ThermalToCost(int8_t thermal_value) const;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _sub;
    std::mutex _map_mutex;
    nav_msgs::msg::OccupancyGrid::SharedPtr _thermal_map;
    bool _received_map = false;

    double _hot_threshold;
    double _lethal_threshold;
    double _cold_threshold;
    std::string _thermal_map_topic;
};
} // namespace thermocator
