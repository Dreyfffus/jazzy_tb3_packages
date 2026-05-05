#pragma once
#include <atomic>
#include <future>
#include <memory>

#include <nav_msgs/msg/occupancy_grid.h>
#include <rclcpp/rclcpp.hpp>

#include "thermocator/thermal_grid.hpp"
namespace thermocator {
class WorldInitializer : public rclcpp::Node {
  public:
    explicit WorldInitializer(std::shared_ptr<ThermalGrid> grid)
        : Node("world_initializer"), _grid(std::move(grid)), _fired(false) {
        rclcpp::QoS qos(1);
        qos.transient_local();
        qos.reliable();

        _sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                if (this->_fired.exchange(true))
                    return;
                this->_grid->Initialize(*msg);
                RCLCPP_INFO(get_logger(),
                            "ThermalGrid initialised - %u x %u cells, %.3f m/cell, origin (%.2f, %.2f)",
                            msg->info.width,
                            msg->info.height,
                            msg->info.resolution,
                            msg->info.origin.position.x,
                            msg->info.origin.position.y);
                _promise.set_value();
            });

        RCLCPP_INFO(get_logger(), "Waiting for /map ...");
    }
    std::shared_future<void> getFuture() { return _promise.get_future().share(); }

  private:
    std::shared_ptr<ThermalGrid> _grid;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _sub;
    std::promise<void> _promise;
    std::atomic<bool> _fired;
};
} // namespace thermocator
