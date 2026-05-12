#pragma once
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "thermocator/thermal_grid.hpp"

namespace thermocator {

class ThermalMapBuilder : public rclcpp::Node {
  public:
    explicit ThermalMapBuilder(std::shared_ptr<ThermalGrid> grid)
        : Node("thermal_map_builder"), _grid(std::move(grid)) {

        declare_parameter("map_frame", "map");
        declare_parameter("robot_frame", "base_footprint");
        declare_parameter("cold_threshold", 20.0);
        declare_parameter("hot_threshold", 50.0);
        declare_parameter("min_confidence", 0.5);
        declare_parameter("publish_rate", 1.0); // Hz
        declare_parameter("tf_timeout", 0.1);   // Sec
        set_parameter(rclcpp::Parameter("use_sim_time", true));

        _map_frame = get_parameter("map_frame").as_string();
        _robot_frame = get_parameter("robot_frame").as_string();
        _cold_thresh = get_parameter("cold_threshold").as_double();
        _hot_thresh = get_parameter("hot_threshold").as_double();
        _min_conf = static_cast<float>(get_parameter("min_confidence").as_double());
        _tf_timeout = get_parameter("tf_timeout").as_double();

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        rclcpp::QoS pub_qos(1);
        pub_qos.transient_local();
        pub_qos.reliable();

        rclcpp::QoS map_qos(1);
        map_qos.transient_local();
        map_qos.reliable();

        _publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/thermal_map", pub_qos);
        _thermal_sub = create_subscription<sensor_msgs::msg::Temperature>("/thermal_reading", rclcpp::QoS(10),
                                                                          std::bind(&ThermalMapBuilder::ThermalCallback, this, std::placeholders::_1));
        _ogrid_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&ThermalMapBuilder::MapCallback, this, std::placeholders::_1));

        const double rate = get_parameter("publish_rate").as_double();
        const auto period = std::chrono::duration<double>(1.0 / rate);

        _timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ThermalMapBuilder::PublishCallback, this));

        RCLCPP_INFO(get_logger(), "Thermal Map Builder ready - publishing /thermal_map at %.1f Hz", rate);
    }

  private:
    void ThermalCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);
    void PublishCallback();
    void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    std::shared_ptr<ThermalGrid> _grid;
    std::mutex _grid_mutex;

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr _thermal_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _publisher;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _ogrid_sub;
    rclcpp::TimerBase::SharedPtr _timer;

    std::string _map_frame;
    std::string _robot_frame;
    double _cold_thresh;
    double _hot_thresh;
    float _min_conf;
    double _tf_timeout;
};
} // namespace thermocator
