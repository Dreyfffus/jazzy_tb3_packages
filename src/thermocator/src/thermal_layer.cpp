#include "thermocator/thermal_layer.hpp"

#include <algorithm>
#include <cmath>

#include <nav2_costmap_2d/costmap_math.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(thermocator::ThermalLayer, nav2_costmap_2d::Layer)

namespace thermocator {

void ThermalLayer::onInitialize() {
    // Nav2 provides a node handle via the Layer base class — use it
    // Do NOT create your own node here
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("ThermalLayer: unable to lock node handle");
    }

    declareParameter("cold_threshold", rclcpp::ParameterValue(25.0));
    declareParameter("hot_threshold", rclcpp::ParameterValue(60.0));
    declareParameter("lethal_threshold", rclcpp::ParameterValue(75.0));
    declareParameter("_thermal_maptopic", rclcpp::ParameterValue(std::string("/thermal_map")));

    node->get_parameter(name_ + ".cold_threshold", _cold_threshold);
    node->get_parameter(name_ + ".hot_threshold", _hot_threshold);
    node->get_parameter(name_ + ".lethal_threshold", _lethal_threshold);
    node->get_parameter(name_ + "._thermal_maptopic", _thermal_map_topic);

    // ── Subscribe to thermal map ──────────────────────────────────────────────
    // transient_local — must match ThermalMapBuilder publisher QoS
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();

    _sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        _thermal_map_topic, qos,
        std::bind(&ThermalLayer::thermalMapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(),
                "ThermalLayer initialised — topic: %s  cold: %.1f  hot: %.1f  lethal: %.1f",
                _thermal_map_topic.c_str(), _cold_threshold, _hot_threshold, _lethal_threshold);

    current_ = true;
}

void ThermalLayer::ThermalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(_map_mutex);
    _thermal_map = msg;
    _received_map = true;
}

void ThermalLayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
    double *min_x, double *min_y,
    double *max_x, double *max_y) {
    if (!_received_map)
        return;

    std::lock_guard<std::mutex> lock(_map_mutex);

    // Expand bounds to cover the full thermal map extent
    // Nav2 will only call updateCosts() for this region
    const auto &info = _thermal_map->info;

    const double x_min = info.origin.position.x;
    const double y_min = info.origin.position.y;
    const double x_max = x_min + info.width * info.resolution;
    const double y_max = y_min + info.height * info.resolution;

    *min_x = std::min(*min_x, x_min);
    *min_y = std::min(*min_y, y_min);
    *max_x = std::max(*max_x, x_max);
    *max_y = std::max(*max_y, y_max);
}

void ThermalLayer::updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid,
    int min_i, int min_j,
    int max_i, int max_j) {
    if (!_received_map || !enabled_)
        return;

    std::lock_guard<std::mutex> lock(_map_mutex);

    const auto &info = _thermal_map->info;

    // Iterate over every costmap cell in the update window
    for (int j = min_j; j < max_j; ++j) {
        for (int i = min_i; i < max_i; ++i) {
            // Convert costmap cell index to world coordinates
            double world_x, world_y;
            master_grid.mapToWorld(
                static_cast<unsigned int>(i),
                static_cast<unsigned int>(j),
                world_x, world_y);

            // Convert world coordinates to thermal grid indices
            const int thermal_col = static_cast<int>(
                std::floor((world_x - info.origin.position.x) / info.resolution));
            const int thermal_row = static_cast<int>(
                std::floor((world_y - info.origin.position.y) / info.resolution));

            // Skip if outside thermal grid bounds
            if (thermal_col < 0 || thermal_col >= static_cast<int>(info.width) ||
                thermal_row < 0 || thermal_row >= static_cast<int>(info.height)) {
                continue;
            }

            const std::size_t thermal_idx =
                static_cast<std::size_t>(thermal_row) * info.width +
                static_cast<std::size_t>(thermal_col);

            const int8_t thermal_value = _thermal_map->data[thermal_idx];

            // Unknown cells — leave master costmap unchanged
            if (thermal_value == -1)
                continue;

            const uint8_t thermal_cost = thermalToCost(thermal_value);

            // Skip free cells — no need to overwrite
            if (thermal_cost == nav2_costmap_2d::FREE_SPACE)
                continue;

            // Use std::max — never reduce costs set by other layers
            const unsigned int master_idx = master_grid.getIndex(
                static_cast<unsigned int>(i),
                static_cast<unsigned int>(j));

            const uint8_t current_cost = master_grid.getCost(
                static_cast<unsigned int>(i),
                static_cast<unsigned int>(j));

            // Do not overwrite lethal obstacles set by other layers
            if (current_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
                continue;

            master_grid.setCost(
                static_cast<unsigned int>(i),
                static_cast<unsigned int>(j),
                std::max(current_cost, thermal_cost));
        }
    }
}

uint8_t ThermalLayer::ThermalToCost(int8_t thermal_value) const {
    // thermal_value is [0, 100] normalized
    // 0   = cold_threshold
    // 100 = hot_threshold and above

    const double normalized = static_cast<double>(thermal_value) / 100.0;
    const double temp_estimate =
        _cold_threshold + normalized * (_lethal_threshold - _cold_threshold);

    if (temp_estimate <= _cold_threshold) {
        return nav2_costmap_2d::FREE_SPACE; // 0
    }

    if (temp_estimate >= _lethal_threshold) {
        return nav2_costmap_2d::LETHAL_OBSTACLE; // 254
    }

    if (temp_estimate >= _hot_threshold) {
        // Hot but not lethal — inscribed cost, strong repulsion
        return nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE; // 253
    }

    // Warm zone — scale linearly between 1 and 252
    const double range = _hot_threshold - _cold_threshold;
    const double heat_ratio = (temp_estimate - _cold_threshold) / range;
    return static_cast<uint8_t>(1.0 + heat_ratio * 251.0);
}

void ThermalLayer::reset() {
    std::lock_guard<std::mutex> lock(_map_mutex);
    _received_map = false;
    _thermal_map.reset();
    current_ = true;
}

} // namespace thermocator
