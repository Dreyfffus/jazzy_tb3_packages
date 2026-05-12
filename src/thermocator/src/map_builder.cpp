
#include "thermocator/map_builder.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
namespace thermocator {
void ThermalMapBuilder::ThermalCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    if (!_grid->IsInitialized())
        return;

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = _tf_buffer->lookupTransform(
            _map_frame,
            _robot_frame,
            rclcpp::Time(0),
            rclcpp::Duration::from_seconds(_tf_timeout));
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup failed (%s) - dropping reading", e.what());
        return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;

    {
        std::lock_guard<std::mutex> lock(_grid_mutex);
        _grid->Update(x, y, msg->temperature);
    }
}

void ThermalMapBuilder::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(_grid_mutex);

    if (!_grid->IsInitialized()) {
        _grid->Initialize(*msg);
        return;
    }

    const auto &new_info = msg->info;
    const auto &old_info = _grid->getInfo();

    // Compare in cell space — ignore sub-cell float drift
    const double res = new_info.resolution;
    const double dx = std::abs(new_info.origin.position.x - old_info.origin.position.x);
    const double dy = std::abs(new_info.origin.position.y - old_info.origin.position.y);

    const bool origin_shifted = (dx > res * 0.5) || (dy > res * 0.5);
    const bool dims_changed = new_info.width != old_info.width ||
                              new_info.height != old_info.height;

    if (origin_shifted || dims_changed) {
        _grid->Resize(new_info);
    }
}

void ThermalMapBuilder::PublishCallback() {
    if (!_grid->IsInitialized())
        return;

    nav_msgs::msg::OccupancyGrid out;

    out.header.stamp = now();
    out.header.frame_id = _map_frame;
    out.info = _grid->getInfo();

    {
        std::lock_guard<std::mutex> lock(_grid_mutex);
        out.data = _grid->ToOccupancyData(
            _cold_thresh,
            _hot_thresh,
            _min_conf);
    }

    _publisher->publish(out);

    RCLCPP_DEBUG(get_logger(), "Published /thermal_map (%zu cells)", out.data.size());
}
} // namespace thermocator
