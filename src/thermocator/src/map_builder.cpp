
#include "thermocator/map_builder.hpp"
namespace thermocator {
void ThermalMapBuilder::ThermalCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    if (!_grid->IsInitialized())
        return;

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = _tf_buffer->lookupTransform(
            _map_frame,
            _robot_frame,
            msg->header.stamp,
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
