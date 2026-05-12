#include "thermal_broadcaster/heat_zone.hpp"
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace thermocator {

class ThermalBroadcaster : public rclcpp::Node {
  public:
    explicit ThermalBroadcaster()
        : Node("thermal_broadcaster"), _rng(std::random_device{}()) {
        declare_parameter("zone_centers_x", std::vector<double>{0.0});
        declare_parameter("zone_centers_y", std::vector<double>{0.0});
        declare_parameter("zone_peak_temps", std::vector<double>{80.0});
        declare_parameter("zone_sigmas", std::vector<double>{0.5});
        declare_parameter("noise_stdev", 0.5);
        declare_parameter("publish_rate", 2.5);
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("robot_frame", std::string("base_footprint"));
        set_parameter(rclcpp::Parameter("use_sim_time", true));

        const auto cx = get_parameter("zone_centers_x").as_double_array();
        const auto cy = get_parameter("zone_centers_y").as_double_array();
        const auto peaks = get_parameter("zone_peak_temps").as_double_array();
        const auto sigs = get_parameter("zone_sigmas").as_double_array();

        if (cx.size() != cy.size() ||
            cx.size() != peaks.size() ||
            cx.size() != sigs.size()) {
            RCLCPP_FATAL(get_logger(),
                         "Zone parameter arrays must all be the same length. "
                         "Got cx=%zu cy=%zu peaks=%zu sigmas=%zu",
                         cx.size(), cy.size(), peaks.size(), sigs.size());
            throw std::invalid_argument("Mismatched zone parameter array lengths");
        }

        _data.reserve(cx.size());
        for (size_t i = 0; i < cx.size(); ++i) {
            HeatZone temp{{cx[i], cy[i]}, peaks[i], sigs[i]};
            _data.push_back(temp);
            RCLCPP_INFO(get_logger(), "Zone %zu — center (%.2f, %.2f)  peak %.1f°C  sigma %.2fm", i, cx[i], cy[i], peaks[i], sigs[i]);
        }

        const double noise_stdev = get_parameter("noise_stdev").as_double();
        _distribution = std::normal_distribution<double>(0.0, noise_stdev);

        _map_frame = get_parameter("map_frame").as_string();
        _robot_frame = get_parameter("robot_frame").as_string();

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        _temp_pub = create_publisher<sensor_msgs::msg::Temperature>("/thermal_reading", rclcpp::QoS(10));

        _temp_sub = create_subscription<sensor_msgs::msg::Temperature>(
            "/thermal_reading",
            rclcpp::QoS(10),
            [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
                RCLCPP_DEBUG(get_logger(),
                             "Loopback — received %.2f°C  stamp: %u.%u",
                             msg->temperature,
                             msg->header.stamp.sec,
                             msg->header.stamp.nanosec);
            });

        const double rate = get_parameter("publish_rate").as_double();
        const auto period = std::chrono::duration<double>(1.0 / rate);

        _timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ThermalBroadcaster::TimerCallback, this));

        RCLCPP_INFO(get_logger(),
                    "ThermalBroadcaster ready — %zu zone(s), publishing at %.1f Hz",
                    _data.size(), rate);
    }

  private:
    void TimerCallback();

    std::vector<HeatZone> _data;

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr _temp_sub;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _temp_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    std::mt19937 _rng;
    std::normal_distribution<double> _distribution;

    std::string _map_frame;
    std::string _robot_frame;
};

void ThermalBroadcaster::TimerCallback() {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform =
            _tf_buffer->lookupTransform(
                _map_frame,
                _robot_frame,
                rclcpp::Time(0),
                rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF Lookup failed (%s) - skipping tick", e.what());
        return;
    }

    const double robot_x = transform.transform.translation.x;
    const double robot_y = transform.transform.translation.y;

    double total_temp = 0.0;
    for (const auto &zone : _data) {
        total_temp += zone.ContributionAt(robot_x, robot_y);
    }

    total_temp += _distribution(_rng);
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = now();
    msg.header.frame_id = _robot_frame;
    msg.temperature = total_temp;
    msg.variance = 0.0;

    _temp_pub->publish(msg);

    RCLCPP_DEBUG(get_logger(),
                 "Published %.2f C at robot position (%.2f %.2f)",
                 total_temp, robot_x, robot_y);
}
} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::ThermalBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
