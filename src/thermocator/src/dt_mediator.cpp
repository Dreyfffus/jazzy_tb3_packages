#include <chrono>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace thermocator {

class DtMediator : public rclcpp::Node {
  public:
    DtMediator() : Node("dt_mediator") {
        auto qos = rclcpp::QoS(10);
        auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/dt/odom", qos);
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/dt/scan", qos);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dt/map", latched_qos);
        thermal_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dt/thermal_map", latched_qos);
        thermal_reading_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/dt/thermal_reading", qos);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", qos);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                logStream("/odom", "/dt/odom");
                odom_pub_->publish(*msg);
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                logStream("/scan", "/dt/scan");
                scan_pub_->publish(*msg);
            });

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", latched_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                logStream("/map", "/dt/map");
                map_pub_->publish(*msg);
            });

        thermal_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/thermal_map", latched_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                logStream("/thermal_map", "/dt/thermal_map");
                thermal_map_pub_->publish(*msg);
            });

        thermal_reading_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
            "/thermal_reading", qos,
            [this](sensor_msgs::msg::Temperature::SharedPtr msg) {
                logStream("/thermal_reading", "/dt/thermal_reading");
                thermal_reading_pub_->publish(*msg);
            });

        dt_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/dt/cmd_vel", qos,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                logStream("/dt/cmd_vel", "/cmd_vel");
                geometry_msgs::msg::TwistStamped stamped_msg;
                stamped_msg.header.stamp = now();
                stamped_msg.header.frame_id = "base_link";
                stamped_msg.twist = *msg;
                cmd_vel_pub_->publish(stamped_msg);
            });

        RCLCPP_INFO(
            get_logger(),
            "DT mediator ready: /odom,/scan,/map,/thermal_map,/thermal_reading -> /dt/* and /dt/cmd_vel -> /cmd_vel");
    }

  private:
    void logStream(const std::string &input, const std::string &output) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Active DT stream: %s -> %s", input.c_str(), output.c_str());
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr thermal_reading_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr thermal_reading_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dt_cmd_vel_sub_;
};

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::DtMediator>());
    rclcpp::shutdown();
    return 0;
}
