#include "thermocator/map_builder.hpp"
#include "thermocator/thermal_grid.hpp"
#include "thermocator/world_init.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {

    using namespace thermocator;

    rclcpp::init(argc, argv);
    auto grid = std::make_shared<ThermalGrid>();

    {
        auto listener = std::make_shared<WorldInitializer>(grid);
        rclcpp::spin_until_future_complete(listener, listener->getFuture());
        RCLCPP_INFO(rclcpp::get_logger("main"), "World Initialized - Thermal Grid ready");
    }

    auto builder = std::make_shared<ThermalMapBuilder>(grid);
    rclcpp::spin(builder);
    rclcpp::shutdown();
    return 0;
}
