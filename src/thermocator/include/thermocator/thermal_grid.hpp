#pragma once
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <optional>
#include <vector>

namespace thermocator {

class ThermalGrid final {

  public:
    ThermalGrid() = default;

    constexpr bool IsInitialized() noexcept { return _initialized; }

    void Initialize(const nav_msgs::msg::OccupancyGrid &map);
    void Update(double world_x, double world_y, double temperature);
    void Resize(const nav_msgs::msg::MapMetaData &new_info);

    std::vector<int8_t> ToOccupancyData(double cold_thresh, double hot_thresh, float min_conf = 0.0f) const;

    std::optional<double> GetTemperatureAt(double world_x, double world_y) const;
    std::optional<float> GetConfidenceAt(double world_x, double world_y) const;
    bool IsVisited(double world_x, double world_y);
    const nav_msgs::msg::MapMetaData &getInfo() const { return _info; }

    bool HasHeatData(double min_temperature) const;
    std::pair<double, double> GetHottestCell() const;

  private:
    std::optional<std::pair<int, int>> WorldToGrid(double x, double y) const;
    void ApplyEMA(std::size_t index, double temperature, float gaussian_weight);

    bool _initialized{false};

    double _origin_x{0.0};
    double _origin_y{0.0};
    float _resolution{0.05f};
    uint32_t _width{0u};
    uint32_t _height{0u};

    float _alpha{0.2f};
    float _sigma{0.15f};

    nav_msgs::msg::MapMetaData _info;

    std::vector<double> _temperature;
    std::vector<float> _confidence;
    std::vector<bool> _visited;
};
} // namespace thermocator
