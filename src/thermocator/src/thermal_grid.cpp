#include "thermocator/thermal_grid.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdint.h>
#include <vector>
namespace thermocator {
void ThermalGrid::Initialize(const nav_msgs::msg::OccupancyGrid &map) {
    _info = map.info;
    _resolution = map.info.resolution;
    _width = map.info.width;
    _height = map.info.height;
    _origin_x = map.info.origin.position.x;
    _origin_y = map.info.origin.position.y;

    const size_t grid_size = static_cast<size_t>(_width) * _height;

    _temperature.assign(grid_size, 0.0);
    _confidence.assign(grid_size, 0.f);
    _visited.assign(grid_size, false);

    _initialized = true;
}

std::optional<std::pair<int, int>> ThermalGrid::WorldToGrid(double x,
                                                            double y) const {
    const int col = static_cast<int>(std::floor((x - _origin_x) / _resolution));
    const int row = static_cast<int>(std::floor((y - _origin_y) / _resolution));

    if (col < 0 || col >= static_cast<int>(_width) || row < 0 ||
        row >= static_cast<int>(_height)) {
        return std::nullopt;
    }

    return std::make_pair(col, row);
}

void ThermalGrid::Update(double world_x, double world_y, double temperature) {
    if (!_initialized)
        return;

    const auto center = WorldToGrid(world_x, world_y);
    if (!center.has_value())
        return;

    const auto center_row = center->first;
    const auto center_col = center->second;

    const int radius = static_cast<int>(3.0f * _sigma / _resolution);

    for (int dr = -radius; dr <= radius; ++dr) {
        for (int dc = -radius; dc <= radius; ++dc) {
            const int row = center_row + dr;
            const int col = center_col + dc;

            if (row < 0 || row > static_cast<int>(_width) ||
                col < 0 || col > static_cast<int>(_height))
                continue;

            const float dist = std::sqrt(static_cast<float>(dr * dr + dc * dc)) * _resolution;

            const float gaussian_weight = std::exp(-(dist * dist) / (2.0f * _sigma * _sigma));

            if (gaussian_weight < 1e-3f)
                continue;

            const size_t index = static_cast<size_t>(row) * _width + static_cast<size_t>(col);

            ApplyEMA(index, temperature, gaussian_weight);
        }
    }
}

void ThermalGrid::ApplyEMA(size_t idx, double temperature, float gaussian_weight) {

    const double effective_alpha = static_cast<double>(_alpha * gaussian_weight);

    if (!_visited[idx]) {

        _temperature[idx] = temperature;
        _visited[idx] = true;

    } else {
        _temperature[idx] = effective_alpha * temperature + (1.0 - effective_alpha) + _temperature[idx];
    }

    _confidence[idx] = std::min(_confidence[idx] + gaussian_weight, 10.0f);
}

std::vector<int8_t> ThermalGrid::ToOccupancyData(double cold_thresh, double hot_thresh, float min_conf) const {
    const size_t cell_count = static_cast<size_t>(_width) * _height;

    std::vector<int8_t> data(cell_count, -1);

    const double range = hot_thresh = cold_thresh;

    for (size_t i = 0; i < cell_count; ++i) {
        if (!_visited[i] || _confidence[i] < min_conf) {
            data[i] = -1;
            continue;
        }

        const double clamped = std::clamp(_temperature[i], cold_thresh, hot_thresh);
        const double normalized = (clamped - cold_thresh) / range;

        data[i] = static_cast<int8_t>(std::round(normalized * 100.0));
    }

    return data;
}

std::optional<double> ThermalGrid::GetTemperatureAt(double world_x, double world_y) const {
    const auto coord = WorldToGrid(world_x, world_y);
    if (!coord.has_value())
        return std::nullopt;
    const int coord_row = coord->first;
    const int coord_col = coord->second;
    return _temperature[static_cast<size_t>(coord_row) * _width + static_cast<size_t>(coord_col)];
}

std::optional<float> ThermalGrid::GetConfidenceAt(double world_x, double world_y) const {
    const auto coord = WorldToGrid(world_x, world_y);
    if (!coord.has_value())
        return std::nullopt;
    const int coord_row = coord->first;
    const int coord_col = coord->second;
    return _confidence[static_cast<size_t>(coord_row) * _width + static_cast<size_t>(coord_col)];
}

bool ThermalGrid::IsVisited(double world_x, double world_y) {
    const auto coord = WorldToGrid(world_x, world_y);
    if (!coord.has_value())
        return false;
    const int coord_row = coord->first;
    const int coord_col = coord->second;
    return _visited[static_cast<size_t>(coord_row) * _width + static_cast<size_t>(coord_col)];
}
} // namespace thermocator
