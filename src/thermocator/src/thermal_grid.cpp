#include "thermocator/thermal_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

bool ThermalGrid::HasHeatData(double min_temperature) const {
    if (!_initialized)
        return false;
    for (size_t i = 0; i < _visited.size(); ++i) {
        if (_visited[i] && _temperature[i] >= min_temperature)
            return true;
    }
    return false;
}

std::pair<double, double> ThermalGrid::GetHottestCell() const {
    double max_temp = std::numeric_limits<double>::lowest();
    int best_col = 0, best_row = 0;

    for (uint32_t row = 0; row < _height; ++row) {
        for (uint32_t col = 0; col < _width; ++col) {
            const size_t idx = static_cast<size_t>(row) * _width + col;
            if (_visited[idx] && _temperature[idx] > max_temp) {
                max_temp = _temperature[idx];
                best_col = static_cast<int>(col);
                best_row = static_cast<int>(row);
            }
        }
    }

    return {
        _origin_x + (best_col + 0.5) * _resolution,
        _origin_y + (best_row + 0.5) * _resolution};
}

void ThermalGrid::Resize(const nav_msgs::msg::MapMetaData &new_info) {
    const uint32_t new_width = new_info.width;
    const uint32_t new_height = new_info.height;
    const size_t new_size = static_cast<size_t>(new_width) * new_height;

    std::vector<double> new_temperature(new_size, 0.0);
    std::vector<float> new_confidence(new_size, 0.0f);
    std::vector<bool> new_visited(new_size, false);

    const int col_offset = static_cast<int>(std::round(
        (_origin_x - new_info.origin.position.x) / _resolution));
    const int row_offset = static_cast<int>(std::round(
        (_origin_y - new_info.origin.position.y) / _resolution));

    for (uint32_t old_row = 0; old_row < _height; ++old_row) {
        for (uint32_t old_col = 0; old_col < _width; ++old_col) {
            const size_t old_idx = static_cast<size_t>(old_row) * _width + old_col;

            if (!_visited[old_idx])
                continue;

            const int new_col = static_cast<int>(old_col) + col_offset;
            const int new_row = static_cast<int>(old_row) + row_offset;

            if (new_col < 0 || new_col >= static_cast<int>(new_width) ||
                new_row < 0 || new_row >= static_cast<int>(new_height))
                continue;

            const size_t new_idx = static_cast<size_t>(new_row) * new_width + new_col;

            new_temperature[new_idx] = _temperature[old_idx];
            new_confidence[new_idx] = _confidence[old_idx];
            new_visited[new_idx] = true;
        }
    }

    _temperature = std::move(new_temperature);
    _confidence = std::move(new_confidence);
    _visited = std::move(new_visited);

    _width = new_width;
    _height = new_height;
    _origin_x = new_info.origin.position.x;
    _origin_y = new_info.origin.position.y;
    _info = new_info;
}

void ThermalGrid::Update(double world_x, double world_y, double temperature) {
    if (!_initialized)
        return;

    const auto center = WorldToGrid(world_x, world_y);
    if (!center.has_value())
        return;

    const auto center_row = center->second;
    const auto center_col = center->first;

    const int radius = static_cast<int>(3.0f * _sigma / _resolution);

    for (int dr = -radius; dr <= radius; ++dr) {
        for (int dc = -radius; dc <= radius; ++dc) {
            const int row = center_row + dr;
            const int col = center_col + dc;

            if (row < 0 || row > static_cast<int>(_height) ||
                col < 0 || col > static_cast<int>(_width))
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
        _temperature[idx] = effective_alpha * temperature + (1.0 - effective_alpha) * _temperature[idx];
    }

    _confidence[idx] = std::min(_confidence[idx] + gaussian_weight, 10.0f);
}

std::vector<int8_t> ThermalGrid::ToOccupancyData(double cold_thresh, double hot_thresh, float min_conf) const {
    const size_t cell_count = static_cast<size_t>(_width) * _height;

    std::vector<int8_t> data(cell_count, -1);

    const double range = hot_thresh - cold_thresh;

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
