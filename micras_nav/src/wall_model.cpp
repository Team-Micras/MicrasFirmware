/**
 * @file
 */

#include <cmath>
#include <cstdint>

#include "micras/nav/state.hpp"
#include "micras/nav/wall_model.hpp"

namespace micras::nav {
WallModel::WallModel(const Config& config) :
    maze_geometry{config.model.maze},
    sensors{config.model.wall_sensors},
    range_noise{config.model.noise.wall_range},
    range_noise_per_meter{config.model.noise.wall_range_per_meter},
    min_range{config.min_range},
    max_range{config.max_range},
    edge_margin{config.edge_margin},
    confidence{config.confidence} { }

PlaneCrossing WallModel::cross(const Pose& pose, uint8_t sensor, const RayHit& hit) const {
    const RobotModel::WallSensor& mounting = this->sensors.at(sensor);

    const Pose  origin = pose.compose({.position = mounting.position, .orientation = mounting.angle});
    const float direction_x = std::cos(origin.orientation);
    const float direction_y = std::sin(origin.orientation);

    const float across = hit.vertical ? direction_x : direction_y;
    const float along = hit.vertical ? direction_y : direction_x;
    const float slope = along / across;

    const float gap = hit.face - (hit.vertical ? origin.position.x : origin.position.y);
    const float start = hit.vertical ? origin.position.y : origin.position.x;

    const float lever_x = origin.position.x - pose.position.x;
    const float lever_y = origin.position.y - pose.position.y;
    const float swing = gap / (across * across);

    PlaneCrossing crossing{.offset = start + gap * slope - hit.base, .range = gap / across, .jacobian = {}};

    if (hit.vertical) {
        crossing.jacobian = {-slope, 1.0F, lever_x + lever_y * slope + swing};
    } else {
        crossing.jacobian = {1.0F, -slope, -lever_y - lever_x * slope - swing};
    }

    return crossing;
}

bool WallModel::is_side_looking(uint8_t sensor) const {
    return std::abs(std::sin(this->sensors.at(sensor).angle)) > side_looking_sine;
}

bool WallModel::is_footprint_clear(
    const RayHit& hit, uint8_t sensor, float position_deviation, float orientation_deviation
) const {
    if (not hit.valid) {
        return false;
    }

    const float spot = hit.range * std::tan(this->sensors.at(sensor).half_angle) / hit.cosine;
    const float uncertainty = this->confidence * (position_deviation + hit.range * orientation_deviation / hit.cosine);

    const float reach = spot + uncertainty + this->edge_margin;

    return hit.offset - reach >= hit.low_end and hit.offset + reach <= hit.high_end;
}

float WallModel::get_centered_range(uint8_t sensor) const {
    const RobotModel::WallSensor& mounting = this->sensors.at(sensor);

    const float face = this->maze_geometry.cell_size / 2.0F - this->maze_geometry.wall_thickness / 2.0F;
    const float sine = std::sin(mounting.angle);

    if (std::abs(sine) > 0.1F) {
        return (face - std::abs(mounting.position.y)) / std::abs(sine);
    }

    return (face - mounting.position.x) / std::cos(mounting.angle);
}

float WallModel::get_range_deviation(float range) const {
    return this->range_noise + this->range_noise_per_meter * range;
}
}  // namespace micras::nav
