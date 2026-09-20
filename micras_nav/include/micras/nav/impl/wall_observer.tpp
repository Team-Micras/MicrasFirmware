/**
 * @file
 */

#ifndef MICRAS_NAV_WALL_OBSERVER_TPP
#define MICRAS_NAV_WALL_OBSERVER_TPP

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/wall_model.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TWallObserver<width, height>::TWallObserver(const Config& config) : config{config} { }

template <uint8_t width, uint8_t height>
void TWallObserver<width, height>::reset() {
    this->votes.fill(0);
}

template <uint8_t width, uint8_t height>
bool TWallObserver<width, height>::update(
    const Measurements& measurements, const Localizer& localizer, const WallModel& wall_model,
    TMaze<width, height>& maze
) {
    const State& state = localizer.get_state();

    if (std::abs(state.velocity.angular) > this->config.max_angular_speed) {
        return false;
    }

    const Pose sampled = state.pose.compose({
        .position = {.x = -state.velocity.linear * this->config.range_delay, .y = 0.0F},
        .orientation = -state.velocity.angular * this->config.range_delay,
    });

    bool changed = false;

    for (uint8_t sensor = 0; sensor < number_of_wall_sensors; sensor++) {
        const WallReading& reading = measurements.walls.at(sensor);

        if (not reading.is_new) {
            continue;
        }

        const RayHit hit = wall_model.cast(sampled, sensor, maze);

        if (not hit.valid or hit.state != WallState::UNKNOWN or
            not wall_model.is_footprint_clear(
                hit, sensor, localizer.get_position_deviation(), localizer.get_orientation_deviation()
            )) {
            continue;
        }

        const float tolerance = this->config.tolerance + this->config.relative_tolerance * hit.range;
        int8_t&     count = this->votes.at(get_index(hit.wall));

        if (reading.valid and std::abs(reading.distance - hit.range) <= tolerance) {
            count = static_cast<int8_t>(std::min<int16_t>(count + 1, this->config.votes_to_decide));
        } else if (
            (not reading.valid or reading.distance > hit.range + tolerance) and
            hit.range <= this->config.detection_range
        ) {
            count = static_cast<int8_t>(std::max<int16_t>(count - 1, -this->config.votes_to_decide));
        } else {
            continue;
        }

        if (std::abs(count) >= this->config.votes_to_decide) {
            changed = maze.set_wall(hit.wall, count > 0) or changed;
        }
    }

    return changed;
}

template <uint8_t width, uint8_t height>
constexpr uint16_t TWallObserver<width, height>::get_index(const GridPose& wall) {
    const uint8_t x = wall.position.x;
    const uint8_t y = wall.position.y;

    switch (wall.orientation) {
        case Side::LEFT:
            return static_cast<uint16_t>(x * height + y);
        case Side::RIGHT:
            return static_cast<uint16_t>((x + 1) * height + y);
        case Side::DOWN:
            return static_cast<uint16_t>((width + 1) * height + x * (height + 1) + y);
        case Side::UP:
        default:
            return static_cast<uint16_t>((width + 1) * height + x * (height + 1) + y + 1);
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_WALL_OBSERVER_TPP
