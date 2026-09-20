/**
 * @file
 */

#ifndef MICRAS_NAV_LOCALIZER_TPP
#define MICRAS_NAV_LOCALIZER_TPP

#include <cmath>
#include <cstdint>

#include "micras/nav/maze.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/wall_model.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
void Localizer::correct(
    const Measurements& measurements, const WallModel& wall_model, const TMaze<width, height>& maze
) {
    if (std::abs(this->state.velocity.angular) > this->config.max_angular_speed) {
        for (EdgeTracker& tracker : this->edge_trackers) {
            tracker.tracking = false;
        }

        return;
    }

    const float gate = this->is_stationary() ? this->config.stationary_gate : this->config.gate;

    for (uint8_t sensor = 0; sensor < number_of_wall_sensors; sensor++) {
        const WallReading& reading = measurements.walls.at(sensor);

        if (not reading.is_new) {
            continue;
        }

        const Pose sampled = this->state.pose.compose({
            .position = {.x = -this->state.velocity.linear * this->config.range_delay, .y = 0.0F},
            .orientation = -this->state.velocity.angular * this->config.range_delay,
        });

        const RayHit hit = wall_model.cast(sampled, sensor, maze);

        if (this->config.use_edges and wall_model.is_side_looking(sensor)) {
            this->track_edge(sensor, reading, sampled, hit, wall_model, maze);
        }

        if (not reading.valid or not hit.valid or hit.state != WallState::WALL or
            not wall_model.is_footprint_clear(
                hit, sensor, this->get_position_deviation(), this->get_orientation_deviation()
            )) {
            continue;
        }

        const float deviation = wall_model.get_range_deviation(hit.range);

        this->update(
            reading.distance - hit.range, {hit.jacobian.at(0), hit.jacobian.at(1), hit.jacobian.at(2), 0.0F},
            deviation * deviation * this->config.range_correlation, gate, this->config.max_position_correction
        );
    }
}

template <uint8_t width, uint8_t height>
void Localizer::track_edge(
    uint8_t sensor, const WallReading& reading, const Pose& sampled, const RayHit& hit, const WallModel& wall_model,
    const TMaze<width, height>& maze
) {
    EdgeTracker& tracker = this->edge_trackers.at(sensor);

    const auto is_on = [this, &reading](float range) {
        return reading.valid and std::abs(reading.distance - range) <
                                     this->config.range_tolerance + this->config.relative_range_tolerance * range;
    };

    const bool on_wall =
        hit.valid and hit.state == WallState::WALL and hit.range < this->config.edge_range and is_on(hit.range);
    const bool was_tracking = tracker.tracking;
    const bool was_on_wall = tracker.on_wall;

    const RayHit previous = tracker.hit;

    tracker.tracking = this->state.velocity.linear > this->config.edge_speed;
    tracker.on_wall = on_wall;

    if (on_wall) {
        tracker.hit = hit;
    }

    if (not was_tracking or not tracker.tracking or was_on_wall == on_wall) {
        return;
    }

    const RayHit& wall = on_wall ? hit : previous;

    const PlaneCrossing crossing = wall_model.cross(sampled, sensor, wall);

    if (not on_wall and (is_on(crossing.range) or (reading.valid and reading.distance < crossing.range))) {
        return;
    }

    const float heading = this->state.pose.orientation;
    const float travel = this->state.velocity.linear * (wall.vertical ? std::sin(heading) : std::cos(heading));
    const bool  leaving_upwards = (travel > 0.0F) == (not on_wall);

    const Side towards =
        wall.vertical ? (leaving_upwards ? Side::UP : Side::DOWN) : (leaving_upwards ? Side::RIGHT : Side::LEFT);
    const GridPose beyond{.position = wall.wall.position + towards, .orientation = wall.wall.orientation};

    if (maze.get_wall(beyond) == WallState::WALL) {
        return;
    }

    const float half_wall = this->config.model.maze.wall_thickness / 2.0F;
    const float edge = leaving_upwards ? this->config.model.maze.cell_size + half_wall : -half_wall;
    const float innovation = edge - crossing.offset;

    if (std::abs(innovation) > this->config.edge_window) {
        return;
    }

    if (this->update(
            innovation, {crossing.jacobian.at(0), crossing.jacobian.at(1), crossing.jacobian.at(2), 0.0F},
            this->config.edge_deviation * this->config.edge_deviation, this->config.gate,
            this->config.max_edge_correction
        )) {
        this->status.edges++;
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_LOCALIZER_TPP
