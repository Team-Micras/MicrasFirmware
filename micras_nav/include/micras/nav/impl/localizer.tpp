/**
 * @file
 */

#ifndef MICRAS_NAV_LOCALIZER_TPP
#define MICRAS_NAV_LOCALIZER_TPP

#include <cmath>
#include <cstdint>
#include <optional>

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
            tracker.locked = false;
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

        if (not reading.valid or not hit.valid or hit.state != WallState::WALL or hit.range > this->config.max_range or
            not wall_model.is_footprint_clear(
                hit, sensor, this->get_position_deviation(), this->get_orientation_deviation()
            )) {
            continue;
        }

        const float deviation = wall_model.get_range_deviation(hit.range);

        this->update(
            wall_model.get_range(reading.distance, sensor, hit) - hit.range,
            {hit.jacobian.at(0), hit.jacobian.at(1), hit.jacobian.at(2), 0.0F},
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

    const auto tolerance = [this](float range) {
        return this->config.range_tolerance + this->config.relative_range_tolerance * range;
    };

    if (this->state.velocity.linear <= this->config.edge_speed) {
        tracker.locked = false;
        return;
    }

    if (tracker.locked) {
        const PlaneCrossing crossing = wall_model.cross(sampled, sensor, tracker.hit);
        const bool reading_on = reading.valid and reading.distance < crossing.range + tolerance(crossing.range);

        if (crossing.range > this->config.edge_range) {
            tracker.locked = false;
        } else if (reading_on != tracker.reading_on) {
            tracker.reading_on = reading_on;

            const float heading = this->state.pose.orientation;
            const float travel =
                this->state.velocity.linear * (tracker.hit.vertical ? std::sin(heading) : std::cos(heading));
            const float                face = this->config.model.maze.wall_thickness * std::abs(crossing.slope);
            const std::optional<float> edge = this->find_edge(maze, tracker.hit.wall, travel > 0.0F, reading_on, face);

            if (edge.has_value() and std::abs(*edge - crossing.offset) <= this->config.edge_window and
                this->update(
                    *edge - crossing.offset,
                    {crossing.jacobian.at(0), crossing.jacobian.at(1), crossing.jacobian.at(2), 0.0F},
                    this->config.edge_deviation * this->config.edge_deviation, this->config.gate,
                    this->config.max_edge_correction
                )) {
                this->status.edges++;
            }
        }
    }

    const bool on_wall = hit.valid and hit.state == WallState::WALL and hit.range < this->config.edge_range and
                         std::abs(reading.distance - hit.range) < tolerance(hit.range) and reading.valid;

    if (on_wall) {
        tracker.locked = true;
        tracker.reading_on = true;
        tracker.hit = hit;
    }
}

template <uint8_t width, uint8_t height>
std::optional<float> Localizer::find_edge(
    const TMaze<width, height>& maze, const GridPose& wall, bool upwards, bool start, float face
) const {
    const bool  vertical = wall.orientation == Side::LEFT or wall.orientation == Side::RIGHT;
    const float cell_size = this->config.model.maze.cell_size;
    const float half_wall = this->config.model.maze.wall_thickness / 2.0F;
    const float sign = upwards ? 1.0F : -1.0F;

    Side step = upwards ? Side::UP : Side::DOWN;

    if (not vertical) {
        step = upwards ? Side::RIGHT : Side::LEFT;
    }

    GridPose segment = wall;
    bool     in_gap = false;

    for (uint8_t shift = 1; shift <= edge_search_cells; shift++) {
        segment.position = segment.position + step;

        const WallState state =
            TMaze<width, height>::contains(segment.position) ? maze.get_wall(segment) : WallState::UNKNOWN;

        if (state == WallState::UNKNOWN) {
            return std::nullopt;
        }

        const bool open = state == WallState::NO_WALL;
        in_gap = in_gap or open;

        if (start ? (in_gap and not open) : open) {
            const auto  cells = static_cast<float>(shift);
            const float boundary = (upwards ? cells : 1.0F - cells) * cell_size;

            return start ? boundary - sign * (half_wall + face) : boundary + sign * half_wall;
        }
    }

    return std::nullopt;
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_LOCALIZER_TPP
