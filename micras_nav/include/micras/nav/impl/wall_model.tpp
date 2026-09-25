/**
 * @file
 */

#ifndef MICRAS_NAV_WALL_MODEL_TPP
#define MICRAS_NAV_WALL_MODEL_TPP

#include <cmath>
#include <cstdint>
#include <limits>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
RayHit WallModel::cast(const Pose& pose, uint8_t sensor, const TMaze<width, height>& maze) const {
    const RobotModel::WallSensor& mounting = this->sensors.at(sensor);

    const Pose  origin = pose.compose({.position = mounting.position, .orientation = mounting.angle});
    const float direction_x = std::cos(origin.orientation);
    const float direction_y = std::sin(origin.orientation);

    const float cell_size = this->maze_geometry.cell_size;
    const float half_wall = this->maze_geometry.wall_thickness / 2.0F;

    auto column = static_cast<int16_t>(std::floor(origin.position.x / cell_size));
    auto row = static_cast<int16_t>(std::floor(origin.position.y / cell_size));

    bool grazed_post = false;

    for (uint8_t crossing = 0; crossing < 4; crossing++) {
        if (column < 0 or row < 0 or column >= width or row >= height) {
            break;
        }

        const float line_x = static_cast<float>(direction_x > 0.0F ? column + 1 : column) * cell_size;
        const float line_y = static_cast<float>(direction_y > 0.0F ? row + 1 : row) * cell_size;

        const float distance_x =
            direction_x != 0.0F ? (line_x - origin.position.x) / direction_x : std::numeric_limits<float>::infinity();
        const float distance_y =
            direction_y != 0.0F ? (line_y - origin.position.y) / direction_y : std::numeric_limits<float>::infinity();

        const bool  vertical = distance_x < distance_y;
        const float distance = vertical ? distance_x : distance_y;
        const float across = vertical ? direction_x : direction_y;
        const float along = vertical ? direction_y : direction_x;
        const float range = distance - half_wall / std::abs(across);

        if (range > this->max_range) {
            break;
        }

        const GridPose wall{
            .position = {.x = static_cast<uint8_t>(column), .y = static_cast<uint8_t>(row)},
            .orientation = vertical ? (direction_x > 0.0F ? Side::RIGHT : Side::LEFT) :
                                      (direction_y > 0.0F ? Side::UP : Side::DOWN),
        };

        const float base = static_cast<float>(vertical ? row : column) * cell_size;
        const float offset =
            (vertical ? origin.position.y + distance * direction_y : origin.position.x + distance * direction_x) - base;

        const WallState state = maze.get_wall(wall);
        const bool      at_post = offset < half_wall or offset > cell_size - half_wall;

        if (state != WallState::WALL and (at_post or grazed_post)) {
            if (state == WallState::UNKNOWN or grazed_post) {
                break;
            }

            grazed_post = true;
        }

        if (state == WallState::NO_WALL) {
            column = static_cast<int16_t>(column + (vertical ? (direction_x > 0.0F ? 1 : -1) : 0));
            row = static_cast<int16_t>(row + (vertical ? 0 : (direction_y > 0.0F ? 1 : -1)));
            continue;
        }

        if (range < this->min_range) {
            break;
        }

        const float lever = vertical ? origin.position.y - pose.position.y : -(origin.position.x - pose.position.x);
        const float gap = distance * across;
        const float swing =
            (vertical ? 1.0F : -1.0F) * along * (gap - std::copysign(half_wall, across)) / (across * across);

        RayHit hit{
            .valid = true,
            .wall = wall,
            .state = state,
            .range = range,
            .offset = offset,
            .cosine = std::abs(across),
            .jacobian = {},
            .vertical = vertical,
            .face = (vertical ? line_x : line_y) - std::copysign(half_wall, across),
            .base = base,
            .low_end = half_wall,
            .high_end = cell_size - half_wall,
        };

        hit.jacobian.at(vertical ? 0 : 1) = -1.0F / across;
        hit.jacobian.at(2) = lever / across + swing;

        if (state == WallState::WALL) {
            hit.low_end = -half_wall;
            hit.high_end = cell_size + half_wall;

            GridPose lower = wall;
            GridPose higher = wall;

            for (uint8_t i = 0; i < 2; i++) {
                lower.position = lower.position + (vertical ? Side::DOWN : Side::LEFT);
                higher.position = higher.position + (vertical ? Side::UP : Side::RIGHT);

                if (hit.low_end == -half_wall - static_cast<float>(i) * cell_size and
                    TMaze<width, height>::contains(lower.position) and maze.get_wall(lower) == WallState::WALL) {
                    hit.low_end -= cell_size;
                }

                if (hit.high_end == cell_size + half_wall + static_cast<float>(i) * cell_size and
                    TMaze<width, height>::contains(higher.position) and maze.get_wall(higher) == WallState::WALL) {
                    hit.high_end += cell_size;
                }
            }
        }

        return hit;
    }

    return {};
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_WALL_MODEL_TPP
