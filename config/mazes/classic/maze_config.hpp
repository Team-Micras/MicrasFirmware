/**
 * @file
 */

#ifndef MICRAS_MAZE_CONFIG_HPP
#define MICRAS_MAZE_CONFIG_HPP

#include <array>
#include <cstdint>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/robot_model.hpp"

namespace micras {
/**
 * @brief The classic maze of the competitions: 16 by 16 cells of 180 mm, the start in a corner and
 * the goal the four cells of the center.
 *
 * @note The build selects the maze with MICRAS_MAZE, which puts the directory of that maze on the
 * include path, as the board directory is. This is the one the simulation and its baselines run.
 *
 * @note The walls are 12 mm thick by the rules. 12.6 mm is what the firmware has always assumed, from
 * a measurement of a competition maze, whose walls are thicker than the rules once painted.
 */
constexpr nav::RobotModel::Maze maze_geometry{
    .cell_size = 0.18F,
    .wall_thickness = 0.0126F,
};

constexpr uint8_t maze_width{16};
constexpr uint8_t maze_height{16};

constexpr nav::GridPose maze_start{.position = {.x = 0, .y = 0}, .orientation = nav::Side::UP};

constexpr std::array<nav::GridPoint, 4> maze_goal{{
    {.x = maze_width / 2, .y = maze_height / 2},
    {.x = (maze_width - 1) / 2, .y = maze_height / 2},
    {.x = maze_width / 2, .y = (maze_height - 1) / 2},
    {.x = (maze_width - 1) / 2, .y = (maze_height - 1) / 2},
}};
}  // namespace micras

#endif  // MICRAS_MAZE_CONFIG_HPP
