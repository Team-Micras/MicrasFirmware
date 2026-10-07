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
 * @brief The 4 by 4 maze the robot is tested in at home.
 *
 * @note The start is the cell in the bottom right corner, open to the north, and the goal is the
 * single cell (2, 3) of the top row. The walls measure 15 to 15.2 mm. The pitch of the cells is the
 * classic one, until the maze is measured post to post.
 *
 * @note The walls are white melamine boards, which are semi-gloss. Square on to a board, a diagonal
 * sensor 97.6 mm from it read 66 mm with its calibration at 45 degrees: a wall sends back 2.5 times
 * more light square on than at 45 degrees, where a matte one sends back 1.41 times more. That is a
 * Minnaert exponent of 1.83. Pulling the robot back from a board at 45 degrees, with one diagonal
 * square on to it and the front sensor on that side at 45 degrees, gives 1.6 to 1.8, as the angle
 * the robot was set at by hand allows.
 */
constexpr nav::RobotModel::Maze maze_geometry{
    .cell_size = 0.18F,
    .wall_thickness = 0.0151F,
    .wall_minnaert = 1.83F,
};

constexpr uint8_t maze_width{4};
constexpr uint8_t maze_height{4};

constexpr nav::GridPose maze_start{.position = {.x = 3, .y = 0}, .orientation = nav::Side::UP};

constexpr std::array<nav::GridPoint, 1> maze_goal{{
    {.x = 2, .y = 3},
}};
}  // namespace micras

#endif  // MICRAS_MAZE_CONFIG_HPP
