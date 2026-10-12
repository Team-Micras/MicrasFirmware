/**
 * @file
 */

#ifndef MICRAS_MAZE_CONFIG_HPP
#define MICRAS_MAZE_CONFIG_HPP

#include <array>
#include <cstdint>
#include <string_view>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/robot_model.hpp"

namespace micras {
/**
 * @brief The competition maze the robot runs in: 16 by 16 cells of 180 mm, the start in a corner and
 * the goal the four cells of the center.
 *
 * @note The walls are 12 mm thick, by the rules.
 *
 * @note The walls are glossy. Turned by hand in front of a wall, a diagonal sensor read 1.9 times more
 * square on to it than at 45 degrees, at the same distance: a Minnaert exponent of 1.4, which is
 * what the robot searched this maze with without a wrong wall.
 */
constexpr nav::RobotModel::Maze maze_geometry{
    .cell_size = 0.18F,
    .wall_thickness = 0.012F,
    .wall_minnaert = 1.4F,
};

/**
 * @brief What each wall sensor reads at its reference distance from these walls, over what it reads
 * from the walls its reference readings were calibrated against (the home maze's semi-gloss
 * boards), in the order of wall_sensors_index.
 *
 * @note The board with AS5047U encoders: its fronts are fitted to a pull-back from touching an
 * inner wall (2.966 and 2.975, within 2 mm rms over 105 to 260 mm), and its diagonals are the ones
 * it searched this maze with (0.3159 and 0.2225).
 *
 * @note The board with AS5047D encoders: its fronts are fitted to two pull-backs from touching an
 * inner wall (3.322 and 1.338), and its diagonals are in its flash.
 */
#ifdef MICRAS_ROTARY_SENSOR_AS5047D
constexpr std::array<float, nav::number_of_wall_sensors> wall_reference_scale{1.2896F, 0.9781F, 0.9781F, 0.6703F};
#else
constexpr std::array<float, nav::number_of_wall_sensors> wall_reference_scale{1.1514F, 1.085F, 0.942F, 1.4905F};
#endif

/**
 * @brief Drawing of the walls of the maze, when they are known before the run: a row of walls and
 * posts above each row of cells, from the top, with "---" for a wall between two posts and "|" for
 * one between two cells. Empty when the maze has to be searched.
 */
constexpr std::string_view known_maze{};

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
