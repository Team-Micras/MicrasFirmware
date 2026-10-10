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
 *
 * @note The walls are taken as matte, painted wood, until a competition maze is measured.
 */
constexpr nav::RobotModel::Maze maze_geometry{
    .cell_size = 0.18F,
    .wall_thickness = 0.0126F,
    .wall_minnaert = 1.0F,
};

/**
 * @brief What each wall sensor reads at its reference distance from these walls, over what it reads
 * from the walls its reference readings were calibrated against (the home maze's semi-gloss
 * boards), in the order of wall_sensors_index.
 *
 * @note Matte walls send a sensor back about as much light square on as the semi-gloss boards, and
 * 1.7 times as much at 45 degrees, where the diagonal sensors calibrate: those are the simulated
 * sensors' readings in these walls over their readings in the home maze's, the only source until a
 * competition maze is measured. A calibration in the maze replaces the references, and this with
 * them.
 */
constexpr std::array<float, nav::number_of_wall_sensors> wall_reference_scale{0.9787F, 1.6999F, 1.6995F, 0.9787F};

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
