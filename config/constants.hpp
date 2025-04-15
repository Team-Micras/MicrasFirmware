/**
 * @file
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <cstdint>
#include <numbers>

#include "micras/nav/action_queuer.hpp"
#include "micras/nav/follow_wall.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/nav/speed_controller.hpp"

namespace micras {
/*****************************************
 * Constants
 *****************************************/

constexpr uint8_t  maze_width{16};
constexpr uint8_t  maze_height{16};
constexpr float    cell_size{0.18};
constexpr uint32_t loop_time_us{1042};
constexpr float    wall_thickness{0.0126F};
constexpr float    start_offset{0.04F + wall_thickness / 2.0F};
constexpr float    exploration_speed{0.1F};

/*****************************************
 * Template Instantiations
 *****************************************/

namespace nav {
using Maze = TMaze<maze_width, maze_height>;
}  // namespace nav

/*****************************************
 * Configurations
 *****************************************/

const nav::ActionQueuer::Config action_queuer_config{
    .cell_size = cell_size,
    .start_offset = start_offset,
    .exploring =
        {
            .max_speed = exploration_speed,
            .max_acceleration = 1.0F,
            .max_deceleration = 1.0F,
            .curve_radius = cell_size / 2.0F,
            .max_centrifugal_acceleration = 1.0F,
        },
    .solving =
        {
            .max_speed = exploration_speed,
            .max_acceleration = 1.0F,
            .max_deceleration = 1.0F,
            .curve_radius = cell_size / 2.0F,
            .max_centrifugal_acceleration = 1.0F,
        },
};

const nav::FollowWall::Config follow_wall_config{
    .pid =
        {
            .kp = 0.5F,
            .ki = 0.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 1.0F,
            .max_integral = -1.0F,
        },
    .max_linear_speed = 0.1F,
};

const nav::Maze::Config maze_config{
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{
        {maze_width / 2, maze_height / 2},
        {(maze_width - 1) / 2, maze_height / 2},
        {maze_width / 2, (maze_height - 1) / 2},
        {(maze_width - 1) / 2, (maze_height - 1) / 2},
    }},
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 5.0F,
    .wheel_radius = 0.0112F,
    .initial_pose = {},
};

const nav::SpeedController::Config speed_controller_config{
    .linear_pid =
        {
            .kp = 0.5F,
            .ki = 0.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 1.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 0.5F,
            .ki = 0.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 1.0F,
            .max_integral = -1.0F,
        },
    .left_feed_forward =
        {
            .bias = 0.0F,
            .speed = 1.0F,
            .acceleration = 1.0F,
        },
    .right_feed_forward =
        {
            .bias = 0.0F,
            .speed = 1.0F,
            .acceleration = 1.0F,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
