/**
 * @file
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <cstdint>

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
constexpr float    max_linear_acceleration{10.0F};
constexpr float    max_linear_deceleration{15.0F};
constexpr float    max_angular_acceleration{300.0F};
constexpr float    crash_acceleration{35.0F};
constexpr float    fan_speed{100.0F};

constexpr core::WallSensorsIndex wall_sensors_index{
    .left_front = 0,
    .left = 1,
    .right = 2,
    .right_front = 3,
};

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
    .curve_safety_margin = 3.75F + 1.5F,
    .exploring =
        {
            .max_linear_speed = 0.4F,
            .max_linear_acceleration = max_linear_acceleration,
            .max_linear_deceleration = max_linear_deceleration,
            .max_centrifugal_acceleration = 2.78F,
            .max_angular_acceleration = max_angular_acceleration,
        },
    .solving =
        {
            .max_linear_speed = 3.0F,
            .max_linear_acceleration = max_linear_acceleration,
            .max_linear_deceleration = max_linear_deceleration,
            .max_centrifugal_acceleration = 5.0F,
            .max_angular_acceleration = max_angular_acceleration,
        }
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
    .post_threshold = 16.5F,
    .cell_size = cell_size,
    .post_clearance = 0.2F * cell_size,
};

const nav::Maze::Config maze_config{
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{
        {maze_width / 2, maze_height / 2},
        {(maze_width - 1) / 2, maze_height / 2},
        {maze_width / 2, (maze_height - 1) / 2},
        {(maze_width - 1) / 2, (maze_height - 1) / 2},
    }},
    .cost_margin = 1.2F,
    .graph_config =
        {
            .cell_size = cell_size,
            .cost_params = action_queuer_config.solving,
        },
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 5.0F,
    .wheel_radius = 0.0112F,
    .initial_pose = {{0.0F, 0.0F}, 0.0F},
};

const nav::SpeedController::Config speed_controller_config{
    .max_linear_acceleration = max_linear_acceleration,
    .max_angular_acceleration = max_angular_acceleration,
    .linear_pid =
        {
            .kp = 10.0F,
            .ki = 1.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 40.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 2.0F,
            .ki = 1.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 40.0F,
            .max_integral = -1.0F,
        },
    .left_feed_forward =
        {
            .linear_speed = 12.706F,
            .linear_acceleration = 2.796F,
            .angular_speed = -0.971F,
            .angular_acceleration = -0.0258F,
        },
    .right_feed_forward =
        {
            .linear_speed = 13.319F,
            .linear_acceleration = 2.878F,
            .angular_speed = 0.901F,
            .angular_acceleration = -0.0244F,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
