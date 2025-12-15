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

constexpr uint8_t  maze_width{4};
constexpr uint8_t  maze_height{4};
constexpr float    cell_size{0.18};
constexpr uint32_t loop_time_us{1042};
constexpr float    wall_thickness{0.0152F};
constexpr float    start_offset{0.0285F + wall_thickness / 2.0F};
constexpr float    max_linear_acceleration{9.0F};
constexpr float    max_linear_deceleration{9.0F};
constexpr float    max_angular_acceleration{800.0F};
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
    .curve_safety_margin = 0.0375F + 0.015F,
    .exploring =
        {
            .max_linear_speed = 0.3F,
            .max_linear_acceleration = max_linear_acceleration,
            .max_linear_deceleration = max_linear_deceleration,
            .max_centrifugal_acceleration = 2.0F,
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
    .max_angular_acceleration = max_angular_acceleration,
    .cell_size = cell_size,
    .post_threshold = 6.5F,
    .post_reference = 0.066F + wall_thickness / 2.0F,
    .post_clearance = 0.025F,
};

const nav::Maze::Config maze_config{
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{{1, 0}}},
    .cost_margin = 1.2F,
    .action_queuer_config = action_queuer_config,
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 5.0F,
    .wheel_radius = 0.0112F,
    .initial_pose = {{cell_size / 2.0F, start_offset}, std::numbers::pi / 2.0F},
};

const nav::SpeedController::Config speed_controller_config{
    .linear_pid =
        {
            .kp = 15.0F,
            .ki = 1.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 40.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 5.0F,
            .ki = 1.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 40.0F,
            .max_integral = -1.0F,
        },
    .left_feed_forward =
        {
            .linear_speed = 15.369F,
            .linear_acceleration = 0.256F,
            .angular_speed = -0.645F,
            .angular_acceleration = -0.023F,
        },
    .right_feed_forward =
        {
            .linear_speed = 12.139F,
            .linear_acceleration = 1.393F,
            .angular_speed = 0.346F,
            .angular_acceleration = 0.024F,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
