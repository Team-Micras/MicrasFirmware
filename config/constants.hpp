/**
 * @file
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <cstdint>
#include <numbers>

#include "micras/core/pid_controller.hpp"
#include "micras/nav/go_to_point.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/look_at_point.hpp"
#include "micras/nav/mapping.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/proxy/storage.hpp"

namespace micras {
/*****************************************
 * Constants
 *****************************************/

constexpr uint8_t  maze_width{16};
constexpr uint8_t  maze_height{16};
constexpr float    cell_size{0.18};
constexpr uint32_t loop_time_us{1042};

/*****************************************
 * Template Instantiations
 *****************************************/

namespace nav {
using Maze = TMaze<maze_width, maze_height>;
using Mapping = TMapping<maze_width, maze_height>;
}  // namespace nav

/*****************************************
 * Configurations
 *****************************************/

const nav::LookAtPoint::Config look_at_point_config{
    .linear_pid =
        {
            .kp = 50.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 20.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 150.0F,
            .ki = 0.5F,
            .kd = 0.01F,
            .setpoint = 0.0F,
            .saturation = 20.0F,
            .max_integral = 15.0F,
        },
    .distance_tolerance = 0.015F,
    .velocity_tolerance = 0.015F,
};

const nav::GoToPoint::Config go_to_point_config{
    .stop_pid =
        {
            .kp = 270.0F,
            .ki = 0.5F,
            .kd = 0.08F,
            .setpoint = 0.0F,
            .saturation = 7.0F,
            .max_integral = 30.0F,
        },
    .angular_pid =
        {
            .kp = 150.0F,
            .ki = 0.0F,
            .kd = 0.02F,
            .setpoint = 0.0F,
            .saturation = 60.0F,
            .max_integral = -1.0F,
        },
    .cell_size = cell_size,
    .min_linear_command = 5.0F,
    .max_linear_command = 15.0F,
    .deceleration_factor = 0.3F,
    .distance_tolerance = 0.025F,
    .velocity_tolerance = 0.02F,
};

const nav::FollowWall::Config follow_wall_config = {
    .pid =
        {
            .kp = 100.0F,
            .ki = 0.0F,
            .kd = 0.04F,
            .setpoint = 0.0F,
            .saturation = 200.0F,
            .max_integral = -1.0F,
        },
    .base_left_reading = 0.168,
    .base_right_reading = 0.163F,
};

const nav::Mapping::Config mapping_config{
    .wall_thickness = 0.0126,
    .cell_size = cell_size,
    .front_sensor_pose = {{0.028F, 0.045F}, 0.0F},
    .side_sensor_pose = {{0.009F, 0.055F}, std::numbers::pi_v<float> / 6.0F},
    .front_distance_alignment_tolerance = 0.04F,
    .side_distance_alignment_tolerance = 0.02F,
    .front_orientation_alignment_tolerance = 0.02F,
    .side_orientation_alignment_tolerance = 0.02F,
    .front_distance_reading = {{
        0.57F,
        0.60F,
    }},
    .front_orientation_reading = {{
        0.32F,
        0.37F,
    }},
    .side_distance_reading = {{
        0.17F,
        0.12F,
    }},
    .start = {{0, 0}, nav::Side::UP},
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 5.0F,
    .wheel_radius = 0.0112F,
    .initial_pose =
        {
            {0.09F, 0.04F + mapping_config.wall_thickness / 2.0F},
            static_cast<uint8_t>(mapping_config.start.orientation) * std::numbers::pi_v<float> / 2,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
