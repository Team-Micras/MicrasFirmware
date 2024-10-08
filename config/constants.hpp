/**
 * @file constants.hpp
 *
 * @brief Navigation constants
 *
 * @date 04/2024
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <cstdint>
#include <numbers>

#include "micras/nav/go_to_point.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/look_at_point.hpp"
#include "micras/nav/mapping.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/nav/pid_controller.hpp"

namespace micras {
constexpr uint8_t maze_width{4};
constexpr uint8_t maze_height{4};
constexpr float   cell_size{0.18};

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
            .kp = 300.0F,
            .ki = 0.5F,
            .kd = 0.01F,
            .setpoint = 0.0F,
            .saturation = 30.0F,
            .max_integral = 30.0F,
        },
    .distance_tolerance = 0.015F,
    .velocity_tolerance = 0.015F,
};

const nav::GoToPoint::Config go_to_point_config{
    .linear_pid =
        {
            .kp = 8.0F,
            .ki = 14.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 50.0F,
            .max_integral = 40.0F,
        },
    .stop_pid =
        {
            .kp = 150.0F,
            .ki = 0.5F,
            .kd = 0.08F,
            .setpoint = 0.0F,
            .saturation = 25.0F,
            .max_integral = 25.0F,
        },
    .angular_pid =
        {
            .kp = 15.0F,
            .ki = 0.0F,
            .kd = 0.02F,
            .setpoint = 0.0F,
            .saturation = 20.0F,
            .max_integral = -1.0F,
        },
    .cell_size = cell_size,
    .base_speed = 0.3F,
    .linear_decay_damping = 0.1F,
    .distance_tolerance = 0.02F,
    .velocity_tolerance = 0.02F,
};

const nav::FollowWall::Config follow_wall_config = {
    .pid =
        {
            .kp = 90.0F,
            .ki = 0.0F,
            .kd = 0.04F,
            .setpoint = 0.0F,
            .saturation = 15.0F,
            .max_integral = -1.0F,
        },
    .can_follow_tolerance = 0.05F,
    .base_left_reading = 0.38,
    .base_right_reading = 0.31F,
    .cutoff_frequency = 5.0F,
};

const nav::Mapping<maze_width, maze_height>::Config mapping_config{
    .wall_thickness = 0.015F,
    .cell_size = cell_size,
    .alignment_threshold = 0.15F,
    .front_sensor_pose = {{0.028F, 0.045F}, 0.0F},
    .side_sensor_pose = {{0.015F, 0.06F}, std::numbers::pi_v<float> / 4.0F},
    .front_alignment_tolerance = 0.06F,
    .side_alignment_tolerance = 0.03F,
    .front_alignment_measure = {{
        0.93F,
        0.93F,
    }},
    .side_alignment_measure = {{
        0.38F,
        0.31F,
    }},
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{1, 0}},
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 5.0F,
    .wheel_radius = 0.011F,
    .initial_pose =
        {
            nav::Point::from_grid(mapping_config.start.position, cell_size),
            static_cast<uint8_t>(mapping_config.start.orientation) * std::numbers::pi_v<float> / 2,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
