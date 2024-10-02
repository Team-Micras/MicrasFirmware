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
constexpr uint8_t maze_width{16};
constexpr uint8_t maze_height{16};
constexpr float   cell_size{0.18};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 10.0F,
    .angular_cutoff_frequency = 10.0F,
    .wheel_radius = 0.0105F,
    .wheel_separation = 0.0585F,
    .initial_pose = {{cell_size / 2.0F, cell_size / 2.0F}, std::numbers::pi_v<float> / 2},
};

const nav::LookAtPoint::Config look_at_point_config{
    .pid =
        {
            .kp = 1.0F,
            .ki = 1.0F,
            .kd = 1.0F,
            .setpoint = 0.0F,
            .saturation = -1.0F,
            .max_integral = -1.0F,
        },
    .tolerance = 0.1F,
};

const nav::GoToPoint::Config go_to_point_config{
    .linear_pid =
        {
            .kp = 1.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = -1.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 1.0F,
            .ki = 0.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = -1.0F,
            .max_integral = -1.0F,
        },
    .base_speed = 0.1F,
    .linear_decay_damping = 0.1F,
    .tolerance = 0.1F,
};

const nav::Mapping<maze_width, maze_height>::Config mapping_config{
    .wall_thickness = 0.012F,
    .cell_size = cell_size / 2.0F,
    .wall_distance_threshold = 6.0F,
    .free_distance_threshold = 8.0F,
    .alignment_threshold = 0.1F,
    .front_sensor_pose = {{0.028F, 0.045F}, 0.0F},
    .side_sensor_pose = {{0.015F, 0.06F}, std::numbers::pi_v<float> / 4.0F},
    .start = {{0, 0}, nav::Side::UP},
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
