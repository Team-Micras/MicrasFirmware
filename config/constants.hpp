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
constexpr float    cell_size{0.18F};
constexpr uint32_t loop_time_us{125};
constexpr float    wall_thickness{0.0126F};
constexpr float    start_offset{0.04F + wall_thickness / 2.0F};
constexpr float    max_linear_acceleration{9.0F};
constexpr float    max_linear_deceleration{9.0F};
constexpr float    max_angular_acceleration{300.0F};
constexpr float    crash_acceleration{35.0F};
constexpr float    fan_speed{100.0F};

/**
 * @brief Rate at which the control loop runs, and therefore the rate at which every filter driven
 * by it is sampled.
 *
 * @note Derived from the loop period rather than written twice: a filter designed for a sampling
 * rate it is not sampled at is a filter with the wrong cutoff.
 *
 * @note The period is the one of the fastest sensor, the 8 kHz of the inertial measurement unit,
 * since an iteration without a new sample of anything has nothing to compute. The next periods that
 * keep that property are 250 and 500 microseconds, with the data rate of the sensor following.
 */
constexpr float loop_frequency{1.0e6F / static_cast<float>(loop_time_us)};

/**
 * @brief Rate at which the wall sensors produce a reading, which is the rate their filter runs at.
 *
 * @note One reading per period of the emitter timer, which the peripheral configuration makes four
 * periods of the control loop. The wall sensors check this value against the registers of the timer
 * when they start.
 */
constexpr float wall_sensors_frequency{2000.0F};

/**
 * @brief Time without a control loop iteration that resets the microcontroller.
 *
 * @note A reset brings the driver enable pins and the PWM outputs back to their reset state, which
 * makes the watchdog the shutdown path for a hang or a fault handler that never returns.
 *
 * @note It is a time and not a number of iterations, since what it bounds is how far the robot
 * travels with nobody driving it, and it has to stay above the longest iteration there is.
 */
constexpr uint32_t watchdog_timeout_ms{10};

/**
 * @brief Watchdog timeout used around a flash erase.
 *
 * @note Erasing a sector stalls the core for around 2 s, and up to 4 s in the worst case, since the
 * flash cannot be read while it is being erased. The robot is stopped whenever this happens.
 */
constexpr uint32_t flash_watchdog_timeout_ms{8000};

/**
 * @brief Cutoff frequencies of the sensor filters, in hertz.
 *
 * @note These are the cutoffs the firmware was actually running before the sampling rate and the
 * bilinear prewarping were corrected, so that fixing the mathematics changed no behavior on the
 * bench. They were never tuned against a working robot and are a starting point, not a result.
 */
///@{
constexpr float sensor_filter_cutoff{7.64F};
constexpr float torque_filter_cutoff{15.27F};
///@}

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
            .max_linear_speed = 0.4F,
            .max_linear_acceleration = max_linear_acceleration,
            .max_linear_deceleration = max_linear_deceleration,
            .max_centrifugal_acceleration = 2.78F,
            .max_angular_acceleration = max_angular_acceleration,
        },
    .solving = {
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
    .post_threshold = 16.5F,
    .post_reference = 0.44F * cell_size,
    .post_clearance = 0.025F,
};

// NOLINTBEGIN(modernize-use-designated-initializers) the nested points and poses read better positionally
const nav::Maze::Config maze_config{
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{
        {maze_width / 2, maze_height / 2},
        {(maze_width - 1) / 2, maze_height / 2},
        {maze_width / 2, (maze_height - 1) / 2},
        {(maze_width - 1) / 2, (maze_height - 1) / 2},
    }},
    .cost_margin = 1.2F,
    .action_queuer_config = action_queuer_config,
};

const nav::Odometry::Config odometry_config{
    .linear_filter =
        {
            .cutoff_frequency = sensor_filter_cutoff,
            .sampling_frequency = loop_frequency,
        },
    .wheel_radius = 0.0112F,
    .initial_pose = {{cell_size / 2.0F, start_offset}, std::numbers::pi_v<float> / 2.0F},
};
// NOLINTEND(modernize-use-designated-initializers)

const nav::SpeedController::Config speed_controller_config{
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
    .right_feed_forward = {
        .linear_speed = 13.319F,
        .linear_acceleration = 2.878F,
        .angular_speed = 0.901F,
        .angular_acceleration = -0.0244F,
    },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
