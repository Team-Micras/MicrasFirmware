/**
 * @file
 */

#ifndef MICRAS_ROBOT_HPP
#define MICRAS_ROBOT_HPP

#include <numbers>

#include "micras/nav/robot_model.hpp"

namespace micras {
/**
 * @brief Physical description of the Micras v1 robot and of the classic maze.
 *
 * @note These are the constants to measure on the robot: every speed, acceleration, turn shape,
 * feed forward gain and sensing window of the navigation is computed from them. The values marked
 * as estimates were never measured and are only good enough to start from.
 *
 * - The mass wants a scale and the yaw inertia a bifilar pendulum, or the CAD model. The inertia
 *   here is that of a uniform plate of the size of the robot.
 * - The wheel radius and the track width are best calibrated by driving a known distance and a
 *   known number of turns. The track width here is the distance between the wheel cutouts of the
 *   board.
 * - The outline is the rectangle that encloses whatever can touch a wall. The one here is the board
 *   with some allowance for the tires, and is an estimate.
 * - The friction coefficient comes from a tilt test of the robot on the maze floor, and the fan
 *   downforce from a scale under the robot with the fan running. Both are estimates.
 * - The drive constants reproduce the feed forward that was identified with the previous firmware,
 *   which is why the resistance is not that of a winding. The drive identification procedure
 *   replaces them.
 * - The position and the angle of each wall sensor come from the board, where the footprints of
 *   the two outer ones are square to it and those of the two inner ones are turned by 45 degrees.
 *   The half angle comes from the datasheet of the emitter. What the housings really do to both
 *   wants a bench check against the edge of a wall.
 * - The range noise of the wall sensors is an estimate, for a reading whose noise no other shares.
 * - The gyroscope noise is the density in the datasheet of the sensor, 2.8 mdps per square root of
 *   hertz, with a third more for what the robot adds to it.
 * - The gyroscope scale comes from the scale calibration procedure.
 */
constexpr nav::RobotModel robot_model{
    .maze =
        {
            .cell_size = 0.18F,
            .wall_thickness = 0.0126F,
        },
    .chassis =
        {
            .mass = 0.13F,
            .yaw_inertia = 1.3e-4F,
            .wheel_radius = 0.0112F,
            .track_width = 0.04225F,
            .half_width = 0.03F,
            .front_length = 0.0535F,
            .rear_length = 0.04F,
        },
    .traction =
        {
            .friction_coefficient = 1.0F,
            .fan_downforce = 0.6F,
        },
    .drive =
        {
            .torque_constant = 0.00711F,
            .resistance = 21.9F,
            .gear_ratio = 4.0F,
            .supply_voltage = 20.0F,
            .static_friction_voltage = 0.0F,
        },
    .wall_sensors = {{
        {.position = {.x = 0.0367F, .y = 0.0215F}, .angle = 0.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0476F, .y = 0.0102F}, .angle = std::numbers::pi_v<float> / 4.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0476F, .y = -0.0102F}, .angle = -std::numbers::pi_v<float> / 4.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0367F, .y = -0.0215F}, .angle = 0.0F, .half_angle = 0.09F},
    }},
    .noise =
        {
            .gyroscope = 6.5e-5F,
            .gyroscope_bias_walk = 1.0e-4F,
            .wheel_angle = 2.0F * std::numbers::pi_v<float> / 16384.0F,
            .longitudinal_slip = 0.005F,
            .longitudinal_slip_per_acceleration = 0.0005F,
            .lateral_slip = 0.003F,
            .wall_range = 0.001F,
            .wall_range_per_meter = 0.017F,
        },
    .gyroscope_scale = 1.0F,
};
}  // namespace micras

#endif  // MICRAS_ROBOT_HPP
