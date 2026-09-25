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
 * feed forward gain and sensing window of the navigation is computed from them. The geometry comes
 * from the KiCad board and the SolidWorks assembly, the drive from the datasheets, and the values
 * marked as estimates were never measured.
 *
 * - The mass and the yaw inertia are estimates from the volumes of the parts in the SolidWorks
 *   assembly, with the datasheet mass of the motors, about 6 g for each of the three 250 mAh 1S
 *   cells of the battery and about 5 g for the components of the board, which the assembly leaves
 *   out. A scale settles the mass, and the drive identification the inertia.
 * - The tires are Kyosho MZW40-20, a thin slick of 20 degree compound, 22 mm across on the 20 mm
 *   seat of the hubs. The wheel radius is that of the assembly. The track width is the distance
 *   between the centers of the tires, which sit in the outer part of the board cutouts. Driving a
 *   known distance and a known number of turns calibrates both. The tires flatten under load and
 *   the wheels roll on a smaller radius: the rolling compliance is that of the simulated band,
 *   56 um less per newton on a tire, 0.17 % of the radius with the fan off and 0.7 % with it on.
 *   Driving a known distance with the fan on and off measures it.
 * - The outline is the rectangle that encloses whatever can touch a wall: the board, 50 by 90 mm,
 *   with the tires flush with its sides and the housings of the inner wall sensors 1.4 mm ahead of
 *   its nose.
 * - The friction coefficient comes from a tilt test of the robot on the maze floor, and the fan
 *   downforce from a scale under the robot with the fan running. The friction is an estimate; the
 *   downforce, about 3 N at full speed, is the owner's figure. The fan has no
 *   skirt, it draws through a 15 mm hole with a 1 mm gap under the whole board. It pulls over the
 *   hole, 17.5 mm ahead of the axle: the flow through the gap, which leaves the pressure harmonic
 *   between the edges of the board and the hole, puts the center of the suction at 14 to 16 mm.
 *   So the robot rests on the front edge of its board with the fan on, and the edge carries about a
 *   third of the downforce. Scales under the wheels and under the nose settle both.
 * - The lateral compliance is the simulation's: the tires there slide sideways at 4.8 mm/s per m/s^2
 *   of lateral acceleration, and the real tires are not measured yet. Driving a circle at a known
 *   speed with the fan on, and comparing where the robot ends with where the odometry says, does.
 * - The motors are Maxon DCX 8 M with the 4.2 V winding, run from the 19.63 V boost converter
 *   through a spur stage of 4 and 21 mm pitch diameters. The resistance is that of the winding,
 *   12 ohm, plus 0.62 ohm of the bridge, both at 25 degrees. The static friction voltage is the no
 *   load current of the motor times that resistance, which leaves out the friction of the gears.
 *   The motors reach their permissible 17300 rpm at 3.8 m/s, above the speed limit of the
 *   dynamics. The drive identification procedure measures all of these.
 * - The position of each wall sensor is the midpoint of the lenses of its emitter and receiver,
 *   7.2 mm ahead of its footprint along its axis, and the angle is that of the footprint: the two
 *   outer ones are square to the board and the two inner ones turned by 45 degrees. The half angle
 *   is the 3 degrees of the emitter datasheet plus the mounting tolerance. What the housings really
 *   do to both wants a bench check against the edge of a wall.
 * - The range noise of the wall sensors is an estimate, for a reading whose noise no other shares.
 * - The gyroscope noise is the density in the datasheet of the sensor, 2.8 mdps per square root of
 *   hertz, with a third more for what the robot adds to it.
 * - The wheel angle noise is the resolution of the encoders, whose magnets turn with the wheels.
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
            .mass = 0.07F,
            .yaw_inertia = 2.9e-5F,
            .wheel_radius = 0.011F,
            .rolling_compliance = 56e-6F,
            .track_width = 0.04575F,
            .half_width = 0.0251F,
            .front_length = 0.0549F,
            .rear_length = 0.0365F,
        },
    .traction =
        {
            .friction_coefficient = 1.0F,
            .fan_downforce = 3.0F,
            .fan_offset = 0.0175F,
            .lateral_compliance = 0.0048F,
        },
    .drive =
        {
            .torque_constant = 0.00336F,
            .resistance = 12.62F,
            .gear_ratio = 5.25F,
            .supply_voltage = 19.63F,
            .static_friction_voltage = 0.09F,
        },
    .wall_sensors = {{
        {.position = {.x = 0.0439F, .y = 0.0215F}, .angle = 0.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0526F, .y = 0.0153F}, .angle = std::numbers::pi_v<float> / 4.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0526F, .y = -0.0153F}, .angle = -std::numbers::pi_v<float> / 4.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0439F, .y = -0.0215F}, .angle = 0.0F, .half_angle = 0.09F},
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
