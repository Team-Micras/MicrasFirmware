/**
 * @file
 */

#ifndef MICRAS_ROBOT_HPP
#define MICRAS_ROBOT_HPP

#include <numbers>

#include "maze_config.hpp"
#include "micras/nav/robot_model.hpp"

namespace micras {
/**
 * @brief Physical description of the Micras robot, on the v2 chassis (MicrasHardware, chassis-v2),
 * and of the maze it runs in, which maze_config.hpp describes.
 *
 * @note These are the constants to measure on the robot: every speed, acceleration, turn shape,
 * feed forward gain and sensing window of the navigation is computed from them. The geometry comes
 * from the KiCad board and the v2 chassis (the parametric build123d model in MicrasHardware), the
 * drive from measurements and estimates, and the values marked as estimates were never measured.
 * sim/robot.toml holds the same quantities as the physical truth of the simulation, each with its
 * source.
 *
 * - The robot runs with its fan: 78 g on a scale without it, against the 76.7 g the MicrasHardware
 *   model predicts without the fan, its mount and their screws, and 86 g with them, with the brass
 *   gears; the 9.3 g they add make 87.3 g, until the robot is weighed with its fan. The yaw inertia
 *   is that model's, 34.4 g cm^2 about the axle from the volumes of the parts with their densities,
 *   scaled to the 78 g, plus the 9.3 g of the fan 17.5 mm ahead of the axle; the drive
 *   identification settles it.
 * - The tires are 2 mm silicone bands stretched over 18.3 mm hubs into a channel between two
 *   flanges. The wheel radius is the rolling radius measured on the robot, 11.384 mm from rolling it
 *   by hand over 615 mm between two walls of the maze with the fan off, plus the 29 um its tires
 *   flatten under that load; the two wheels agree within 0.13 %. The track width is the one the
 *   wheels turn on, 47.2 mm from five turns in place against the gyroscope (whose scale the same
 *   turns measured at 1.0003 against a wall), wider than the 45 mm between the tire centers. The
 *   tires flatten under load and the wheels roll on a smaller radius: the rolling compliance is that
 *   of the simulated tire, 77 um less per newton on a tire (29 um at the 0.43 N a tire carries with
 *   the fan off, about 70 um at the 1.05 N with the fan on). Driving a known distance with the fan on and
 *   off measures the real one.
 * - The outline is the rectangle that encloses whatever can touch a wall: the board, whose nose is
 *   53.5 mm ahead of the axle and whose back is 36.5 mm behind it (90 mm long, 89.9 mm measured on
 *   the robot, which has no bumper), and the keyway covers of the outer sensor caps
 *   0.7 mm past its sides.
 * - The friction coefficient is tan(28.5 deg), the slope at which the robot slides sideways on the
 *   maze floor, its wheels across the slope. The fan runs at half of the battery, about 6.2 V, and
 *   with its skirt it pulled a plate on a scale under the board up by 150 g. The MicrasHardware fan
 *   study centers the suction under a sealed skirt 7.8 mm ahead of the axle. The nose rests on its
 *   front skid with the fan on, which carries the share of the downforce that the center of the
 *   suction leaves it.
 * - The lateral compliance is the simulation's: the tires there slide sideways at 4.8 mm/s per m/s^2
 *   of lateral acceleration, and the real tires are not measured yet. Driving a circle at a known
 *   speed with the fan on, and comparing where the robot ends with where the odometry says, does.
 * - The motors are 1020 coreless motors (9.61 x 20.3 mm measured), about 18000 rpm with no load at
 *   12 V (owner), run from the 19.63 V boost converter through a 0.5 module spur stage of 7 and 36
 *   teeth. The torque constant comes from the sweep of the check of the polarity, with the wheels in
 *   the air: the free left wheel gains 7.6 rad/s per percent of the command, 38.7 rad/s per volt, so
 *   its back EMF is at most 25.8 mV s/rad at the wheel, 5.0 mN m/A at the motor. The resistance is
 *   the estimate from a maker's 1020 windings of the same speed: 16.06 ohm is 15.4 of the
 *   winding and 0.66 of the bridge and shunt. The static friction voltage is the command at which
 *   that wheel breaks away, 13 % of the supply in both directions: it holds both the friction of the
 *   drive and the part of each pulse the bridge loses at its 100 kHz, which the current sensors
 *   would be needed to tell apart. The right drive has more friction, up to twice as much backward,
 *   and the feedback makes up the difference. These are 12 V motors on a 19.63 V supply: at 20 V a
 *   stalled one draws about 1.2 A, so the limits of voltage and current matter. The drive
 *   identification procedure measures all of these.
 * - The position of each wall sensor is the midpoint of the lenses of its emitter and receiver in
 *   the v2 caps (MicrasHardware leds.py and front.py), and the angle is that of the footprint: the
 *   two outer ones are square to the board and the two inner ones turned by 45 degrees. The half
 *   angle is the 3 degrees of the emitter datasheet plus the mounting tolerance. What the caps
 *   really do to both wants a bench check against the edge of a wall.
 * - The range noise of the wall sensors is an estimate, for a reading whose noise no other shares.
 * - The gyroscope noise is the density in the datasheet of the sensor, 2.8 mdps per square root of
 *   hertz, with a third more for what the robot adds to it.
 * - The wheel angle noise is the resolution of the encoders, whose magnets turn with the wheels.
 * - The gyroscope scale comes from the scale calibration procedure.
 */
constexpr nav::RobotModel robot_model{
    .maze = maze_geometry,
    .chassis =
        {
            .mass = 0.0873F,
            .yaw_inertia = 3.78e-5F,
            .wheel_radius = 0.011413F,
            .rolling_compliance = 77e-6F,
            .track_width = 0.0472F,
            .half_width = 0.0257F,
            .front_length = 0.0535F,
            .rear_length = 0.0365F,
        },
    .traction =
        {
            .friction_coefficient = 0.54F,
            .fan_downforce = 1.47F,
            .fan_offset = 0.008F,
            .lateral_compliance = 0.0048F,
        },
    .drive =
        {
            .torque_constant = 0.0050F,
            .resistance = 16.06F,
            .gear_ratio = 36.0F / 7.0F,
            .supply_voltage = 19.63F,
            .static_friction_voltage = 2.5F,
        },
    .wall_sensors = {{
        {.position = {.x = 0.0429F, .y = 0.0215F}, .angle = 0.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0519F, .y = 0.01455F}, .angle = std::numbers::pi_v<float> / 4.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0519F, .y = -0.01455F}, .angle = -std::numbers::pi_v<float> / 4.0F, .half_angle = 0.09F},
        {.position = {.x = 0.0429F, .y = -0.0215F}, .angle = 0.0F, .half_angle = 0.09F},
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
