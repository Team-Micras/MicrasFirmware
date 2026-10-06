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
 * - The robot runs without its fan for now, which takes about 9 g off the 87 g the v2 model
 *   predicts with it: 78 g on a scale. The yaw inertia is the model's estimate from the volumes of
 *   the parts with their densities, less the fan motor 17.5 mm ahead of the axle; the drive
 *   identification settles it.
 * - The tires are 2 mm silicone bands stretched over 18.3 mm hubs into a channel between two
 *   flanges. The wheel radius is half of the 22.5 mm measured over the tires, against the 22.08 mm
 *   of the model, and the track width the 45 mm measured between their centers. Driving a known
 *   distance and a known number of turns calibrates both. The tires flatten under load and the wheels roll on a smaller radius: the
 *   rolling compliance is that of the simulated tire, 77 um less per newton on a tire (29 um at
 *   the 0.43 N of the fan off, 58 um at the 0.73 N of the fan on), still to measure on the v2
 *   tires by driving a known distance with the fan on and off.
 * - The outline is the rectangle that encloses whatever can touch a wall: the board, with the TPU
 *   bumper 3.2 mm ahead of its nose (56.7 mm ahead of the axle) and the keyway covers of the outer
 *   sensor caps 0.7 mm past its sides.
 * - The friction coefficient is an estimate, for a tilt test of the robot on the maze floor to
 *   measure. The fan downforce, about 1 N at full speed, is the prediction of the v2 fan study
 *   (MicrasHardware README: the 26.4 mm impeller on the 18000 rpm motor, with the skirt; the earlier
 *   figure was 3 N), for a scale under the robot with the fan running to check. The fan draws
 *   through a 15 mm hole with a 1 mm gap under the whole board, which a thin film skirt taped under
 *   its edge closes down to the floor. It pulls over the hole, 17.5 mm ahead of the axle: the
 *   flow through the gap, which leaves the pressure harmonic between the edges of the board and the
 *   hole, puts the center of the suction at 14 to 16 mm. So the robot rests on its nose with the
 *   fan on, on the TPU bumper's lower edge, which carries about a third of the downforce. Scales
 *   under the wheels and under the nose settle both.
 * - The lateral compliance is the simulation's: the tires there slide sideways at 4.8 mm/s per m/s^2
 *   of lateral acceleration, and the real tires are not measured yet. Driving a circle at a known
 *   speed with the fan on, and comparing where the robot ends with where the odometry says, does.
 * - The motors are 1020 coreless motors (9.61 x 20.3 mm measured), about 18000 rpm with no load at
 *   12 V (owner), run from the 19.63 V boost converter through a 0.5 module spur stage of 7 and 36
 *   teeth. The torque constant comes from the sweep of the check of the polarity, with the wheels
 *   in the air: the free left wheel gains 7.6 rad/s per percent of the command, 38.7 rad/s per volt,
 *   so its back EMF is at most 25.8 mV s/rad at the wheel, 5.0 mN m/A at the motor, against the 6.27
 *   the no-load speed of the datasheet gave. The resistance is still the estimate from a maker's 1020
 *   windings of the same speed: 16.06 ohm is 15.4 of the winding and 0.66 of the bridge and shunt.
 *   The static friction voltage is the command at which that wheel breaks away, 13 % of the supply
 *   in both directions: it holds both the friction of the drive and the part of each pulse the
 *   bridge loses at its 100 kHz, which the current sensors would be needed to tell apart. The right
 *   drive has more friction, up to twice as much backward, and the feedback makes up the difference.
 *   These are 12 V motors on a 19.63 V supply: at 20 V a stalled one draws about 1.2 A, so the limits
 *   of voltage and current matter. The drive identification procedure measures all of these.
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
            .mass = 0.078F,
            .yaw_inertia = 3.96e-5F,
            .wheel_radius = 0.01125F,
            .rolling_compliance = 77e-6F,
            .track_width = 0.045F,
            .half_width = 0.0257F,
            .front_length = 0.0567F,
            .rear_length = 0.0365F,
        },
    .traction =
        {
            .friction_coefficient = 1.0F,
            .fan_downforce = 1.0F,
            .fan_offset = 0.0175F,
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
