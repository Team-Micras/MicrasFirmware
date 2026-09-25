/**
 * @file
 */

#ifndef MICRAS_NAV_ROBOT_MODEL_HPP
#define MICRAS_NAV_ROBOT_MODEL_HPP

#include <array>
#include <cstdint>

#include "micras/core/vector.hpp"

namespace micras::nav {
/**
 * @brief Number of wall sensors the navigation is written for.
 */
inline constexpr uint8_t number_of_wall_sensors{4};

/**
 * @brief Physical description of a robot and of the maze it runs in.
 *
 * @details Every limit, gain and geometric window of the navigation is derived from these constants,
 * so tuning the robot is a matter of measuring them. All of them have a unit and a plausible range,
 * which makes a wrong one visible, and another robot reuses the navigation by describing itself here.
 *
 * @note The body frame has its origin at the midpoint of the axle, x forward and y to the left.
 */
struct RobotModel {
    /**
     * @brief Standard acceleration of gravity, in m/s^2.
     */
    static constexpr float gravity{9.80665F};

    /**
     * @brief Dimensions of the maze, in meters.
     */
    struct Maze {
        float cell_size;
        float wall_thickness;
    };

    /**
     * @brief Mass properties and outline of the robot.
     *
     * @note The outline is the rectangle that encloses everything that can touch a wall, given by
     * its half width and by how far it extends ahead of and behind the axle. The rolling compliance
     * is how much the radius a wheel rolls on shrinks for each newton on its tire, as the tire
     * flattens under the load.
     */
    struct Chassis {
        float mass;
        float yaw_inertia;
        float wheel_radius;
        float rolling_compliance;
        float track_width;
        float half_width;
        float front_length;
        float rear_length;
    };

    /**
     * @brief What limits the force the tires can transmit.
     *
     * @note The fan downforce is the extra normal force, in newtons, with the fan at the speed it
     * is run at, and the fan offset is how far ahead of the axle it pulls, in meters. Pulling ahead
     * of the axle, the fan tips the robot onto the front edge of its board, which then carries the
     * share of the downforce that the lever of the offset over the front length gives it. The
     * lateral compliance is how fast the tires slide sideways, to the outside of a curve, per unit
     * of lateral acceleration, in m/s per m/s^2: a tire only pushes sideways by slipping a little.
     */
    struct Traction {
        float friction_coefficient;
        float fan_downforce;
        float fan_offset;
        float lateral_compliance;
    };

    /**
     * @brief Electrical and mechanical constants of one side of the drive train.
     *
     * @note The torque constant, in N*m/A, is numerically equal to the back EMF constant in
     * V*s/rad. The gear ratio is motor turns per wheel turn. The static friction voltage is what it
     * takes to keep one side turning slowly, and the supply voltage is what a command of 100 %
     * applies to a motor.
     */
    struct Drive {
        float torque_constant;
        float resistance;
        float gear_ratio;
        float supply_voltage;
        float static_friction_voltage;
    };

    /**
     * @brief Mounting of one wall sensor.
     *
     * @note The angle of the optical axis is measured from the forward direction, positive to the
     * left. The half angle is that of the cone the emitter illuminates, plus the mounting tolerance.
     */
    struct WallSensor {
        core::Vector position;
        float        angle;
        float        half_angle;
    };

    /**
     * @brief Noise of the sensors, which is what the pose estimator weighs them by.
     *
     * @note The gyroscope noise is its rate noise density, in rad/s per square root of hertz, which
     * unlike the deviation of a sample does not depend on how often it is sampled. Its bias walk
     * is how much the bias may drift per square root of second. The wheel angle noise is the
     * resolution of the encoders, in radians of wheel. The slip allowances are the
     * standard deviation of the traveled distance and of the sideways motion per square root of
     * meter traveled, the first of them growing with the acceleration. The range noise of a wall
     * sensor is a constant part plus a part proportional to the range, for a reading that shares
     * its noise with no other, which is for the estimator to account for when the readings come
     * faster than their filter lets them change.
     */
    struct Noise {
        float gyroscope;
        float gyroscope_bias_walk;
        float wheel_angle;
        float longitudinal_slip;
        float longitudinal_slip_per_acceleration;
        float lateral_slip;
        float wall_range;
        float wall_range_per_meter;
    };

    /**
     * @brief Get the part of the fan downforce that the tires carry.
     *
     * @return The downforce on the tires in newtons.
     */
    constexpr float tire_downforce() const {
        return this->traction.fan_downforce * (1.0F - this->traction.fan_offset / this->chassis.front_length);
    }

    /**
     * @brief Get the radius the wheels roll on under the load on their tires.
     *
     * @param downforce The share of the fan downforce there is, from 0 with the fan off to 1.
     * @return The rolling radius in meters.
     */
    constexpr float rolling_radius(float downforce) const {
        const float load = (this->chassis.mass * gravity + downforce * this->tire_downforce()) / 2.0F;

        return this->chassis.wheel_radius - this->chassis.rolling_compliance * load;
    }

    /**
     * @brief Get the largest acceleration the tires can transmit, in any direction.
     *
     * @param fan_on Whether the fan is adding downforce.
     * @return The traction limited acceleration in m/s^2.
     */
    constexpr float traction_acceleration(bool fan_on) const {
        return this->traction.friction_coefficient *
               (gravity + (fan_on ? this->tire_downforce() / this->chassis.mass : 0.0F));
    }

    /**
     * @brief Get the largest angular acceleration the tires can transmit.
     *
     * @param fan_on Whether the fan is adding downforce.
     * @return The traction limited angular acceleration in rad/s^2.
     */
    constexpr float traction_angular_acceleration(bool fan_on) const {
        return this->traction_acceleration(fan_on) * this->chassis.mass * this->chassis.track_width /
               (2.0F * this->chassis.yaw_inertia);
    }

    /**
     * @brief Get the distance over which a turn ramps its curvature up.
     *
     * @details With the lateral and the angular accelerations both limited by the tires, the ratio
     * between them is a property of the chassis alone, and it fixes the length of the clothoid that
     * takes the robot from a straight into the tightest arc it can hold. It does not depend on the
     * friction, on the fan or on the speed.
     *
     * @return The length of the curvature ramp in meters.
     */
    constexpr float turn_ramp_length() const {
        return 2.0F * this->chassis.yaw_inertia / (this->chassis.mass * this->chassis.track_width);
    }

    /**
     * @brief Get the voltage per unit of linear speed, applied to both motors.
     *
     * @return The speed constant in V*s/m.
     */
    constexpr float speed_constant() const {
        return this->drive.torque_constant * this->drive.gear_ratio / this->chassis.wheel_radius;
    }

    /**
     * @brief Get the voltage per unit of linear acceleration, applied to both motors.
     *
     * @return The acceleration constant in V*s^2/m.
     */
    constexpr float acceleration_constant() const {
        return this->chassis.wheel_radius * this->drive.resistance * this->chassis.mass /
               (2.0F * this->drive.gear_ratio * this->drive.torque_constant);
    }

    /**
     * @brief Get the differential voltage per unit of angular speed.
     *
     * @return The angular speed constant in V*s/rad.
     */
    constexpr float angular_speed_constant() const { return this->speed_constant() * this->chassis.track_width / 2.0F; }

    /**
     * @brief Get the differential voltage per unit of angular acceleration.
     *
     * @return The angular acceleration constant in V*s^2/rad.
     */
    constexpr float angular_acceleration_constant() const {
        return this->chassis.yaw_inertia * this->chassis.wheel_radius * this->drive.resistance /
               (this->drive.gear_ratio * this->drive.torque_constant * this->chassis.track_width);
    }

    Maze                                           maze;
    Chassis                                        chassis;
    Traction                                       traction;
    Drive                                          drive;
    std::array<WallSensor, number_of_wall_sensors> wall_sensors;
    Noise                                          noise;

    /**
     * @brief Factor that corrects the sensitivity of the gyroscope, from the scale calibration.
     */
    float gyroscope_scale;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ROBOT_MODEL_HPP
