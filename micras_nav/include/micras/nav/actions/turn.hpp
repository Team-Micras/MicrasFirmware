/**
 * @file
 */

#ifndef MICRAS_NAV_TURN_ACTION_HPP
#define MICRAS_NAV_TURN_ACTION_HPP

#include <algorithm>
#include <cmath>

#include "micras/nav/actions/base.hpp"

namespace micras::nav {
/**
 * @brief Action to turn the robot following a curve radius.
 */
class TurnAction : public Action {
public:
    /**
     * @brief Construct a new Turn Action object.
     *
     * @param action_type The type of the action to be performed.
     * @param angle Angle to turn in radians.
     * @param max_angular_speed Maximum angular speed in rad/s.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     */
    TurnAction(
        uint8_t action_type, float angle, float max_angular_speed, float linear_speed, float max_angular_acceleration
    ) :
        Action{{action_type, angle}, false, calculate_total_time(angle, max_angular_speed, max_angular_acceleration)},
        angle{angle},
        max_angular_speed{max_angular_speed},
        linear_speed{linear_speed},
        acceleration{max_angular_acceleration} { }

    /**
     * @brief Get the desired speeds for the robot to complete the action.
     *
     * @param current_pose The current pose of the robot.
     * @param time_step The time step for the action in seconds.
     * @return The desired speeds for the robot to complete the action.
     */
    Twist get_speeds(const Pose& /*current_pose*/, float time_step) override {
        this->elapsed_time += time_step;
        Twist twist{};

        if (this->elapsed_time < this->get_total_time() / 2.0F) {
            twist = {
                .linear = linear_speed,
                .angular = this->acceleration * this->elapsed_time,
            };
        } else {
            twist = {
                .linear = linear_speed,
                .angular = this->acceleration * (this->get_total_time() - this->elapsed_time),
            };
        }

        twist.angular = std::copysign(std::clamp(twist.angular, 0.0F, this->max_angular_speed), this->angle);

        return twist;
    }

    /**
     * @brief Check if the action is finished.
     *
     * @param current_pose The current pose of the robot.
     * @return True if the action is finished, false otherwise.
     */
    bool finished(const Pose& /*current_pose*/) override {
        if (this->elapsed_time >= this->get_total_time()) {
            this->elapsed_time = 0.0F;
            return true;
        }

        return false;
    }

    /**
     * @brief Calculate the maximum angular speed for a given curve radius and linear speed.
     *
     * @param angle Angle to turn in radians.
     * @param curve_radius Radius of the curve in meters.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     * @param max_centripetal_acceleration Maximum centripetal acceleration in m/s^2.
     * @return Maximum angular speed in rad/s.
     *
     * @details Maximum angular velocity is computed to generate a curve equivalent in displacement to one that
     * maintains the desired radius of curvature throughout the entire turn.
     * If linear speed is zero, a minimal angular speed is assigned.
     */
    static constexpr float calculate_max_angular_speed(
        float angle, float curve_radius, float max_angular_acceleration, float max_centripetal_acceleration
    ) {
        if (curve_radius == 0.0F) {
            return max_angular_acceleration * 0.01F;
        }

        const float correction_factor =
            14.35F - 13.57F * std::cos(std::abs(angle) - 2) - 10.0F / max_angular_acceleration;

        const float transformed_acceleration =
            std::pow(1.0F - std::cos(angle), 2.0F) * correction_factor * max_angular_acceleration;

        return std::sqrt(
            (max_centripetal_acceleration * transformed_acceleration) /
            (curve_radius * transformed_acceleration - max_centripetal_acceleration)
        );
    }

    /**
     * @brief Calculate the total side displacement for a curve.
     *
     * @param angle Angle to turn in radians.
     * @param max_angular_speed Maximum angular speed in rad/s.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     * @return Side displacement in meters.
     */
    static constexpr float calculate_side_displacement(
        float angle, float max_angular_speed, float linear_speed, float max_angular_acceleration
    ) {
        const float transformed_speed = max_angular_speed / (1.0F - std::cos(angle));
        const float correction_factor =
            14.35F - 13.57F * std::cos(std::abs(angle) - 2) - 10.0F / max_angular_acceleration;

        return linear_speed *
               (1.0F / transformed_speed + transformed_speed / (correction_factor * max_angular_acceleration));
    }

    /**
     * @brief Calculate the total forward displacement for a curve.
     *
     * @param angle Angle to turn in radians.
     * @param max_angular_speed Maximum angular speed in rad/s.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     * @return Forward displacement in meters.
     */
    static constexpr float calculate_forward_displacement(
        float angle, float max_angular_speed, float linear_speed, float max_angular_acceleration
    ) {
        const float transformed_speed = max_angular_speed / std::sin(std::abs(angle));
        const float correction_factor =
            14.12F - 13.3F * std::cos(std::abs(angle) - 1.146F) - 10.0F / max_angular_acceleration;

        return linear_speed *
               (1.0F / transformed_speed + transformed_speed / (correction_factor * max_angular_acceleration));
    }

    /**
     * @brief Calculate the total time to complete the action.
     *
     * @param angle Angle to turn in radians.
     * @param max_angular_speed Maximum angular speed in rad/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     * @return Total time to complete the action in seconds.
     */
    static constexpr float calculate_total_time(float angle, float max_angular_speed, float max_angular_acceleration) {
        const float peak_velocity = std::sqrt(std::abs(angle) * max_angular_acceleration);

        if (peak_velocity < max_angular_speed) {
            return 2.0F * peak_velocity / max_angular_acceleration;
        }

        const float acceleration_time = max_angular_speed / max_angular_acceleration;

        return std::abs(angle) / max_angular_speed + acceleration_time;
    }

private:
    /**
     * @brief Angle to turn in radians.
     */
    float angle;

    /**
     * @brief Maximum angular speed in rad/s.
     */
    float max_angular_speed;

    /**
     * @brief Linear speed in m/s while turning.
     */
    float linear_speed;

    /**
     * @brief Maximum angular acceleration in rad/s^2.
     */
    float acceleration;

    /**
     * @brief Elapsed time in seconds since the action started.
     */
    float elapsed_time{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_TURN_ACTION_HPP
