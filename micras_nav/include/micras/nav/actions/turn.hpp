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
     * @param curve_radius Radius of the curve in meters.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     */
    TurnAction(
        uint8_t action_type, float angle, float curve_radius, float linear_speed, float max_angular_acceleration
    ) :
        Action{{action_type, angle}, false},
        angle{angle},
        linear_speed{linear_speed},
        acceleration{max_angular_acceleration},
        max_angular_speed{calculate_max_angular_speed(angle, curve_radius, linear_speed, max_angular_acceleration)},
        curve_radius{curve_radius} {
        this->set_total_time(calculate_total_time(angle, max_angular_speed, max_angular_acceleration));
    }

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
     * @brief Increment the angle to move by a certain value.
     *
     * @param value_increment The increment value.
     * @return A reference to the current action.
     */
    TurnAction& operator+=(float value_increment) override {
        Action::operator+=(value_increment);
        this->angle += value_increment;
        this->max_angular_speed =
            calculate_max_angular_speed(this->angle, this->curve_radius, this->linear_speed, this->acceleration);

        this->set_total_time(calculate_total_time(this->angle, this->max_angular_speed, this->acceleration));

        return *this;
    }

    /**
     * @brief Decrement the angle to move by a certain value.
     *
     * @param value_increment The decrement value.
     * @return A reference to the current action.
     */
    TurnAction& operator-=(float value_increment) override { return *this += -value_increment; }

    /**
     * @brief Calculate the maximum angular speed for a given curve radius and linear speed.
     *
     * @param angle Angle to turn in radians.
     * @param curve_radius Radius of the curve in meters.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     * @return Maximum angular speed in rad/s.
     *
     * @details Maximum angular velocity is computed to generate a curve equivalent in displacement to one that
     * maintains the desired radius of curvature throughout the entire turn.
     * If linear speed is zero, a minimal angular speed is assigned.
     */
    static constexpr float calculate_max_angular_speed(
        float angle, float curve_radius, float linear_speed, float max_angular_acceleration
    ) {
        if (linear_speed == 0.0F) {
            return max_angular_acceleration * 0.01F;
        }
        const float correction_factor =
            14.35F - 13.57F * std::cos(std::abs(angle) - 2) - 10.0F / max_angular_acceleration;
        const float radius_speed_ratio = curve_radius / linear_speed;
        const float quadratic_term =
            1.0F / (std::pow(1 - std::cos(angle), 2.0F) * correction_factor * max_angular_acceleration);
        const float discriminant = std::pow(radius_speed_ratio, 2.0F) - 4.0F * quadratic_term;

        return (radius_speed_ratio - std::sqrt(discriminant)) / (2.0F * quadratic_term);
    }

private:
    /**
     * @brief Calculate the total time to complete the action.
     *
     * @param angle Angle to turn in radians.
     * @param max_angular_speed Maximum angular speed in rad/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     * @return Total time to complete the action in seconds.
     */
    static constexpr float calculate_total_time(float angle, float max_angular_speed, float max_angular_acceleration) {
        const float peak_velocity = std::sqrt(angle * max_angular_acceleration);

        if (peak_velocity < max_angular_speed) {
            return 2.0F * peak_velocity / max_angular_acceleration;
        }

        const float acceleration_time = max_angular_speed / max_angular_acceleration;

        return std::abs(angle) / max_angular_speed + acceleration_time;
    }

    /**
     * @brief Angle to turn in radians.
     */
    float angle;

    /**
     * @brief Linear speed in m/s while turning.
     */
    float linear_speed;

    /**
     * @brief Maximum angular acceleration in rad/s^2.
     */
    float acceleration;

    /**
     * @brief Maximum angular speed in rad/s while turning.
     */
    float max_angular_speed;

    /**
     * @brief Radius of the equivalent curve in meters.
     */
    float curve_radius;

    /**
     * @brief Elapsed time in seconds since the action started.
     */
    float elapsed_time{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_TURN_ACTION_HPP
