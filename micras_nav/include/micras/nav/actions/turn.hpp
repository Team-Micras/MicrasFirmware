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
 * @brief Correction factor to adjust the maximum angular speed.
 *
 * @details This factor is determined by analyzing the graph of the maximum angular speed.
 */
static constexpr float correction_factor{0.95F};

/**
 * @brief Action to turn the robot following a curve radius.
 */
class TurnAction : public Action {
public:
    /**
     * @brief Construct a new Turn Action object.
     *
     * @param action_id The ID of the action.
     * @param angle Angle to turn in radians.
     * @param curve_radius Radius of the curve in meters.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     */
    TurnAction(uint8_t action_id, float angle, float curve_radius, float linear_speed, float max_angular_acceleration) :
        Action{action_id},
        start_orientation{max_angular_acceleration * 0.001F * 0.001F / 2.0F},
        angle{angle},
        linear_speed{linear_speed},
        acceleration{max_angular_acceleration},
        max_angular_speed{calculate_max_angular_speed(angle, curve_radius, linear_speed, max_angular_acceleration)} { }

    /**
     * @brief Get the desired speeds for the robot to complete the action.
     *
     * @param pose Current pose of the robot.
     * @return The desired speeds for the robot to complete the action.
     *
     * @details The desired velocity is calculated from the angular displacement based on the Torricelli equation.
     */
    Twist get_speeds(const Pose& pose) const override {
        Twist       twist{};
        const float current_orientation = std::max(std::abs(pose.orientation), this->start_orientation);

        if (current_orientation < std::abs(this->angle)) {
            twist = {
                .linear = linear_speed,
                .angular = std::sqrt(2.0F * this->acceleration * current_orientation),
            };
        } else {
            twist = {
                .linear = linear_speed,
                .angular = std::sqrt(2.0F * this->acceleration * (std::abs(this->angle) - current_orientation)),
            };
        }

        twist.angular =
            std::copysign(std::clamp(twist.angular, -this->max_angular_speed, this->max_angular_speed), this->angle);

        return twist;
    }

    /**
     * @brief Check if the action is finished.
     *
     * @param pose Current pose of the robot.
     * @return True if the action is finished, false otherwise.
     */
    bool finished(const Pose& pose) const override { return std::abs(pose.orientation) >= std::abs(this->angle); }

    /**
     * @brief Check if the action allows following the wall.
     *
     * @return True if the action allows following the wall, false otherwise.
     */
    bool allow_follow_wall() const override { return false; }

private:
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

        const float radius_speed_ratio = curve_radius / linear_speed;
        const float discriminant = std::pow(radius_speed_ratio, 2.0F) -
                                   (2.0F * (1 - std::cos(angle))) / (correction_factor * max_angular_acceleration);

        return correction_factor * max_angular_acceleration * (radius_speed_ratio - std::sqrt(discriminant));
    }

    /**
     * @brief Start orientation in radians. Being zero causes the robot to not move.
     */
    float start_orientation;

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
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_TURN_ACTION_HPP
