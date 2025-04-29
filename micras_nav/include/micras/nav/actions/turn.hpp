/**
 * @file
 */

#ifndef MICRAS_NAV_TURN_ACTION_HPP
#define MICRAS_NAV_TURN_ACTION_HPP

#include <algorithm>
#include <cmath>

#include "micras/nav/actions/base.hpp"

namespace micras::nav {
static constexpr float correction_factor{0.95F};

/**
 * @brief Action to turn the robot following a curve radius.
 */
class TurnAction : public Action {
public:
    /**
     * @brief Construct a new Turn Action object.
     *
     * @param angle Angle to turn in radians.
     * @param curve_radius Radius of the curve in meters.
     * @param linear_speed Linear speed in m/s.
     * @param max_angular_acceleration Maximum angular acceleration in rad/s^2.
     *
     * @details Maximum angular velocity is computed to generate a curve equivalent in displacement to one that
     * maintains the desired radius of curvature throughout the entire turn.
     * If linear speed is zero, a minimal angular speed is assigned.
     */
    TurnAction(float angle, float curve_radius, float linear_speed, float max_angular_acceleration) :
        start_orientation{max_angular_acceleration * 0.001F * 0.001F / 2.0F},
        angle{angle},
        linear_speed{linear_speed},
        acceleration{max_angular_acceleration},
        max_angular_speed{
            (linear_speed == 0.0F) ?
                (max_angular_acceleration * 0.01F) :
                (correction_factor * max_angular_acceleration *
                 (curve_radius / linear_speed - std::sqrt(
                                                    std::pow(curve_radius / linear_speed, 2.0F) -
                                                    2.0F / (correction_factor * max_angular_acceleration)
                                                )))
        } { }

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
            twist = {linear_speed, std::sqrt(2.0F * this->acceleration * current_orientation)};
        } else {
            twist = {
                linear_speed, std::sqrt(2.0F * this->acceleration * (std::abs(this->angle) - current_orientation))
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
