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
 * @brief Class to follow the side walls using a PID controller.
 */
class TurnAction : public Action {
public:
    TurnAction(float angle, float curve_radius, float linear_speed, float max_angular_acceleration) :
        angle{angle},
        linear_speed{linear_speed},
        acceleration{max_angular_acceleration},
        max_speed{
            (linear_speed == 0.0F) ?
                (max_angular_acceleration * 0.01F) :
                (correction_factor * max_angular_acceleration *
                 (curve_radius / linear_speed - std::sqrt(
                                                    std::pow(curve_radius / linear_speed, 2.0F) -
                                                    2.0F / (correction_factor * max_angular_acceleration)
                                                )))
        } { }

    Twist get_twist(const Pose& pose) const override {
        Twist       twist{};
        const float current_orientation = std::abs(pose.orientation);

        if (current_orientation < std::abs(this->angle)) {
            twist = {
                linear_speed, std::sqrt(2.0F * this->acceleration * (current_orientation + this->acceleration / 2e6F))
            };
        } else {
            twist = {
                linear_speed, std::sqrt(2.0F * this->acceleration * (std::abs(this->angle) - current_orientation))
            };
        }

        twist.angular = std::copysign(std::clamp(twist.angular, -this->max_speed, this->max_speed), this->angle);

        return twist;
    }

    bool finished(const Pose& pose) const override { return std::abs(pose.orientation) >= std::abs(this->angle); }

    constexpr bool allow_follow_wall() const override { return false; }

private:
    float angle;
    float linear_speed;
    float acceleration;
    float max_speed;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_TURN_ACTION_HPP
