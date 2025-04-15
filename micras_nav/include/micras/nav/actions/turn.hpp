/**
 * @file
 */

#ifndef MICRAS_NAV_TURN_ACTION_HPP
#define MICRAS_NAV_TURN_ACTION_HPP

#include <cmath>

#include "micras/nav/actions/base.hpp"

namespace micras::nav {
/**
 * @brief Class to follow the side walls using a PID controller.
 */
class TurnAction : public Action {
public:
    TurnAction(float angle, float curve_radius, float max_centrifugal_acceleration) :
        angle{angle},
        linear_speed{std::sqrt(max_centrifugal_acceleration * curve_radius)},
        angular_speed{std::copysignf(linear_speed / curve_radius, angle)} { }

    Twist get_twist(const State& /*state*/) const override { return {linear_speed, angular_speed}; }

    bool finished(const State& state) const override {
        return std::abs(state.pose.orientation) >= std::abs(this->angle);
    }

private:
    float angle;
    float linear_speed;
    float angular_speed;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_TURN_ACTION_HPP
