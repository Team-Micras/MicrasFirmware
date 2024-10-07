/**
 * @file look_at_point.cpp
 *
 * @brief Nav LookAtPoint class implementation
 *
 * @date 10/2024
 */

#include "micras/core/utils.hpp"
#include "micras/nav/look_at_point.hpp"

namespace micras::nav {
LookAtPoint::LookAtPoint(Config config) :
    linear_pid(config.linear_pid),
    angular_pid{config.angular_pid},
    distance_tolerance{config.distance_tolerance},
    velocity_tolerance{config.velocity_tolerance} { }

Twist LookAtPoint::action(const State& state, const Point& goal, float elapsed_time) {
    float angular_error = core::assert_angle(state.pose.orientation - state.pose.position.angle_between(goal));

    return {
        this->linear_pid.update(state.velocity.linear, elapsed_time),
        this->angular_pid.update(angular_error, elapsed_time, state.velocity.angular)
    };
}

void LookAtPoint::reset() {
    this->linear_pid.reset();
    this->angular_pid.reset();
}

bool LookAtPoint::finished(const State& state, const Point& goal, bool stop) const {
    return std::abs(core::assert_angle(state.pose.orientation - state.pose.position.angle_between(goal))) <=
               this->distance_tolerance and
           (not stop or std::abs(state.velocity.angular) <= this->velocity_tolerance);
}
}  // namespace micras::nav
