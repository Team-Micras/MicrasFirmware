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
    linear_pid(config.linear_pid), angular_pid{config.angular_pid}, tolerance{config.tolerance} { }

Twist LookAtPoint::action(const State& state, const Point& goal, float elapsed_time) {
    float angular_target = state.pose.position.angle_between(goal);
    return {
        this->linear_pid.update(state.velocity.linear, elapsed_time),
        this->angular_pid.update(core::assert_angle(state.pose.orientation - angular_target), elapsed_time)
    };
}

bool LookAtPoint::finished(const Pose& pose, const Point& goal) const {
    return std::abs(core::assert_angle(pose.orientation - pose.position.angle_between(goal))) <= this->tolerance;
}
}  // namespace micras::nav
