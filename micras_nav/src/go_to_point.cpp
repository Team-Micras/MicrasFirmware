/**
 * @file go_to_point.cpp
 *
 * @brief Nav GoToPoint class implementation
 *
 * @date 10/2024
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/nav/go_to_point.hpp"

namespace micras::nav {
GoToPoint::GoToPoint(Config config) :
    linear_pid(config.linear_pid),
    angular_pid(config.angular_pid),
    base_speed{config.base_speed},
    linear_decay_damping{config.linear_decay_damping},
    tolerance{config.tolerance} { }

Twist GoToPoint::action(const State& state, const Point& goal, float elapsed_time) {
    float angular_error = core::assert_angle(state.pose.orientation - state.pose.position.angle_between(goal));
    float linear_target = core::decay(angular_error, this->linear_decay_damping) * this->base_speed;

    float angular_command = this->angular_pid.update(angular_error, elapsed_time);
    float linear_command = this->linear_pid.update(state.velocity.linear - linear_target, elapsed_time);

    return {linear_command, angular_command};
}

bool GoToPoint::finished(const State& state, const Point& goal) const {
    return std::hypot(goal.x - state.pose.position.x, goal.y - state.pose.position.y) <= this->tolerance;
}
}  // namespace micras::nav
