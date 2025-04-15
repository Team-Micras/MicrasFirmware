/**
 * @file
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/nav/speed_controller.hpp"

namespace micras::nav {
SpeedController::SpeedController(const Config& config) :
    linear_pid(config.linear_pid),
    angular_pid(config.angular_pid),
    left_feed_forward{config.left_feed_forward},
    right_feed_forward{config.right_feed_forward} { }

Twist SpeedController::action(const Twist& current_twist, const Twist& desired_twist, float elapsed_time) {
    this->linear_pid.set_setpoint(desired_twist.linear);
    this->angular_pid.set_setpoint(desired_twist.angular);

    const float linear_command = this->linear_pid.update(current_twist.linear, elapsed_time);
    const float angular_command = this->angular_pid.update(current_twist.angular, elapsed_time);

    const float left_command = linear_command - angular_command;
    const float right_command = linear_command + angular_command;

    const float current_left_speed = current_twist.linear - current_twist.angular;
    const float desired_left_speed = desired_twist.linear - desired_twist.angular;
    const float left_ff =
        this->left_feed_forward.bias + this->left_feed_forward.speed * desired_left_speed +
        this->left_feed_forward.acceleration * (desired_left_speed - current_left_speed) / elapsed_time;

    const float current_right_speed = current_twist.linear + current_twist.angular;
    const float desired_right_speed = desired_twist.linear + desired_twist.angular;
    const float right_ff =
        this->right_feed_forward.bias + this->right_feed_forward.speed * desired_right_speed +
        this->right_feed_forward.acceleration * (desired_right_speed - current_right_speed) / elapsed_time;

    return {left_command + left_ff, right_command + right_ff};
}

void SpeedController::reset() {
    this->linear_pid.reset();
    this->angular_pid.reset();
}
}  // namespace micras::nav
