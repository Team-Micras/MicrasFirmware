/**
 * @file
 */

#include <algorithm>
#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/nav/speed_controller.hpp"

namespace micras::nav {
SpeedController::SpeedController(const Config& config) :
    max_linear_acceleration{config.max_linear_acceleration},
    max_angular_acceleration{config.max_angular_acceleration},
    linear_pid(config.linear_pid),
    angular_pid(config.angular_pid),
    left_feed_forward{config.left_feed_forward},
    right_feed_forward{config.right_feed_forward} { }

std::pair<float, float>
    SpeedController::action(const Twist& current_twist, const Twist& desired_twist, float elapsed_time) {
    this->linear_pid.set_setpoint(desired_twist.linear);
    this->angular_pid.set_setpoint(desired_twist.angular);

    const float linear_command = this->linear_pid.update(current_twist.linear, elapsed_time);
    const float angular_command = this->angular_pid.update(current_twist.angular, elapsed_time);

    const float linear_acceleration = (desired_twist.linear - this->last_linear_speed) / elapsed_time;
    const float angular_acceleration = (desired_twist.angular - this->last_angular_speed) / elapsed_time;
    const Twist acceleration_twist{
        std::clamp(linear_acceleration, -this->max_linear_acceleration, this->max_linear_acceleration),
        std::clamp(angular_acceleration, -this->max_angular_acceleration, this->max_angular_acceleration)
    };

    this->last_linear_speed = desired_twist.linear;
    this->last_angular_speed = desired_twist.angular;

    const float left_feed_forward = feed_forward(desired_twist, acceleration_twist, this->left_feed_forward);
    const float right_feed_forward = feed_forward(desired_twist, acceleration_twist, this->right_feed_forward);

    return {
        left_feed_forward + linear_command - angular_command, right_feed_forward + linear_command + angular_command
    };
}

float SpeedController::feed_forward(const Twist& speed, const Twist& acceleration, const Config::FeedForward& config) {
    return config.linear_speed * speed.linear + config.linear_acceleration * acceleration.linear +
           config.angular_speed * speed.angular + config.angular_acceleration * acceleration.angular;
}

void SpeedController::reset() {
    this->linear_pid.reset();
    this->angular_pid.reset();
}
}  // namespace micras::nav
