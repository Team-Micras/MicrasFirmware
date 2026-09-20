/**
 * @file
 */

#include <algorithm>
#include <cmath>

#include "micras/proxy/locomotion.hpp"

namespace micras::proxy {
Locomotion::Locomotion(const Config& config) :
    left_motor{config.left_motor},
    right_motor{config.right_motor},
    enable_gpio{config.enable_gpio},
    reserved_rotation{config.reserved_rotation} {
    this->stop();
    this->disable();
}

void Locomotion::enable() {
    this->enable_gpio.write(true);
}

void Locomotion::disable() {
    this->enable_gpio.write(false);
}

void Locomotion::set_wheel_command(float left_command, float right_command) {
    this->left_motor.set_command(left_command);
    this->right_motor.set_command(right_command);
}

Locomotion::Command Locomotion::set_command(float linear, float angular) {
    const float angular_limit = std::max(this->reserved_rotation, max_command - std::abs(linear));

    angular = std::clamp(angular, -angular_limit, angular_limit);

    const float linear_limit = max_command - std::abs(angular);

    linear = std::clamp(linear, -linear_limit, linear_limit);

    this->set_wheel_command(linear - angular, linear + angular);

    return {.linear = linear, .angular = angular};
}

void Locomotion::stop() {
    this->set_wheel_command(0.0F, 0.0F);
}

bool Locomotion::was_initialized() const {
    return this->left_motor.was_initialized() and this->right_motor.was_initialized();
}
}  // namespace micras::proxy
