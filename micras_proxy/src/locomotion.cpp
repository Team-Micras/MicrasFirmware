/**
 * @file
 */

#include <algorithm>
#include <cmath>

#include "micras/proxy/locomotion.hpp"

namespace micras::proxy {
Locomotion::Locomotion(const Config& config) :
    left_motor{config.left_motor}, right_motor{config.right_motor}, enable_gpio{config.enable_gpio} {
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

void Locomotion::set_command(float linear, float angular) {
    float left_command = linear - angular;
    float right_command = linear + angular;

    // Both wheels are scaled by the same factor, so that saturation preserves the commanded ratio
    // and therefore the turn radius
    const float peak = std::max(std::abs(left_command), std::abs(right_command));

    if (peak > 100.0F) {
        const float scale = 100.0F / peak;
        left_command *= scale;
        right_command *= scale;
    }

    this->set_wheel_command(left_command, right_command);
}

void Locomotion::stop() {
    this->set_wheel_command(0.0F, 0.0F);
}

bool Locomotion::was_initialized() const {
    return this->left_motor.was_initialized() and this->right_motor.was_initialized();
}
}  // namespace micras::proxy
