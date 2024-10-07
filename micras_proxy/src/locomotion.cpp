/**
 * @file locomotion.cpp
 *
 * @brief Proxy Locomotion class source
 *
 * @date 03/2024
 */

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

    if (std::abs(left_command) > 100.0F) {
        left_command *= 100.0F / std::abs(left_command);
        right_command *= 100.0F / std::abs(left_command);
    }

    if (std::abs(right_command) > 100.0F) {
        left_command *= 100.0F / std::abs(right_command);
        right_command *= 100.0F / std::abs(right_command);
    }

    this->set_wheel_command(left_command, right_command);
}

void Locomotion::stop() {
    this->set_wheel_command(0.0F, 0.0F);
}
}  // namespace micras::proxy
