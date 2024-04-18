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
    pwm_left_forward{config.pwm_left_forward},
    pwm_left_backwards{config.pwm_left_backwards},
    pwm_right_forward{config.pwm_right_forward},
    pwm_right_backwards{config.pwm_right_backwards},
    enable_gpio{config.enable_gpio} {
    this->stop();
    this->enable();
}

void Locomotion::enable() {
    this->enable_gpio.write(true);
}

void Locomotion::disable() {
    this->enable_gpio.write(false);
}

void Locomotion::set_wheel_command(float left_command, float right_command) {
    if (left_command > 0.0F) {
        this->pwm_left_forward.set_duty_cycle(left_command);
        this->pwm_left_backwards.set_duty_cycle(0.0F);
    } else if (left_command < 0.0F) {
        this->pwm_left_forward.set_duty_cycle(0.0F);
        this->pwm_left_backwards.set_duty_cycle(-left_command);
    } else {
        this->pwm_left_forward.set_duty_cycle(0.0F);
        this->pwm_left_backwards.set_duty_cycle(0.0F);
    }

    if (right_command > 0.0F) {
        this->pwm_right_forward.set_duty_cycle(right_command);
        this->pwm_right_backwards.set_duty_cycle(0.0F);
    } else if (right_command < 0.0F) {
        this->pwm_right_forward.set_duty_cycle(0.0F);
        this->pwm_right_backwards.set_duty_cycle(-right_command);
    } else {
        this->pwm_right_forward.set_duty_cycle(0.0F);
        this->pwm_right_backwards.set_duty_cycle(0.0F);
    }
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
    this->pwm_left_forward.set_duty_cycle(0.0F);
    this->pwm_left_backwards.set_duty_cycle(0.0F);

    this->pwm_right_forward.set_duty_cycle(0.0F);
    this->pwm_right_backwards.set_duty_cycle(0.0F);
}
}  // namespace micras::proxy
