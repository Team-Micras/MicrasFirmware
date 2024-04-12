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

void Locomotion::set_wheel_speed(float left_speed, float right_speed) {
    if (left_speed > 0.0F) {
        this->pwm_left_forward.set_duty_cycle(left_speed);
        this->pwm_left_backwards.set_duty_cycle(0.0F);
    } else if (left_speed < 0.0F) {
        this->pwm_left_forward.set_duty_cycle(0.0F);
        this->pwm_left_backwards.set_duty_cycle(-left_speed);
    } else {
        this->pwm_left_forward.set_duty_cycle(0.0F);
        this->pwm_left_backwards.set_duty_cycle(0.0F);
    }

    if (right_speed > 0.0F) {
        this->pwm_right_forward.set_duty_cycle(right_speed);
        this->pwm_right_backwards.set_duty_cycle(0.0F);
    } else if (right_speed < 0.0F) {
        this->pwm_right_forward.set_duty_cycle(0.0F);
        this->pwm_right_backwards.set_duty_cycle(-right_speed);
    } else {
        this->pwm_right_forward.set_duty_cycle(0.0F);
        this->pwm_right_backwards.set_duty_cycle(0.0F);
    }
}

void Locomotion::set_speed(float linear, float angular) {
    float left_speed = linear - angular;
    float right_speed = linear + angular;

    if (std::abs(left_speed) > 100.0F) {
        left_speed *= 100.0F / std::abs(left_speed);
        right_speed *= 100.0F / std::abs(left_speed);
    }

    if (std::abs(right_speed) > 100.0F) {
        left_speed *= 100.0F / std::abs(right_speed);
        right_speed *= 100.0F / std::abs(right_speed);
    }

    this->set_wheel_speed(left_speed, right_speed);
}

void Locomotion::stop() {
    this->pwm_left_forward.set_duty_cycle(0.0F);
    this->pwm_left_backwards.set_duty_cycle(0.0F);

    this->pwm_right_forward.set_duty_cycle(0.0F);
    this->pwm_right_backwards.set_duty_cycle(0.0F);
}
}  // namespace micras::proxy
