/**
 * @file locomotion.cpp
 *
 * @brief Proxy Locomotion class source
 *
 * @date 03/2024
 */

#include <cmath>

#include "proxy/locomotion.hpp"

namespace proxy {
Locomotion::Locomotion(const Config& config) :
    pwm_left_fwd{config.pwm_left_fwd},
    pwm_left_bwd{config.pwm_left_bwd},
    pwm_right_fwd{config.pwm_right_fwd},
    pwm_right_bwd{config.pwm_right_bwd},
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
        this->pwm_left_fwd.set_duty_cycle(left_speed);
        this->pwm_left_bwd.set_duty_cycle(0.0F);
    } else if (left_speed < 0.0F) {
        this->pwm_left_fwd.set_duty_cycle(0.0F);
        this->pwm_left_bwd.set_duty_cycle(-left_speed);
    } else {
        this->stop_left();
    }

    if (right_speed > 0.0F) {
        this->pwm_right_fwd.set_duty_cycle(right_speed);
        this->pwm_right_bwd.set_duty_cycle(0.0F);
    } else if (right_speed < 0.0F) {
        this->pwm_right_fwd.set_duty_cycle(0.0F);
        this->pwm_right_bwd.set_duty_cycle(-right_speed);
    } else {
        this->stop_right();
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
    this->stop_left();
    this->stop_right();
}

void Locomotion::stop_left() {
    this->pwm_left_fwd.set_duty_cycle(0.0F);
    this->pwm_left_bwd.set_duty_cycle(0.0F);
}

void Locomotion::stop_right() {
    this->pwm_right_fwd.set_duty_cycle(0.0F);
    this->pwm_right_bwd.set_duty_cycle(0.0F);
}
}  // namespace proxy
