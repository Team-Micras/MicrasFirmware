/**
 * @file locomotion.cpp
 *
 * @brief Proxy Locomotion class source
 *
 * @date 03/2024
 */

#include "proxy/locomotion.hpp"

namespace proxy {
Locomotion::Locomotion(Config& config) :
    pwm_left_fwd{config.pwm_left_fwd}, pwm_left_bwd{config.pwm_left_bwd},
    pwm_right_fwd{config.pwm_right_fwd}, pwm_right_bwd{config.pwm_right_bwd},
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

void Locomotion::set_speed(float left_speed, float right_speed) {
    if (left_speed > 0.0f) {
        this->pwm_left_fwd.set_duty_cycle(left_speed);
        this->pwm_left_bwd.set_duty_cycle(0.0f);
    } else if (left_speed < 0.0f) {
        this->pwm_left_fwd.set_duty_cycle(0.0f);
        this->pwm_left_bwd.set_duty_cycle(-left_speed);
    } else {
        this->stop_left_motor();
    }

    if (right_speed > 0.0f) {
        this->pwm_right_fwd.set_duty_cycle(right_speed);
        this->pwm_right_bwd.set_duty_cycle(0.0f);
    } else if (right_speed < 0.0f) {
        this->pwm_right_fwd.set_duty_cycle(0.0f);
        this->pwm_right_bwd.set_duty_cycle(-right_speed);
    } else {
        this->stop_right_motor();
    }
}

void Locomotion::stop() {
    this->stop_left_motor();
    this->stop_right_motor();
}

void Locomotion::stop_left_motor() {
    this->pwm_left_fwd.set_duty_cycle(0.0f);
    this->pwm_left_bwd.set_duty_cycle(0.0f);
}

void Locomotion::stop_right_motor() {
    this->pwm_right_fwd.set_duty_cycle(0.0f);
    this->pwm_right_bwd.set_duty_cycle(0.0f);
}
}  // namespace proxy
