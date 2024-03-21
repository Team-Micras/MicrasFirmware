/**
 * @file dual_motor_driver.cpp
 *
 * @brief Proxy DualMotorDriver class source
 *
 * @date 03/2024
 */

#include "proxy/dual_motor_driver.hpp"

namespace proxy {
DualMotorDriver::DualMotorDriver(Config& motor_driver_config) :
    pwm_left_fwd{motor_driver_config.pwm_left_fwd},
    pwm_left_bwd{motor_driver_config.pwm_left_bwd},
    pwm_right_fwd{motor_driver_config.pwm_right_fwd},
    pwm_right_bwd{motor_driver_config.pwm_right_bwd},
    enable_gpio{motor_driver_config.enable_gpio} {
    this->enable();
    this->stop();
}

void DualMotorDriver::enable() {
    this->enable_gpio.write(true);
}

void DualMotorDriver::disable() {
    this->enable_gpio.write(false);
}

void DualMotorDriver::set_speed(float left_speed, float right_speed) {
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

void DualMotorDriver::stop() {
    this->stop_left_motor();
    this->stop_right_motor();
}

void DualMotorDriver::stop_left_motor() {
    this->pwm_left_fwd.set_duty_cycle(0.0f);
    this->pwm_left_bwd.set_duty_cycle(0.0f);
}

void DualMotorDriver::stop_right_motor() {
    this->pwm_right_fwd.set_duty_cycle(0.0f);
    this->pwm_right_bwd.set_duty_cycle(0.0f);
}
}  // namespace proxy
