/**
 * @file motor_driver.cpp
 *
 * @brief Proxy MotorDriver class source
 *
 * @date 03/2024
 */

#include "proxy/motor_driver.hpp"

namespace proxy {
MotorDriver::MotorDriver(Config& motor_driver_config) :
    pwm{motor_driver_config.pwm},
    direction_gpio{motor_driver_config.direction_gpio},
    enable_gpio{motor_driver_config.enable_gpio} {
    this->enable();
    this->stop();
}

void MotorDriver::enable() {
    this->enable_gpio.write(true);
}

void MotorDriver::disable() {
    this->enable_gpio.write(false);
}

void MotorDriver::set_speed(float speed) {
    if (speed > 0.0f) {
        this->pwm.set_duty_cycle(speed);
        this->set_direction(RotationDirection::FORWARD);
    } else if (speed < 0.0f) {
        this->pwm.set_duty_cycle(-speed);
        this->set_direction(RotationDirection::BACKWARD);
    } else {
        this->stop();
    }
}

void MotorDriver::stop() {
    this->set_speed(0.0f);
}

void MotorDriver::set_direction(RotationDirection direction) {
    this->direction_gpio.write((bool) direction);
}
}  // namespace proxy
