/**
 * @file fan.cpp
 *
 * @brief Proxy Fan class source
 *
 * @date 03/2024
 */

#include "proxy/fan.hpp"

namespace proxy {
Fan::Fan(Config& config) : pwm{config.pwm}, direction_gpio{config.direction_gpio}, enable_gpio{config.enable_gpio} {
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enable_gpio.write(true);
}

void Fan::disable() {
    this->enable_gpio.write(false);
}

void Fan::set_speed(float speed) {
    if (speed > 0.0f) {
        this->set_direction(RotationDirection::FORWARD);
        this->pwm.set_duty_cycle(speed);
    } else if (speed < 0.0f) {
        this->set_direction(RotationDirection::BACKWARD);
        this->pwm.set_duty_cycle(-speed);
    } else {
        this->stop();
    }
}

void Fan::stop() {
    this->set_speed(0.0f);
}

void Fan::set_direction(RotationDirection direction) {
    this->direction_gpio.write(static_cast<bool>(direction));
}
}  // namespace proxy
