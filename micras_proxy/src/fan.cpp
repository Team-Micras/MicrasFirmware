/**
 * @file fan.cpp
 *
 * @brief Proxy Fan class source
 *
 * @date 03/2024
 */

#include "micras/core/utils.hpp"
#include "micras/proxy/fan.hpp"

namespace micras::proxy {
Fan::Fan(const Config& config) :
    pwm{config.pwm},
    direction_gpio{config.direction_gpio},
    enable_gpio{config.enable_gpio},
    max_acceleration{config.max_acceleration} {
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
    this->update();
    this->target_speed = speed;
}

float Fan::update() {
    this->current_speed = core::move_towards<float>(
        this->current_speed, this->target_speed, this->acceleration_timer.elapsed_time_ms() * this->max_acceleration
    );

    this->acceleration_timer.reset_ms();

    if (this->current_speed > 0.0F) {
        this->set_direction(RotationDirection::FORWARD);
        this->pwm.set_duty_cycle(this->current_speed);
    } else if (this->current_speed < 0.0F) {
        this->set_direction(RotationDirection::BACKWARDS);
        this->pwm.set_duty_cycle(-this->current_speed);
    } else {
        this->stop();
    }

    return this->current_speed;
}

void Fan::stop() {
    this->pwm.set_duty_cycle(0.0F);
}

void Fan::set_direction(RotationDirection direction) {
    this->direction_gpio.write(static_cast<bool>(direction));
}
}  // namespace micras::proxy
