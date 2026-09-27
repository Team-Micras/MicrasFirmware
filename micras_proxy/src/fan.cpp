/**
 * @file
 */

#include <algorithm>

#include "micras/core/utils.hpp"
#include "micras/proxy/fan.hpp"

namespace micras::proxy {
Fan::Fan(const Config& config) :
    pwm{config.pwm}, enable_gpio{config.enable_gpio}, max_acceleration{config.max_acceleration} {
    this->stop();
    this->enable();
}

void Fan::enable() {
    this->enabled = true;
    this->enable_gpio.write(true);
}

void Fan::disable() {
    this->enabled = false;
    this->enable_gpio.write(false);
}

void Fan::set_speed(float speed) {
    this->update();
    this->target_speed = speed;
}

float Fan::update() {
    this->current_speed = core::move_towards<float>(
        this->current_speed, this->target_speed, this->acceleration_stopwatch.elapsed_time_ms() * this->max_acceleration
    );

    this->acceleration_stopwatch.reset_ms();

    this->pwm.set_duty_cycle(std::max(this->current_speed, 0.0F));

    return this->current_speed;
}

void Fan::stop() {
    this->target_speed = 0.0F;
    this->current_speed = 0.0F;
    this->pwm.set_duty_cycle(0.0F);
}

bool Fan::check_fault() const {
    return this->enabled and not this->enable_gpio.read();
}

bool Fan::was_initialized() const {
    return this->pwm.was_initialized();
}
}  // namespace micras::proxy
