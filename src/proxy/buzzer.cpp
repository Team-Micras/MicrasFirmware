/**
 * @file buzzer.cpp
 *
 * @brief Proxy Buzzer class implementation
 *
 * @date 03/2024
 */

#include "proxy/buzzer.hpp"

namespace proxy {
Buzzer::Buzzer(Config& config) : pwm{config.pwm} {
    this->stop();
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->pwm.set_duty_cycle(50.0F);
    this->pwm.set_frequency(frequency);
    this->is_playing = true;

    if (duration > 0) {
        this->duration = duration;
        this->timer.reset_ms();
    }
}

void Buzzer::update() {
    if (this->is_playing and this->duration > 0 and this->timer.elapsed_time_ms() > this->duration) {
        this->stop();
    }
}

void Buzzer::stop() {
    this->is_playing = false;
    this->pwm.set_duty_cycle(0.0F);
}
}  // namespace proxy
