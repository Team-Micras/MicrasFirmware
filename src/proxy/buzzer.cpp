/**
 * @file buzzer.cpp
 *
 * @brief Proxy Buzzer class implementation
 *
 * @date 03/2024
 */

#include "hal/mcu.hpp"
#include "proxy/buzzer.hpp"

namespace proxy {
Buzzer::Buzzer(Config& buzzer_config) : pwm{buzzer_config.pwm} {
    this->stop();
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->pwm.set_duty_cycle(50.0f);
    this->pwm.set_frequency(frequency);
    this->is_playing = true;

    if (duration > 0) {
        this->duration = duration;
        this->timer.reset_ms();
    }
}

void Buzzer::update() {
    if (this->is_playing and this->timer.elapsed_time_ms() > this->duration) {
        this->stop();
    }
}

void Buzzer::stop() {
    this->is_playing = false;
    this->pwm.set_duty_cycle(0.0f);
}
}  // namespace proxy
