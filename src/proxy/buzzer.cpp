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
Buzzer::Buzzer(const Config& buzzer_config) : pwm{buzzer_config.pwm} {
    this->stop();
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->pwm.set_duty_cycle(50.0f);
    this->pwm.set_frequency(frequency);
    this->is_playing = true;

    if (duration > 0) {
        this->duration = duration;
        hal::mcu::reset_timer(this->timer);
    }
}

void Buzzer::update() {
    if (this->is_playing and hal::mcu::get_timer_ms(this->timer) > this->duration) {
        this->stop();
    }
}

void Buzzer::stop() {
    this->is_playing = false;
    this->pwm.set_duty_cycle(0.0f);
}
}  // namespace proxy
