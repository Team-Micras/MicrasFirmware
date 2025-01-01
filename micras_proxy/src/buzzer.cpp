/**
 * @file
 */

#include "micras/proxy/buzzer.hpp"

namespace micras::proxy {
Buzzer::Buzzer(const Config& config) : pwm{config.pwm} {
    this->stop();
}

void Buzzer::play(uint32_t frequency, uint32_t duration) {
    this->pwm.set_frequency(frequency);
    this->pwm.set_duty_cycle(50.0F);
    this->is_playing = true;
    this->duration = duration;

    if (duration > 0) {
        this->timer.reset_ms();
    }
}

void Buzzer::update() {
    if (this->is_playing and this->duration > 0 and this->timer.elapsed_time_ms() > this->duration) {
        this->stop();
    }
}

void Buzzer::wait(uint32_t interval) {
    while (this->duration > 0 and this->is_playing) {
        this->update();
    }

    hal::Timer wait_timer;

    while (wait_timer.elapsed_time_ms() < interval) {
        this->update();
    }
}

void Buzzer::stop() {
    this->is_playing = false;
    this->pwm.set_duty_cycle(0.0F);
}
}  // namespace micras::proxy
