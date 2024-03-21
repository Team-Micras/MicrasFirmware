/**
 * @file pwm.cpp
 *
 * @brief STM32 PWM HAL wrapper
 *
 * @date 03/2024
 */

#include "hal/pwm.hpp"

namespace hal {
Pwm::Pwm(Config& pwm_config) : timer{pwm_config.timer}, channel{pwm_config.timer_channel} {
    this->timer.pwm_start(this->channel);
    this->timer.set_compare(this->channel, 0);
}

void Pwm::set_duty_cycle(float duty_cycle) {
    uint32_t compare = (duty_cycle * this->timer.get_autoreload()) / 100;
    this->timer.set_compare(this->channel, compare);
}

void Pwm::set_frequency(uint32_t frequency) {
    uint32_t clock_freq = this->timer.get_timer_clock_freq();
    uint32_t prescaler = this->timer.get_prescaler();
    uint32_t autoreload = clock_freq / ((prescaler + 1) * frequency);
    this->timer.set_autoreload(autoreload);
}
}  // namespace hal
