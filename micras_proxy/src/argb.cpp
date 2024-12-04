/**
 * @file
 */

#ifndef MICRAS_PROXY_ARGB_CPP
#define MICRAS_PROXY_ARGB_CPP

#include "micras/proxy/argb.hpp"

namespace micras::proxy {
template <uint8_t num_of_leds>
Argb<num_of_leds>::Argb(const Config& config) :
    pwm{config.pwm},
    low_bit{pwm.get_compare(config.low_duty_cycle)},
    high_bit{pwm.get_compare(config.high_duty_cycle)},
    brightness{config.max_brightness / 100.0F} {
    this->set_color({0, 0, 0});
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::set_color(const Color& color, uint8_t index) {
    if (index >= num_of_leds or this->pwm.is_busy()) {
        return;
    }

    this->encode_color(color * this->brightness, index);
    this->pwm.start_dma(this->buffer);
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::set_color(const Color& color) {
    if (this->pwm.is_busy()) {
        return;
    }

    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->encode_color(color * this->brightness, i);
    }

    this->pwm.start_dma(this->buffer);
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::turn_off(uint8_t index) {
    this->set_color(index, {0, 0, 0});
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::turn_off() {
    this->set_color({0, 0, 0});
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::encode_color(const Color& color, uint8_t index) {
    uint32_t data = (color.green << (2 * bits_per_color)) | (color.red << bits_per_color) | color.blue;

    for (uint32_t i = bits_per_led * index, j = bits_per_led - 1; i < bits_per_led * (index + 1U); i++, j--) {
        this->buffer.at(i) = ((data >> j) & 1) == 1 ? this->high_bit : this->low_bit;
    }
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ARGB_CPP
