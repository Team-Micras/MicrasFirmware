/**
 * @file argb.cpp
 *
 * @brief Proxy Argb class implementation
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_ARGB_CPP
#define MICRAS_PROXY_ARGB_CPP

#include "proxy/argb.hpp"

namespace proxy {
template <uint8_t num_of_leds>
Argb<num_of_leds>::Argb(const Config& config) :
    pwm{config.pwm}, low_bit{pwm.get_compare(low_duty_cycle)}, high_bit{pwm.get_compare(high_duty_cycle)} { }

template <uint8_t num_of_leds>
void Argb<num_of_leds>::set_color(const Color& color, uint8_t index) {
    if (index >= num_of_leds) {
        return;
    }

    this->encode_color(color, index);
    this->pwm.start_dma(this->buffer.data(), this->buffer.size());
}

template <uint8_t num_of_leds>
void Argb<num_of_leds>::set_color(const Color& color) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->encode_color(color, i);
    }

    this->pwm.start_dma(this->buffer.data(), this->buffer.size());
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
    uint32_t data = color.green << (2 * bits_per_color) | color.red << bits_per_color | color.blue;

    for (uint32_t i = bits_per_led * index, j = 0; i < bits_per_led * (index + 1); i++) {
        this->buffer.at(i) = ((data >> j) & 1) == 1 ? this->high_bit : this->low_bit;
        j++;
    }
}
}  // namespace proxy

#endif  // MICRAS_PROXY_ARGB_CPP
