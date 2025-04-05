/**
 * @file
 */

#ifndef MICRAS_PROXY_ARGB_CPP
#define MICRAS_PROXY_ARGB_CPP

#include "micras/proxy/argb.hpp"

namespace micras::proxy {
template <uint8_t num_of_leds>
TArgb<num_of_leds>::TArgb(const Config& config) :
    pwm{config.pwm},
    low_bit{pwm.get_compare(config.low_duty_cycle)},
    high_bit{pwm.get_compare(config.high_duty_cycle)},
    brightness{config.max_brightness / 100.0F} {
    this->turn_off();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_color(const Color& color, uint8_t index) {
    this->colors.at(index) = color;
    this->update();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_color(const Color& color) {
    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->colors.at(i) = color;
    }

    this->update();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::set_colors(const std::array<Color, num_of_leds>& colors) {
    this->colors = colors;
    this->update();
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::turn_off(uint8_t index) {
    this->set_color({0, 0, 0}, index);
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::turn_off() {
    this->set_color({0, 0, 0});
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::update() {
    if (this->pwm.is_busy()) {
        return;
    }

    for (uint8_t i = 0; i < num_of_leds; i++) {
        this->encode_color(this->colors.at(i) * this->brightness, i);
    }

    this->pwm.start_dma(this->buffer);
}

template <uint8_t num_of_leds>
void TArgb<num_of_leds>::encode_color(const Color& color, uint8_t index) {
    uint32_t data = (color.green << (2 * bits_per_color)) | (color.red << bits_per_color) | color.blue;

    for (uint32_t i = bits_per_led * index, j = bits_per_led - 1; i < bits_per_led * (index + 1U); i++, j--) {
        this->buffer.at(i) = ((data >> j) & 1) == 1 ? this->high_bit : this->low_bit;
    }
}
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_ARGB_CPP
