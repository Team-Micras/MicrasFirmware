/**
 * @file button.cpp
 *
 * @brief Proxy Button class source
 *
 * @date 03/2024
 */

#include "hal/mcu.hpp"
#include "proxy/button.hpp"

namespace proxy {
Button::Button(const Config& button_config) :
    debounce_delay{button_config.debounce_delay},
    long_press_delay{button_config.long_press_delay},
    extra_long_press_delay{button_config.extra_long_press_delay},
    gpio{button_config.gpio},
    pull_resistor{button_config.pull_resistor} {
}

bool Button::is_pressed() {
    this->update_state();
    return this->current_state;
}

Button::Status Button::get_status() {
    this->update_state();

    if (this->is_rising_edge()) {
        hal::mcu::reset_timer(this->status_timer);
    } else if (this->is_falling_edge()) {
        if (hal::mcu::get_timer_ms(this->status_timer) > extra_long_press_delay) {
            return EXTRA_LONG_PRESS;
        } else if (hal::mcu::get_timer_ms(this->status_timer) > long_press_delay) {
            return LONG_PRESS;
        } else {
            return SHORT_PRESS;
        }
    }

    return NO_PRESS;
}

bool Button::get_raw_reading() const {
    return this->gpio.read() == this->pull_resistor;
}

void Button::update_state() {
    bool raw_reading = this->get_raw_reading();

    if ((raw_reading != this->current_state) and not this->is_debouncing) {
        this->is_debouncing = true;
        hal::mcu::reset_timer(this->debounce_timer);
    } else if ((hal::mcu::get_timer_ms(this->debounce_timer) < debounce_delay) and this->is_debouncing) {
        if (this->current_state == raw_reading) {
            this->is_debouncing = false;
        }
    } else {
        this->is_debouncing = false;
        this->previous_state = this->current_state;
        this->current_state = raw_reading;
    }
}

bool Button::is_rising_edge() const {
    return this->current_state and not this->previous_state;
}

bool Button::is_falling_edge() const {
    return not this->current_state and this->previous_state;
}
}  // namespace proxy
