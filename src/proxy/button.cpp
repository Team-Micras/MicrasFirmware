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
    debounce_delay_ms(button_config.debounce_delay_ms),
    long_press_delay_ms(button_config.long_press_delay_ms),
    extra_long_press_delay_ms(button_config.extra_long_press_delay_ms),
    gpio(button_config.gpio_config),
    pull_resistor(button_config.pull_resistor) {
}

void Button::update_state() {
    bool raw_reading = this->get_raw_reading();

    if ((raw_reading != this->current_state) && !this->is_debouncing) {
        this->is_debouncing = true;
        hal::mcu::reset_timer(this->debounce_timer);
    } else if ((hal::mcu::get_timer_ms(this->debounce_timer) < debounce_delay_ms) && this->is_debouncing) {
        if (this->current_state == raw_reading) {
            this->is_debouncing = false;
        }
    } else {
        this->is_debouncing = false;
        this->previous_state = this->current_state;
        this->current_state = raw_reading;
    }
}

Button::Status Button::get_status() {
    this->update_state();

    if (this->is_rising_edge()) {
        hal::mcu::reset_timer(this->status_timer);
    } else if (this->is_falling_edge()) {
        if (hal::mcu::get_timer_ms(this->status_timer) > extra_long_press_delay_ms) {
            return EXTRA_LONG_PRESS;
        } else if (hal::mcu::get_timer_ms(this->status_timer) > long_press_delay_ms) {
            return LONG_PRESS;
        } else {
            return SHORT_PRESS;
        }
    }

    return NO_PRESS;
}

bool Button::is_pressed() {
    this->update_state();
    return (this->current_state == true);
}

bool Button::is_released() {
    this->update_state();
    return (this->current_state == false);
}

bool Button::is_rising_edge() const {
    return (this->current_state == true && this->previous_state == false);
}

bool Button::is_falling_edge() const {
    return (this->current_state == false && this->previous_state == true);
}

bool Button::get_raw_reading() const {
    return (this->gpio.read() == (bool) this->pull_resistor);
}
}  // namespace proxy
