/**
 * @file
 */

#include "micras/proxy/button.hpp"

namespace micras::proxy {
Button::Button(const Config& config) :
    debounce_delay{config.debounce_delay},
    long_press_delay{config.long_press_delay},
    extra_long_press_delay{config.extra_long_press_delay},
    gpio{config.gpio},
    pull_resistor{config.pull_resistor} { }

bool Button::is_pressed() {
    return this->update_state();
}

Button::Status Button::get_status() {
    this->previous_state = this->current_state;
    this->current_state = this->update_state();

    if (this->is_rising_edge()) {
        this->status_stopwatch.reset_ms();
    } else if (this->is_falling_edge()) {
        if (this->status_stopwatch.elapsed_time_ms() > extra_long_press_delay) {
            return EXTRA_LONG_PRESS;
        }

        if (this->status_stopwatch.elapsed_time_ms() > long_press_delay) {
            return LONG_PRESS;
        }

        return SHORT_PRESS;
    }

    return NO_PRESS;
}

bool Button::get_raw_reading() const {
    return this->gpio.read() == static_cast<bool>(this->pull_resistor);
}

bool Button::update_state() {
    const bool raw_reading = this->get_raw_reading();

    if ((raw_reading != this->current_state) and not this->is_debouncing) {
        this->is_debouncing = true;
        this->debounce_stopwatch.reset_ms();
    } else if ((this->debounce_stopwatch.elapsed_time_ms() < debounce_delay) and this->is_debouncing) {
        if (this->current_state == raw_reading) {
            this->is_debouncing = false;
        }
    } else {
        return raw_reading;
    }

    return this->current_state;
}

bool Button::is_rising_edge() const {
    return this->current_state and not this->previous_state;
}

bool Button::is_falling_edge() const {
    return not this->current_state and this->previous_state;
}
}  // namespace micras::proxy
