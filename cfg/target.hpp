#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <cstdint>

#include "proxy/button.hpp"
#include "proxy/led.hpp"

proxy::Led::Config led_config = {
    .gpio_config = {
        .port = hal::Gpio::Port::A,
        .pin = hal::Gpio::Pin::P1
    }
};

proxy::Button::Config button_config = {
    .gpio_config = {
        .port = hal::Gpio::Port::A,
        .pin = hal::Gpio::Pin::P0
    }
};

#endif // __TARGET_HPP__
