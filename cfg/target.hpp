#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <cstdint>

#include "proxy/button.hpp"
#include "proxy/led.hpp"
#include "proxy/distance_sensors.hpp"
#include "proxy/current_sensors.hpp"
#include "proxy/dip_switch.hpp"

proxy::Led::Config led_config = {
    .gpio = {
        .port = GPIOA,
        .pin = GPIO_PIN_15
    }
};

proxy::Button::Config button_config = {
    .gpio = {
        .port = GPIOA,
        .pin = GPIO_PIN_12
    },
    .pull_resistor = proxy::Button::PullResistor::PULL_UP
};

proxy::DistanceSensors<4>::Config distance_sensor_config = {
    .adc = {
        .handle = &hadc1,
        .init_function = MX_ADC1_Init
    },
    .infrared_pwm = {
        .timer = {
            .handle = &htim15,
            .init_function = MX_TIM15_Init
        },
        .timer_channel = TIM_CHANNEL_1
    }
};

proxy::CurrentSensors<2>::Config current_sensor_config = {
    .adc = {
        .handle = &hadc2,
        .init_function = MX_ADC2_Init
    },
    .shunt_resistor = 0.04f
};

proxy::DipSwitch<4>::Config dip_switch_config = {
    .gpio_array = {{
        {
            .port = GPIOC,
            .pin = GPIO_PIN_7
        },
        {
            .port = GPIOB,
            .pin = GPIO_PIN_15
        },
        {
            .port = GPIOB,
            .pin = GPIO_PIN_14
        },
        {
            .port = GPIOB,
            .pin = GPIO_PIN_13
        }
    }}
};

#endif // __TARGET_HPP__
