#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <cstdint>

#include "proxy/button.hpp"
#include "proxy/led.hpp"
#include "proxy/distance_sensor_array.hpp"
#include "proxy/current_sensor_array.hpp"

proxy::Led::Config led_config = {
    .gpio_config = {
        .port = GPIOA,
        .pin = GPIO_PIN_15
    }
};

proxy::Button::Config button_config = {
    .gpio_config = {
        .port = GPIOA,
        .pin = GPIO_PIN_12
    },
    .pull_resistor = proxy::Button::PULL_UP
};

proxy::DistanceSensorArray<4>::Config distance_sensor_config = {
    .sensors_adc_config = {
        .handle = &hadc1,
        .init_function = MX_ADC1_Init
    },
    .infrared_tim_config = {
        .handle = &htim2,
        .init_function = MX_TIM15_Init
    },
    .infrared_led_channel = TIM_CHANNEL_1
};

proxy::CurrentSensorArray<2>::Config current_sensor_config = {
    .adc_config = {
        .handle = &hadc2,
        .init_function = MX_ADC2_Init
    }
};

#endif // __TARGET_HPP__
