#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <cstdint>

#include "proxy/button.hpp"
#include "proxy/led.hpp"
#include "proxy/distance_sensors.hpp"
#include "proxy/current_sensors.hpp"
#include "proxy/dip_switch.hpp"
#include "proxy/buzzer.hpp"
#include "proxy/motor_driver.hpp"
#include "proxy/dual_motor_driver.hpp"
#include "proxy/imu.hpp"

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

proxy::DistanceSensors<4>::Config distance_sensors_config = {
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

proxy::CurrentSensors<2>::Config current_sensors_config = {
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

proxy::Buzzer::Config buzzer_config = {
    .pwm = {
        .timer = {
            .handle = &htim4,
            .init_function = MX_TIM4_Init
        },
        .timer_channel = TIM_CHANNEL_1
    }
};

proxy::MotorDriver::Config vent_motor_driver_config = {
    .pwm = {
        .timer = {
            .handle = &htim17,
            .init_function = MX_TIM17_Init
        },
        .timer_channel = TIM_CHANNEL_1
    },
    .direction_gpio = {
        .port = GPIOC,
        .pin = GPIO_PIN_13
    },
    .enable_gpio = {
        .port = GPIOB,
        .pin = GPIO_PIN_7
    }
};

proxy::DualMotorDriver::Config loc_motor_driver_config = {
    .pwm_left_fwd = {
        .timer = {
            .handle = &htim4,
            .init_function = MX_TIM4_Init
        },
        .timer_channel = TIM_CHANNEL_4
    },
    .pwm_left_bwd = {
        .timer = {
            .handle = &htim4,
            .init_function = MX_TIM4_Init
        },
        .timer_channel = TIM_CHANNEL_3
    },
    .pwm_right_fwd = {
        .timer = {
            .handle = &htim1,
            .init_function = MX_TIM1_Init
        },
        .timer_channel = TIM_CHANNEL_2
    },
    .pwm_right_bwd = {
        .timer = {
            .handle = &htim1,
            .init_function = MX_TIM1_Init
        },
        .timer_channel = TIM_CHANNEL_1
    },
    .enable_gpio = {
        .port = GPIOA,
        .pin = GPIO_PIN_10
    }
};

proxy::Imu::Config imu_config = {
    .spi = {
        .handle = &hspi1,
        .init_function = MX_SPI1_Init,
        .gpio = {
            .port = GPIOB,
            .pin = GPIO_PIN_6
        }
    }
};

#endif // __TARGET_HPP__
