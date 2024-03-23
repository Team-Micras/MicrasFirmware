/**
 * @file target.cpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#include "target.hpp"

proxy::Battery::Config battery_config = {
    .adc = {
        .handle = &hadc3,
        .init_function = MX_ADC3_Init
    }
};

proxy::Button::Config button_config = {
    .gpio = {
        .port = GPIOA,
        .pin = GPIO_PIN_12
    },
    .pull_resistor = proxy::Button::PullResistor::PULL_UP
};

proxy::Buzzer::Config buzzer_config = {
    .pwm = {
        .handle = &htim4,
        .init_function = MX_TIM4_Init,
        .timer_channel = TIM_CHANNEL_1
    }
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

proxy::DistanceSensors<4>::Config distance_sensors_config = {
    .adc = {
        .handle = &hadc1,
        .init_function = MX_ADC1_Init
    },
    .led_pwm = {
        .handle = &htim15,
        .init_function = MX_TIM15_Init,
        .timer_channel = TIM_CHANNEL_1
    }
};

proxy::Fan::Config fan_config = {
    .pwm = {
        .handle = &htim17,
        .init_function = MX_TIM17_Init,
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

proxy::Led::Config led_config = {
    .gpio = {
        .port = GPIOA,
        .pin = GPIO_PIN_15
    }
};

proxy::Locomotion::Config locomotion_config = {
    .pwm_left_fwd = {
        .handle = &htim4,
        .init_function = MX_TIM4_Init,
        .timer_channel = TIM_CHANNEL_4
    },
    .pwm_left_bwd = {
        .handle = &htim4,
        .init_function = MX_TIM4_Init,
        .timer_channel = TIM_CHANNEL_3
    },
    .pwm_right_fwd = {
        .handle = &htim1,
        .init_function = MX_TIM1_Init,
        .timer_channel = TIM_CHANNEL_2
    },
    .pwm_right_bwd = {
        .handle = &htim1,
        .init_function = MX_TIM1_Init,
        .timer_channel = TIM_CHANNEL_1
    },
    .enable_gpio = {
        .port = GPIOA,
        .pin = GPIO_PIN_10
    }
};

proxy::RotarySensor::Config rotary_sensor_left_config = {
    .spi = {
        .handle = &hspi1,
        .init_function = MX_SPI1_Init,
        .gpio = {
            .port = GPIOA,
            .pin = GPIO_PIN_6
        }
    },
    .encoder = {
        .handle = &htim2,
        .init_function = MX_TIM2_Init,
        .timer_channel = TIM_CHANNEL_ALL
    },
    .resolution = 4096
};

proxy::RotarySensor::Config rotary_sensor_right_config = {
    .spi = {
        .handle = &hspi1,
        .init_function = MX_SPI1_Init,
        .gpio = {
            .port = GPIOB,
            .pin = GPIO_PIN_1
        }
    },
    .encoder = {
        .handle = &htim5,
        .init_function = MX_TIM5_Init,
        .timer_channel = TIM_CHANNEL_ALL
    },
    .resolution = 4096
};

proxy::TorqueSensors<2>::Config torque_sensors_config = {
    .current_sensors = {
        .adc = {
            .handle = &hadc2,
            .init_function = MX_ADC2_Init
        },
        .shunt_resistor = 0.04f
    }
};
