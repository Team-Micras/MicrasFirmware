/**
 * @file target.cpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#include "target.hpp"

proxy::Argb<2>::Config argb_config = {
    .pwm = {
        .handle = &htim8,
        .init_function = MX_TIM8_Init,
        .timer_channel = TIM_CHANNEL_1
    }
};

proxy::Battery::Config battery_config = {
    .adc = {
        .handle = &hadc3,
        .init_function = MX_ADC3_Init,
        .max_reading = 4095,
        .reference_voltage = 3.0F
    },
    .voltage_divider = 3.0F
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
        .init_function = MX_ADC1_Init,
        .max_reading = 4095,
        .reference_voltage = 3.3F
    },
    .led_pwm = {
        .handle = &htim15,
        .init_function = MX_TIM15_Init,
        .timer_channel = TIM_CHANNEL_1
    },
    .max_distance = 0.3F
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
        },
        .timeout = 2
    },
    .gyroscope_data_rate = LSM6DSV_ODR_AT_480Hz,
    .accelerometer_data_rate = LSM6DSV_ODR_AT_480Hz,
    .orientation_data_rate = LSM6DSV_SFLP_480Hz,
    .gyroscope_scale = LSM6DSV_4000dps,
    .accelerometer_scale = LSM6DSV_8g,
    .gyroscope_filter = LSM6DSV_GY_ULTRA_LIGHT,
    .accelerometer_filter = LSM6DSV_XL_ULTRA_LIGHT
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

proxy::RotarySensor::Registers rotary_sensor_reg_config = {
    .disable = {{
        .UVW_off = 1,
        .ABI_off = 0,
        .na = 0,
        .FILTER_disable = 0
    }},
    .zposm = {{
        .ZPOSM = 0
    }},
    .zposl = {{
        .ZPOSL = 0,
        .Dia1_en = 0,
        .Dia2_en = 0
    }},
    .settings1 = {{
        .K_max = 0,
        .K_min = 0,
        .Dia3_en = 0,
        .Dia4_en = 0
    }},
    .settings2 = {{
        .IWIDTH = 0,
        .NOISESET = 0,
        .DIR = 0,
        .UVW_ABI = 0,
        .DAECDIS = 0,
        .ABI_DEC = 0,
        .Data_select = 0,
        .PWMon = 0
    }},
    .settings3 = {{
        .UVWPP = 0,
        .HYS = 0,
        .ABIRES = 0b100
    }},
    .ecc = {{
        .ECC_chsum = 0,
        .ECC_en = 0
    }}
};

proxy::RotarySensor::Config rotary_sensor_left_config = {
    .spi = {
        .handle = &hspi1,
        .init_function = MX_SPI1_Init,
        .gpio = {
            .port = GPIOA,
            .pin = GPIO_PIN_6
        },
        .timeout = 2
    },
    .encoder = {
        .handle = &htim2,
        .init_function = MX_TIM2_Init,
        .timer_channel = TIM_CHANNEL_ALL
    },
    .crc = {
        .handle = &hcrc
    },
    .resolution = 4096,
    .registers = rotary_sensor_reg_config
};

proxy::RotarySensor::Config rotary_sensor_right_config = {
    .spi = {
        .handle = &hspi1,
        .init_function = MX_SPI1_Init,
        .gpio = {
            .port = GPIOB,
            .pin = GPIO_PIN_1
        },
        .timeout = 2
    },
    .encoder = {
        .handle = &htim5,
        .init_function = MX_TIM5_Init,
        .timer_channel = TIM_CHANNEL_ALL
    },
    .crc = {
        .handle = &hcrc
    },
    .resolution = 4096,
    .registers = rotary_sensor_reg_config
};

proxy::TorqueSensors<2>::Config torque_sensors_config = {
    .current_sensors = {
        .adc = {
            .handle = &hadc2,
            .init_function = MX_ADC2_Init,
            .max_reading = 4095,
            .reference_voltage = 3.3F
        },
        .shunt_resistor = 0.04F
    },
    .max_torque = 0.5F
};
