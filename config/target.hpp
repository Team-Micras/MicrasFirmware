/**
 * @file
 */

#ifndef MICRAS_TARGET_HPP
#define MICRAS_TARGET_HPP

#include <main.h>

#include "micras/proxy/argb.hpp"
#include "micras/proxy/battery.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/led.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/storage.hpp"
#include "micras/proxy/torque_sensors.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras {
/*****************************************
 * Interface
 *****************************************/

const proxy::Led::Config led_config = {
    .gpio =
        {
            .port = LED_RED_GPIO_Port,
            .pin = LED_RED_Pin,
        },
};

const proxy::Argb<2>::Config argb_config = {
    .pwm =
        {
            .init_function = MX_TIM8_Init,
            .handle = &htim8,
            .timer_channel = TIM_CHANNEL_1,
        },
    .low_duty_cycle = 32.0F,
    .high_duty_cycle = 64.0F,
    .max_brightness = 10.0F,
};

const proxy::Button::Config button_config = {
    .gpio =
        {
            .port = Button_GPIO_Port,
            .pin = Button_Pin,
        },
    .pull_resistor = proxy::Button::PullResistor::PULL_UP,
};

const proxy::DipSwitch<4>::Config dip_switch_config = {
    .gpio_array = {{
        {
            .port = Switch_0_GPIO_Port,
            .pin = Switch_0_Pin,
        },
        {
            .port = Switch_1_GPIO_Port,
            .pin = Switch_1_Pin,
        },
        {
            .port = Switch_2_GPIO_Port,
            .pin = Switch_2_Pin,
        },
        {
            .port = Switch_3_GPIO_Port,
            .pin = Switch_3_Pin,
        },
    }}
};

const proxy::Buzzer::Config buzzer_config = {
    .pwm =
        {
            .init_function = MX_TIM4_Init,
            .handle = &htim4,
            .timer_channel = TIM_CHANNEL_1,
        },
};

/*****************************************
 * Sensors
 *****************************************/

const proxy::RotarySensor::Registers rotary_sensor_reg_config = {
    .disable = {{
        .UVW_off = 1,
        .ABI_off = 0,
        .na = 0,
        .FILTER_disable = 0,
    }},
    .zposm = {{
        .ZPOSM = 0,
    }},
    .zposl = {{
        .ZPOSL = 0,
        .Dia1_en = 0,
        .Dia2_en = 0,
    }},
    .settings1 = {{
        .K_max = 0,
        .K_min = 0,
        .Dia3_en = 0,
        .Dia4_en = 0,
    }},
    .settings2 = {{
        .IWIDTH = 0,
        .NOISESET = 0,
        .DIR = 0,
        .UVW_ABI = 0,
        .DAECDIS = 0,
        .ABI_DEC = 0,
        .Data_select = 0,
        .PWMon = 0,
    }},
    .settings3 = {{
        .UVWPP = 0,
        .HYS = 0,
        .ABIRES = 0b100,
    }},
    .ecc = {{
        .ECC_chsum = 0,
        .ECC_en = 0,
    }},
};

const proxy::RotarySensor::Config rotary_sensor_left_config = {
    .spi =
        {
            .init_function = MX_SPI1_Init,
            .handle = &hspi1,
            .cs_gpio =
                {
                    .port = Encoder_Left_CSn_GPIO_Port,
                    .pin = Encoder_Left_CSn_Pin,
                },
            .timeout = 2,
        },
    .encoder =
        {
            .init_function = MX_TIM2_Init,
            .handle = &htim2,
            .timer_channel = TIM_CHANNEL_ALL,
        },
    .crc =
        {
            .handle = &hcrc,
        },
    .resolution = 4096,
    .registers = rotary_sensor_reg_config,
};

const proxy::RotarySensor::Config rotary_sensor_right_config = {
    .spi =
        {
            .init_function = MX_SPI1_Init,
            .handle = &hspi1,
            .cs_gpio =
                {
                    .port = Encoder_Right_CSn_GPIO_Port,
                    .pin = Encoder_Right_CSn_Pin,
                },
            .timeout = 2,
        },
    .encoder =
        {
            .init_function = MX_TIM5_Init,
            .handle = &htim5,
            .timer_channel = TIM_CHANNEL_ALL,
        },
    .crc =
        {
            .handle = &hcrc,
        },
    .resolution = 4096,
    .registers = rotary_sensor_reg_config,
};

const proxy::TorqueSensors<2>::Config torque_sensors_config = {
    .adc =
        {
            .init_function = MX_ADC2_Init,
            .handle = &hadc2,
            .max_reading = 4095,
        },
    .shunt_resistor = 0.04F * 20,
    .max_torque = 10.0F,
    .filter_cutoff = 10.0F,
};

const proxy::WallSensors<4>::Config wall_sensors_config = {
    .adc =
        {
            .init_function = MX_ADC1_Init,
            .handle = &hadc1,
            .max_reading = 4095,
        },
    .led_0_pwm =
        {
            .init_function = MX_TIM15_Init,
            .handle = &htim15,
            .timer_channel = TIM_CHANNEL_1,
        },
    .led_1_pwm =
        {
            .init_function = MX_TIM15_Init,
            .handle = &htim15,
            .timer_channel = TIM_CHANNEL_2,
        },
    .filter_cutoff = 5.0F,
    .uncertainty = 0.35F,
    .wall_threshold =
        {
            0.42418F,
            0.0961F,
            0.09384F,
            0.31886F,
        },
    .free_threshold =
        {
            0.39012F,
            0.0881F,
            0.08916F,
            0.28704F,
        },
};

const proxy::Imu::Config imu_config = {
    .spi =
        {
            .init_function = MX_SPI1_Init,
            .handle = &hspi1,
            .cs_gpio =
                {
                    .port = IMU_SPI_CSn_GPIO_Port,
                    .pin = IMU_SPI_CSn_Pin,
                },
            .timeout = 2,
        },
    .gyroscope_data_rate = LSM6DSV_ODR_AT_960Hz,
    .accelerometer_data_rate = LSM6DSV_ODR_AT_960Hz,
    .orientation_data_rate = LSM6DSV_SFLP_480Hz,
    .gyroscope_scale = LSM6DSV_4000dps,
    .accelerometer_scale = LSM6DSV_8g,
    .gyroscope_filter = LSM6DSV_GY_ULTRA_LIGHT,
    .accelerometer_filter = LSM6DSV_XL_MEDIUM
};

const proxy::Battery::Config battery_config = {
    .adc =
        {
            .init_function = MX_ADC3_Init,
            .handle = &hadc3,
            .max_reading = 4095,
        },
    .voltage_divider = 3.0F,
    .filter_cutoff = 5.0F,
};

const hal::Timer::Config timer_config = {
    .init_function = MX_TIM6_Init,
    .handle = &htim6,
};

/*****************************************
 * Actuators
 *****************************************/

const proxy::Fan::Config fan_config = {
    .pwm =
        {
            .init_function = MX_TIM17_Init,
            .handle = &htim17,
            .timer_channel = TIM_CHANNEL_1,
        },
    .direction_gpio =
        {
            .port = Fan_Direction_GPIO_Port,
            .pin = Fan_Direction_Pin,
        },
    .enable_gpio =
        {
            .port = Fan_Enable_GPIO_Port,
            .pin = Fan_Enable_Pin,
        },
    .max_acceleration = 0.02F,
};

const proxy::Locomotion::Config locomotion_config = {
    .left_motor =
        {
            .backwards_pwm =
                {
                    .init_function = MX_TIM3_Init,
                    .handle = &htim3,
                    .timer_channel = TIM_CHANNEL_4,
                },
            .forward_pwm =
                {
                    .init_function = MX_TIM3_Init,
                    .handle = &htim3,
                    .timer_channel = TIM_CHANNEL_3,
                },
            .max_stopped_command = 1.0F,
            .deadzone = 33.0F,
        },
    .right_motor =
        {
            .backwards_pwm =
                {
                    .init_function = MX_TIM1_Init,
                    .handle = &htim1,
                    .timer_channel = TIM_CHANNEL_2,
                },
            .forward_pwm =
                {
                    .init_function = MX_TIM1_Init,
                    .handle = &htim1,
                    .timer_channel = TIM_CHANNEL_1,
                },
            .max_stopped_command = 1.0F,
            .deadzone = 30.0F,
        },
    .enable_gpio =
        {
            .port = Motors_Enable_GPIO_Port,
            .pin = Motors_Enable_Pin,
        },
};
}  // namespace micras

#endif  //  MICRAS_TARGET_HPP
