/**
 * @file
 */

#ifndef MICRAS_TARGETS_V1_HPP
#define MICRAS_TARGETS_V1_HPP

#include <array>
#include <main.h>

#include "constants.hpp"
#include "micras/hal/fmac.hpp"
#include "micras/hal/gpio.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/proxy/argb.hpp"
#include "micras/proxy/battery.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/fmac_filter.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/led.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/storage.hpp"
#include "micras/proxy/torque_sensors.hpp"
#include "micras/proxy/wall_sensors.hpp"

/**
 * @brief Configuration of the Micras v1 mainboard, built around an STM32H725RGV.
 */
namespace micras {
/*****************************************
 * Template Instantiations
 *****************************************/

namespace proxy {
using Argb = proxy::TArgb<2>;
using DipSwitch = TDipSwitch<4>;
using TorqueSensors = TTorqueSensors<2>;
using WallSensors = TWallSensors<4>;
}  // namespace proxy

/*****************************************
 * Board properties
 *****************************************/

/**
 * @brief Analogue supply of the microcontroller, which is also the ADC reference.
 *
 * @note The package has no separate reference pin, so every analogue reading is relative to this.
 */
constexpr float adc_reference_voltage{3.3F};

/**
 * @brief SPI modes of the devices sharing SPI3.
 *
 * @note The inertial measurement unit wants mode 3 and the magnetic encoders want mode 1, so the
 * mode belongs to the device and the bus is reconfigured when the selected device changes.
 */
///@{
constexpr uint32_t imu_clock_polarity{SPI_POLARITY_HIGH};
constexpr uint32_t imu_clock_phase{SPI_PHASE_2EDGE};
constexpr uint32_t rotary_sensor_clock_polarity{SPI_POLARITY_LOW};
constexpr uint32_t rotary_sensor_clock_phase{SPI_PHASE_2EDGE};
///@}

/*****************************************
 * Internal
 *****************************************/

const proxy::Storage::Config maze_storage_config{
    .start_sector = 2,
    .number_of_sectors = 1,
};

const hal::Fmac::Config fmac_config{
    .init_function = MX_FMAC_Init,
    .handle = &hfmac,
};

/*****************************************
 * Interface
 *****************************************/

const proxy::Led::Config led_config = {
    .gpio = {
        .port = LED_Red_GPIO_Port,
        .pin = LED_Red_Pin,
    },
};

const proxy::Argb::Config argb_config = {
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

const proxy::DipSwitch::Config dip_switch_config = {
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
    }},
    // The common side of the switch package is grounded and every pin has an internal pull up, so
    // a closed switch reads low
    .active_low = true,
};

const proxy::Buzzer::Config buzzer_config = {
    .pwm = {
        .init_function = MX_TIM15_Init,
        .handle = &htim15,
        .timer_channel = TIM_CHANNEL_1,
    },
};

/*****************************************
 * Sensors
 *****************************************/

/**
 * @brief Configuration written to the volatile registers of the magnetic encoders.
 *
 * @note ABIRES selects the pulses per revolution of the quadrature output. The proxy reads this
 * field back after writing it and derives its scale factor from what the sensor reports, so this
 * value cannot silently disagree with the odometry.
 */
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
            .init_function = MX_SPI3_Init,
            .handle = &hspi3,
            .cs_gpio =
                {
                    .port = Encoder_Left_CSn_GPIO_Port,
                    .pin = Encoder_Left_CSn_Pin,
                },
            .timeout = 2,
            .clock_polarity = rotary_sensor_clock_polarity,
            .clock_phase = rotary_sensor_clock_phase,
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
    .registers = rotary_sensor_reg_config,
};

const proxy::RotarySensor::Config rotary_sensor_right_config = {
    .spi =
        {
            .init_function = MX_SPI3_Init,
            .handle = &hspi3,
            .cs_gpio =
                {
                    .port = Encoder_Right_CSn_GPIO_Port,
                    .pin = Encoder_Right_CSn_Pin,
                },
            .timeout = 2,
            .clock_polarity = rotary_sensor_clock_polarity,
            .clock_phase = rotary_sensor_clock_phase,
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
    .registers = rotary_sensor_reg_config,
};

const proxy::TorqueSensors::Config torque_sensors_config = {
    .adc =
        {
            .init_function = MX_ADC2_Init,
            .handle = &hadc2,
            .max_reading = 65535,
            .reference_voltage = adc_reference_voltage,
        },
    // 40 mOhm shunts into current sense amplifiers of gain 20
    .shunt_resistor = 0.04F * 20,
    // Full scale current times the torque constant of the motor. The torque constant has not been
    // measured on these motors, so this is an order of magnitude estimate for a coreless
    // micromouse motor and wants a bench calibration before nav relies on the value.
    .max_torque = 0.01F,
    .filter = {
        .cutoff_frequency = torque_filter_cutoff,
        .sampling_frequency = loop_frequency,
    },
};

const proxy::WallSensors::Config wall_sensors_config = {
    .adc =
        {
            .init_function = MX_ADC1_Init,
            .handle = &hadc1,
            .max_reading = 65535,
            .reference_voltage = adc_reference_voltage,
        },
    .led_pwms = {{
        {
            .init_function = MX_TIM4_Init,
            .handle = &htim4,
            .timer_channel = TIM_CHANNEL_1,
        },
        {
            .init_function = MX_TIM4_Init,
            .handle = &htim4,
            .timer_channel = TIM_CHANNEL_2,
        },
        {
            .init_function = MX_TIM4_Init,
            .handle = &htim4,
            .timer_channel = TIM_CHANNEL_3,
        },
        {
            .init_function = MX_TIM4_Init,
            .handle = &htim4,
            .timer_channel = TIM_CHANNEL_4,
        },
    }},
    .filter =
        {
            .cutoff_frequency = sensor_filter_cutoff,
            .sampling_frequency = loop_frequency,
        },
    .base_readings =
        {
            0.413F,
            0.161F,
            0.177F,
            0.230F,
        },
    .uncertainty = 0.5F,
};

const proxy::Imu::Config imu_config = {
    .spi =
        {
            .init_function = MX_SPI3_Init,
            .handle = &hspi3,
            .cs_gpio =
                {
                    .port = IMU_SPI_CSn_GPIO_Port,
                    .pin = IMU_SPI_CSn_Pin,
                },
            .timeout = 2,
            .clock_polarity = imu_clock_polarity,
            .clock_phase = imu_clock_phase,
        },
    .gyroscope_data_rate = LSM6DSV_ODR_AT_960Hz,
    .accelerometer_data_rate = LSM6DSV_ODR_AT_960Hz,
    .gyroscope_scale = LSM6DSV_4000dps,
    .accelerometer_scale = LSM6DSV_8g,
    .gyroscope_filter = LSM6DSV_GY_ULTRA_LIGHT,
    .accelerometer_filter = LSM6DSV_XL_MEDIUM,
    .calibration_filter = {
        .cutoff_frequency = sensor_filter_cutoff,
        .sampling_frequency = loop_frequency,
    },
};

const proxy::Battery::Config battery_config = {
    .adc =
        {
            .init_function = MX_ADC3_Init,
            .handle = &hadc3,
            .max_reading = 4095,
            .reference_voltage = adc_reference_voltage,
        },
    // The internal channel of this family taps the battery pin through a divider by four, which is
    // what makes a three cell pack measurable against a 3.3 V reference. A board that brought the
    // pack to a normal ADC input would put its real resistor ratio here instead.
    .voltage_divider = 4.0F,
    .filter = {
        .cutoff_frequency = sensor_filter_cutoff,
        .sampling_frequency = loop_frequency,
    },
};

/*****************************************
 * Actuators
 *****************************************/

/**
 * @brief Configuration of the fan driver.
 *
 * @note The driver has its outputs paralleled and its phase input strapped high on this board, so
 * there is no direction pin.
 */
const proxy::Fan::Config fan_config = {
    .pwm =
        {
            .init_function = MX_TIM12_Init,
            .handle = &htim12,
            .timer_channel = TIM_CHANNEL_2,
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
                    .timer_channel = TIM_CHANNEL_2,
                },
            .max_stopped_command = 0.2F,
            .deadzone = 15.0F,
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
            .max_stopped_command = 0.2F,
            .deadzone = 15.0F,
        },
    .enable_gpio = {
        .port = Motors_Enable_GPIO_Port,
        .pin = Motors_Enable_Pin,
    },
};

/*****************************************
 * Emergency stop
 *****************************************/

/**
 * @brief Every PWM output and driver enable pin, for the shutdown path.
 *
 * @note Built from the configurations above rather than written out again, so that a timer or a pin
 * that moves cannot leave the emergency stop pointing at the old one.
 */
///@{
const std::array<hal::Pwm::Config, 10> emergency_pwm_configs{{
    locomotion_config.left_motor.forward_pwm,
    locomotion_config.left_motor.backwards_pwm,
    locomotion_config.right_motor.forward_pwm,
    locomotion_config.right_motor.backwards_pwm,
    fan_config.pwm,
    buzzer_config.pwm,
    std::get<0>(wall_sensors_config.led_pwms),
    std::get<1>(wall_sensors_config.led_pwms),
    std::get<2>(wall_sensors_config.led_pwms),
    std::get<3>(wall_sensors_config.led_pwms),
}};

const std::array<hal::Gpio::Config, 2> emergency_gpio_configs{{
    locomotion_config.enable_gpio,
    fan_config.enable_gpio,
}};
///@}
}  // namespace micras

#endif  // MICRAS_TARGETS_V1_HPP
