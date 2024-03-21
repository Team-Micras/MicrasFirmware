/**
 * @file dual_motor_driver.hpp
 *
 * @brief Proxy DualMotorDriver class declaration
 *
 * @date 03/2024
 */

#ifndef __DUAL_MOTOR_DRIVER_HPP__
#define __DUAL_MOTOR_DRIVER_HPP__

#include <cstdint>

#include "hal/pwm.hpp"
#include "hal/gpio.hpp"

namespace proxy {
/**
 * @brief Class for controlling a dual motor driver
 */
class DualMotorDriver {
    public:
        /**
         * @brief Configuration structure for the dual motor driver
         */
        struct Config {
            hal::Pwm::Config  pwm_left_fwd;
            hal::Pwm::Config  pwm_left_bwd;
            hal::Pwm::Config  pwm_right_fwd;
            hal::Pwm::Config  pwm_right_bwd;
            hal::Gpio::Config enable_gpio;
        };

        /**
         * @brief Construct a new Dual Motor Driver object
         *
         * @param dual_motor_driver_config
         */
        DualMotorDriver(const Config& dual_motor_driver_config);

        /**
         * @brief Enable the motor driver
         */
        void enable();

        /**
         * @brief Disable the motor driver
         */
        void disable();

        /**
         * @brief Set the speed of the motors
         *
         * @param left_speed Speed of the left motor
         * @param right_speed Speed of the right motor
         */
        void set_speed(float left_speed, float right_speed);

        /**
         * @brief Stop the motors
         */
        void stop();

    private:
        /**
         * @brief Stop the left motor
         */
        void stop_left_motor();

        /**
         * @brief Stop the right motor
         */
        void stop_right_motor();

        /**
         * @brief PWM handle for the left motor forward
         */
        hal::Pwm pwm_left_fwd;

        /**
         * @brief PWM handle for the left motor backward
         */
        hal::Pwm pwm_left_bwd;

        /**
         * @brief PWM handle for the right motor forward
         */
        hal::Pwm pwm_right_fwd;

        /**
         * @brief PWM handle for the right motor backward
         */
        hal::Pwm pwm_right_bwd;

        /**
         * @brief GPIO handle for the motor driver enable pin
         */
        hal::Gpio enable_gpio;
};
}  // namespace proxy

#endif // __DUAL_MOTOR_DRIVER_HPP__
