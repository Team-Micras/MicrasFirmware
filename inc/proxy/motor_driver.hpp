/**
 * @file motor_driver.hpp
 *
 * @brief Proxy MotorDriver class declaration
 *
 * @date 03/2024
 */

#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#include <cstdint>

#include "hal/pwm.hpp"
#include "hal/gpio.hpp"

namespace proxy {
/**
 * @brief Class for controlling a motor driver
 */
class MotorDriver {
    public:
        /**
         * @brief Configuration structure for the motor driver
         */
        struct Config {
            hal::Pwm::Config  pwm;
            hal::Gpio::Config direction_gpio;
            hal::Gpio::Config enable_gpio;
        };

        /**
         * @brief Enum for rotation direction
         */
        enum RotationDirection {
            FORWARD,
            BACKWARD
        };

        /**
         * @brief Construct a new  Motor Driver object
         *
         * @param motor_driver_config
         */
        MotorDriver(Config& motor_driver_config);

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
         * @param speed Speed percentage of the motor
         */
        void set_speed(float speed);

        /**
         * @brief Stop the motors
         */
        void stop();

    private:
        /**
         * @brief Set the rotation direction of the motor
         *
         * @param direction Rotation direction
         */
        void set_direction(RotationDirection direction);

        /**
         * @brief PWM object for controlling the motor speed
         */
        hal::Pwm pwm;

        /**
         * @brief GPIO object for controlling the motor rotation direction
         */
        hal::Gpio direction_gpio;

        /**
         * @brief GPIO handle for the motor driver enable pin
         */
        hal::Gpio enable_gpio;
};
}  // namespace proxy

#endif // __MOTOR_DRIVER_HPP__
