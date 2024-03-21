/**
 * @file fan.hpp
 *
 * @brief Proxy Fan class declaration
 *
 * @date 03/2024
 */

#ifndef __FAN_HPP__
#define __FAN_HPP__

#include <cstdint>

#include "hal/pwm.hpp"
#include "hal/gpio.hpp"

namespace proxy {
/**
 * @brief Class for controlling the fan
 */
class Fan {
    public:
        /**
         * @brief Configuration structure for the fan
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
         * @brief Construct a new fan object
         *
         * @param fan_config
         */
        Fan(Config& fan_config);

        /**
         * @brief Enable the fan
         */
        void enable();

        /**
         * @brief Disable the fan
         */
        void disable();

        /**
         * @brief Set the speed of the fans
         *
         * @param speed Speed percentage of the fan
         */
        void set_speed(float speed);

        /**
         * @brief Stop the fan
         */
        void stop();

    private:
        /**
         * @brief Set the rotation direction of the fan
         *
         * @param direction Rotation direction
         */
        void set_direction(RotationDirection direction);

        /**
         * @brief PWM object for controlling the fan speed
         */
        hal::Pwm pwm;

        /**
         * @brief GPIO object for controlling the fan rotation direction
         */
        hal::Gpio direction_gpio;

        /**
         * @brief GPIO handle for the fan enable pin
         */
        hal::Gpio enable_gpio;
};
}  // namespace proxy

#endif // __FAN_HPP__
