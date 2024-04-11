/**
 * @file fan.hpp
 *
 * @brief Proxy Fan class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_FAN_HPP
#define MICRAS_PROXY_FAN_HPP

#include <cstdint>

#include "micras/hal/gpio.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the fan driver
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
     * @brief Construct a new fan object
     *
     * @param config Configuration for the fan driver
     */
    explicit Fan(const Config& config);

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
     * @brief Enum for rotation direction
     */
    enum RotationDirection : uint8_t {
        FORWARD,
        BACKWARD
    };

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
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_FAN_HPP
