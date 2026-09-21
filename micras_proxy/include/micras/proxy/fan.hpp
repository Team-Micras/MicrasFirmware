/**
 * @file
 */

#ifndef MICRAS_PROXY_FAN_HPP
#define MICRAS_PROXY_FAN_HPP

#include "micras/hal/gpio.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the fan driver.
 *
 * @note The driver on this board has its two half bridges paralleled and its phase input strapped,
 * so the drive is unidirectional by construction and there is no direction pin to control.
 */
class Fan {
public:
    /**
     * @brief Configuration struct for the fan.
     */
    struct Config {
        hal::Pwm::Config  pwm;
        hal::Gpio::Config enable_gpio;
        float             max_acceleration;
    };

    /**
     * @brief Construct a new fan object.
     *
     * @param config Configuration for the fan driver.
     */
    explicit Fan(const Config& config);

    /**
     * @brief Enable the fan.
     */
    void enable();

    /**
     * @brief Disable the fan.
     */
    void disable();

    /**
     * @brief Set the speed of the fans.
     *
     * @param speed Speed percentage of the fan.
     */
    void set_speed(float speed);

    /**
     * @brief Update the speed of the fan.
     *
     * @return Current speed percentage of the fan.
     */
    float update();

    /**
     * @brief Stop the fan and cancel any speed it was ramping towards.
     */
    void stop();

    /**
     * @brief Check if the driver is reporting a fault.
     *
     * @note The enable pin is bidirectional: the driver pulls it low to signal an over current or a
     * thermal shutdown, which is only observable while the fan is meant to be enabled and the pin
     * is configured as an open drain output.
     *
     * @return True if the driver is pulling the enable pin low against the firmware, false otherwise.
     */
    bool check_fault() const;

    /**
     * @brief Check if the fan PWM was successfully initialized.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief PWM object for controlling the fan speed.
     */
    hal::Pwm pwm;

    /**
     * @brief GPIO handle for the fan enable and fault pin.
     */
    hal::Gpio enable_gpio;

    /**
     * @brief Current speed of the fan.
     */
    float current_speed{};

    /**
     * @brief Target speed of the fan.
     */
    float target_speed{};

    /**
     * @brief Maximum acceleration of the fan in percentage per millisecond.
     */
    float max_acceleration;

    /**
     * @brief Whether the firmware is currently asking the driver to be enabled.
     */
    bool enabled{};

    /**
     * @brief Stopwatch for limiting the acceleration of the fan.
     */
    proxy::Stopwatch acceleration_stopwatch;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_FAN_HPP
