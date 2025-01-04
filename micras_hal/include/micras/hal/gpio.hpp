/**
 * @file
 */

#ifndef MICRAS_HAL_GPIO_HPP
#define MICRAS_HAL_GPIO_HPP

#include <cstdint>
#include <gpio.h>

namespace micras::hal {
/**
 * @brief Class to handle GPIO pins on STM32 microcontrollers.
 */
class Gpio {
public:
    /**
     * @brief Configuration struct for GPIO pin.
     */
    struct Config {
        GPIO_TypeDef* port;
        uint16_t      pin;
    };

    /**
     * @brief Construct a new Gpio object.
     *
     * @param config Configuration for the GPIO pin.
     */
    explicit Gpio(const Config& config);

    /**
     * @brief Read the current state of the GPIO pin.
     *
     * @return True if the current state of the GPIO pin is high, false otherwise.
     */
    bool read() const;

    /**
     * @brief Write a new state to the GPIO pin.
     *
     * @param pin_state The state to be written (true for high, false for low).
     */
    void write(bool state);

    /**
     * @brief Toggle the state of the GPIO pin.
     */
    void toggle();

private:
    /**
     * @brief The port of the GPIO.
     */
    GPIO_TypeDef* port;

    /**
     * @brief The pin number of the GPIO.
     */
    uint16_t pin;
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_GPIO_HPP
