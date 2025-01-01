/**
 * @file
 */

#ifndef MICRAS_PROXY_LOCOMOTION_HPP
#define MICRAS_PROXY_LOCOMOTION_HPP

#include <cstdint>

#include "micras/hal/gpio.hpp"
#include "micras/proxy/motor.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the locomotion driver.
 */
class Locomotion {
public:
    /**
     * @brief Configuration struct for the locomotion.
     */
    struct Config {
        Motor::Config     left_motor;
        Motor::Config     right_motor;
        hal::Gpio::Config enable_gpio;
    };

    /**
     * @brief Construct a new locomotion object.
     *
     * @param config Configuration for the locomotion driver.
     */
    explicit Locomotion(const Config& config);

    /**
     * @brief Enable the locomotion driver.
     */
    void enable();

    /**
     * @brief Disable the locomotion driver.
     */
    void disable();

    /**
     * @brief Set the command of the wheels.
     *
     * @param left_command Command of the left wheels.
     * @param right_command Command of the right wheels.
     */
    void set_wheel_command(float left_command, float right_command);

    /**
     * @brief Set the linear and angular commands of the robot.
     *
     * @param linear Linear command of the robot.
     * @param angular Angular command of the robot.
     */
    void set_command(float linear, float angular);

    /**
     * @brief Stop the motors.
     */
    void stop();

private:
    /**
     * @brief Left motor of the robot.
     */
    Motor left_motor;

    /**
     * @brief Right motor of the robot.
     */
    Motor right_motor;

    /**
     * @brief GPIO handle for the motor driver enable pin.
     */
    hal::Gpio enable_gpio;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LOCOMOTION_HPP
