/**
 * @file locomotion.hpp
 *
 * @brief Proxy Locomotion class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_LOCOMOTION_HPP
#define MICRAS_PROXY_LOCOMOTION_HPP

#include <cstdint>

#include "micras/hal/gpio.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling the locomotion driver
 */
class Locomotion {
public:
    /**
     * @brief Configuration structure for the locomotion
     */
    struct Config {
        hal::Pwm::Config  pwm_left_forward;
        hal::Pwm::Config  pwm_left_backwards;
        hal::Pwm::Config  pwm_right_forward;
        hal::Pwm::Config  pwm_right_backwards;
        hal::Gpio::Config enable_gpio;
    };

    /**
     * @brief Construct a new locomotion object
     *
     * @param config Configuration for the locomotion driver
     */
    explicit Locomotion(const Config& config);

    /**
     * @brief Enable the locomotion driver
     */
    void enable();

    /**
     * @brief Disable the locomotion driver
     */
    void disable();

    /**
     * @brief Set the command of the wheels
     *
     * @param left_command Command of the left wheels
     * @param right_command Command of the right wheels
     */
    void set_wheel_command(float left_command, float right_command);

    /**
     * @brief Set the linear and angular commands of the robot
     *
     * @param linear Linear command of the robot
     * @param angular Angular command of the robot
     */
    void set_command(float linear, float angular);

    /**
     * @brief Stop the motors
     */
    void stop();

private:
    /**
     * @brief PWM handle for the left motor forward
     */
    hal::Pwm pwm_left_forward;

    /**
     * @brief PWM handle for the left motor backward
     */
    hal::Pwm pwm_left_backwards;

    /**
     * @brief PWM handle for the right motor forward
     */
    hal::Pwm pwm_right_forward;

    /**
     * @brief PWM handle for the right motor backward
     */
    hal::Pwm pwm_right_backwards;

    /**
     * @brief GPIO handle for the motor driver enable pin
     */
    hal::Gpio enable_gpio;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LOCOMOTION_HPP
