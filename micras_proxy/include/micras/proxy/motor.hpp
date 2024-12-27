/**
 * @file
 */

#ifndef MICRAS_PROXY_MOTOR_HPP
#define MICRAS_PROXY_MOTOR_HPP

#include "micras/hal/pwm.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling a motor driver
 */
class Motor {
public:
    /**
     * @brief Configuration structure for the motor
     */
    struct Config {
        hal::Pwm::Config backwards_pwm;
        hal::Pwm::Config forward_pwm;
        float            max_stopped_command;
        float            deadzone;
    };

    /**
     * @brief Construct a new motor object
     *
     * @param config Configuration for the motor driver
     */
    explicit Motor(const Config& config);

    /**
     * @brief Set the command for the motor
     *
     * @param command Command for the motor in percentage
     */
    void set_command(float command);

private:
    /**
     * @brief PWM object for controlling the motor in the backwards direction
     */
    hal::Pwm backwards_pwm;

    /**
     * @brief PWM object for controlling the motor in the forward direction
     */
    hal::Pwm forward_pwm;

    /**
     * @brief Maximum command value for the motor to be considered stopped
     */
    float max_stopped_command;

    /**
     * @brief Deadzone for the motor
     */
    float deadzone;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_MOTOR_HPP
