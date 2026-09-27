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
     *
     * @note The reserved rotation is the part of the range, in percent, that the rotation command
     * keeps when the two commands do not fit in it together.
     */
    struct Config {
        Motor::Config     left_motor;
        Motor::Config     right_motor;
        hal::Gpio::Config enable_gpio;
        float             reserved_rotation;
    };

    /**
     * @brief Linear and angular commands, in percent of the motor supply.
     */
    struct Command {
        float linear;
        float angular;
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
     * @details When the two commands do not fit in the range of the motors together, it is the
     * linear one that gives way: a robot that slows down stays on its path, while one that turns
     * less than it was asked to leaves it. The rotation is only guaranteed up to the reserved
     * part of the range, so that a rotation command gone wrong cannot stall the robot.
     *
     * @param linear Linear command of the robot.
     * @param angular Angular command of the robot.
     * @return The commands that were applied, which differ from the ones asked for when the motors
     * saturate.
     */
    Command set_command(float linear, float angular);

    /**
     * @brief Stop the motors.
     */
    void stop();

    /**
     * @brief Check if both motors were successfully initialized.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Largest command a motor takes, in percent.
     */
    static constexpr float max_command{100.0F};

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

    /**
     * @brief Part of the range that the rotation command keeps under saturation, in percent.
     */
    float reserved_rotation;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_LOCOMOTION_HPP
