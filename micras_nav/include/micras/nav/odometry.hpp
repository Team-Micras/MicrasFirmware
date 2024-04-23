/**
 * @file odometry.hpp
 *
 * @brief Nav Odometry class declaration
 *
 * @date 04/2024
 */

#ifndef MICRAS_NAV_ODOMETRY_HPP
#define MICRAS_NAV_ODOMETRY_HPP

#include <cstdint>

#include "micras/hal/timer.hpp"
#include "micras/proxy/rotary_sensor.hpp"

namespace micras::nav {
/**
 * @brief Class for calculating the robot odometry
 */
class Odometry {
public:
    /**
     * @brief Constructor for the Odometry class
     *
     * @param config Configuration for the odometry
     */
    Odometry(
        const proxy::RotarySensor& left_rotary_sensor, const proxy::RotarySensor& right_rotary_sensor,
        hal::Timer::Config timer_config, float wheel_radius, float wheel_separation
    );

    /**
     * @brief Update the odometry
     */
    void update();

    /**
     * @brief Get the position x
     *
     * @return float Position x
     */
    float get_position_x() const;

    /**
     * @brief Get the position y
     *
     * @return float Position y
     */
    float get_position_y() const;

    /**
     * @brief Get the orientation
     *
     * @return float Orientation
     */
    float get_orientation() const;

    /**
     * @brief Get the linear velocity
     *
     * @return float Linear velocity
     */
    float get_linear_velocity() const;

    /**
     * @brief Get the angular velocity
     *
     * @return float Angular velocity
     */
    float get_angular_velocity() const;

private:
    /**
     * @brief Left rotary sensor
     */
    const proxy::RotarySensor& left_rotary_sensor;

    /**
     * @brief Right rotary sensor
     */
    const proxy::RotarySensor& right_rotary_sensor;

    /**
     * @brief Timer for velocity calculation
     */
    hal::Timer timer;

    /**
     * @brief Wheel radius
     */
    const float wheel_radius;

    /**
     * @brief Robot width
     */
    const float wheel_separation;

    /**
     * @brief Last left rotary sensor position
     */
    float left_last_position{};

    /**
     * @brief Last right rotary sensor position
     */
    float right_last_position{};

    /**
     * @brief Odometry variables
     */
    float position_x{};
    float position_y{};
    float orientation{};
    float linear_velocity{};
    float angular_velocity{};
};

}  // namespace micras::nav

#endif  // MICRAS_NAV_ODOMETRY_HPP
