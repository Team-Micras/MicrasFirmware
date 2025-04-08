/**
 * @file
 */

#ifndef MICRAS_NAV_ODOMETRY_HPP
#define MICRAS_NAV_ODOMETRY_HPP

#include <cstdint>
#include <memory>

#include "micras/core/butterworth_filter.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/rotary_sensor.hpp"

namespace micras::nav {
/**
 * @brief Class for calculating the robot odometry.
 */
class Odometry {
public:
    /**
     * @brief Configuration for the odometry.
     */
    struct Config {
        float linear_cutoff_frequency;
        float wheel_radius;
        Pose  initial_pose;
    };

    /**
     * @brief Construct a new Odometry object.
     *
     * @param left_rotary_sensor Left rotary sensor.
     * @param right_rotary_sensor Right rotary sensor.
     * @param imu IMU sensor.
     * @param config Configuration for the odometry.
     */
    Odometry(
        const std::shared_ptr<proxy::RotarySensor>& left_rotary_sensor,
        const std::shared_ptr<proxy::RotarySensor>& right_rotary_sensor, const std::shared_ptr<proxy::Imu>& imu,
        Config config
    );

    /**
     * @brief Update the odometry.
     *
     * @param elapsed_time Time since the last update.
     */
    void update(float elapsed_time);

    /**
     * @brief Reset the odometry.
     */
    void reset();

    /**
     * @brief Get the state of the robot.
     *
     * @return Current state of the robot in space.
     */
    const State& get_state() const;

    /**
     * @brief Set the state of the robot.
     *
     * @param state New state of the robot.
     */
    void set_state(const State& new_state);

private:
    /**
     * @brief Left rotary sensor.
     */
    std::shared_ptr<proxy::RotarySensor> left_rotary_sensor;

    /**
     * @brief Right rotary sensor.
     */
    std::shared_ptr<proxy::RotarySensor> right_rotary_sensor;

    /**
     * @brief IMU sensor.
     */
    std::shared_ptr<proxy::Imu> imu;

    /**
     * @brief Wheel radius.
     */
    float wheel_radius;

    /**
     * @brief Last left rotary sensor position.
     */
    float left_last_position{};

    /**
     * @brief Last right rotary sensor position.
     */
    float right_last_position{};

    /**
     * @brief Linear velocity filter.
     */
    core::ButterworthFilter linear_filter;

    /**
     * @brief Current state of the robot in space.
     */
    State state;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ODOMETRY_HPP
