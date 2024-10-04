/**
 * @file odometry.hpp
 *
 * @brief Nav Odometry class declaration
 *
 * @date 10/2024
 */

#ifndef MICRAS_NAV_ODOMETRY_HPP
#define MICRAS_NAV_ODOMETRY_HPP

#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/hal/timer.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/rotary_sensor.hpp"

namespace micras::nav {
/**
 * @brief Class for calculating the robot odometry
 */
class Odometry {
public:
    struct Config {
        float linear_cutoff_frequency;
        float angular_cutoff_frequency;
        float wheel_radius;
        float wheel_separation;
        Pose  initial_pose;
    };

    /**
     * @brief Constructor for the Odometry class
     *
     * @param left_rotary_sensor Left rotary sensor
     * @param right_rotary_sensor Right rotary sensor
     * @param config Configuration for the odometry
     */
    Odometry(
        const proxy::RotarySensor& left_rotary_sensor, const proxy::RotarySensor& right_rotary_sensor,
        const proxy::Imu& imu, Config config
    );

    /**
     * @brief Update the odometry
     *
     * @param elapsed_time Time since the last update
     */
    void update(float elapsed_time);

    /**
     * @brief Get the state of the robot
     *
     * @return State current state of the robot in space
     */
    const State& get_state() const;

    /**
     * @brief Get the state of the robot calculated using the IMU
     *
     * @return State current state of the robot in space
     */
    const State& get_imu_state() const;

private:
    /**
     * @brief Update a state based on the position variation
     *
     * @param state State to be updated
     * @param linear_distance Linear distance from last state
     * @param angular_distance Angular distance from last state
     * @param linear_velocity Measured linear velocity of the robot
     * @param angular_velocity Measured angular velocity of the robot
     */
    static constexpr void update_state(
        State& state, float linear_distance, float angular_distance, float linear_velocity, float angular_velocity
    ) {
        float half_angle = angular_distance / 2;
        float linear_diagonal =
            angular_distance < 0.05F ? linear_distance : std::abs(std::sin(half_angle) * linear_distance / half_angle);

        state.pose.position.x += linear_diagonal * std::cos(state.pose.orientation + half_angle);
        state.pose.position.y += linear_diagonal * std::sin(state.pose.orientation + half_angle);

        state.pose.orientation += angular_distance;

        state.velocity.linear = linear_velocity;
        state.velocity.angular = angular_velocity;
    }

    /**
     * @brief Left rotary sensor
     */
    const proxy::RotarySensor& left_rotary_sensor;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    /**
     * @brief Right rotary sensor
     */
    const proxy::RotarySensor& right_rotary_sensor;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    /**
     * @brief IMU sensor
     */
    const proxy::Imu& imu;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    /**
     * @brief Wheel radius
     */
    float wheel_radius;

    /**
     * @brief Robot width
     */
    float wheel_separation;

    /**
     * @brief Last left rotary sensor position
     */
    float left_last_position{};

    /**
     * @brief Last right rotary sensor position
     */
    float right_last_position{};

    /**
     * @brief Linear velocity filter
     */
    core::ButterworthFilter linear_filter;

    /**
     * @brief Angular velocity filter
     */
    core::ButterworthFilter angular_filter;

    /**
     * @brief Current state of the robot in space
     */
    State state;

    /**
     * @brief Current state of the robot in space calculated using the IMU
     */
    State imu_state;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ODOMETRY_HPP
