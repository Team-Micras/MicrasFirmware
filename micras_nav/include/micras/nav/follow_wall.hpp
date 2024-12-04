/**
 * @file
 */

#ifndef MICRAS_NAV_FOLLOW_WALL
#define MICRAS_NAV_FOLLOW_WALL

#include "micras/core/butterworth_filter.hpp"
#include "micras/nav/pid_controller.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::nav {
/**
 * @brief Class to follow a wall using a PID controller
 */
class FollowWall {
public:
    /**
     * @brief Configuration for the FollowWall class
     */
    struct Config {
        PidController::Config pid;
        float                 base_left_reading{};
        float                 base_right_reading{};
        float                 cutoff_frequency{};
    };

    /**
     * @brief Construct a new FollowWall object
     *
     * @param wall_sensors The wall sensors of the robot
     * @param config The configuration for the FollowWall class
     */
    FollowWall(const proxy::WallSensors<4>& wall_sensors, const Config& config);

    /**
     * @brief Update the PID controller and return the response
     *
     * @param follow_wall_type The type of wall following to perform
     * @param elapsed_time The time elapsed since the last update
     *
     * @return The response of the PID controller
     */
    float action(core::FollowWallType follow_wall_type, float elapsed_time);

    /**
     * @brief Resets the PID controller
     */
    void reset();

    /**
     * @brief Resets the base readings of the wall sensors
     */
    void reset_base_readings();

private:
    /**
     * @briefGet the reading of the left wall sensor
     *
     * @return float
     */
    float get_left_value() const;

    /**
     * @brief Get the reading of the right wall sensor
     *
     * @return float
     */
    float get_right_value() const;

    /**
     * @brief Wall sensors of the robot
     */
    const proxy::WallSensors<4>& wall_sensors;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    /**
     * @brief PID controller for the wall following
     */
    PidController pid;

    /**
     * @brief Butterworth filter for the derivative of the error
     */
    core::ButterworthFilter filter;

    /**
     * @brief Last difference between the left and right wall sensors
     */
    float last_diff{};

    /**
     * @brief Base reading of the left wall sensor when aligned with the wall
     */
    float base_left_reading;

    /**
     * @brief Base reading of the right wall sensor when aligned with the wall
     */
    float base_right_reading;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_FOLLOW_WALL
