/**
 * @file
 */

#ifndef MICRAS_NAV_FOLLOW_WALL
#define MICRAS_NAV_FOLLOW_WALL

#include <memory>

#include "micras/core/butterworth_filter.hpp"
#include "micras/core/pid_controller.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::nav {
/**
 * @brief Class to follow the side walls using a PID controller.
 */
class FollowWall {
public:
    /**
     * @brief Configuration struct for the FollowWall class.
     */
    struct Config {
        core::PidController::Config pid;
        float                       base_left_reading{};
        float                       base_right_reading{};
    };

    /**
     * @brief Construct a new FollowWall object.
     *
     * @param wall_sensors The wall sensors of the robot.
     * @param config The configuration for the FollowWall class.
     */
    FollowWall(const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Config& config);

    /**
     * @brief Update the PID controller and return the response.
     *
     * @param follow_wall_type The type of wall following to perform.
     * @param elapsed_time The time elapsed since the last update.
     * @return The response of the PID controller.
     */
    float action(core::FollowWallType follow_wall_type, float elapsed_time);

    /**
     * @brief Reset the PID controller.
     */
    void reset();

    /**
     * @brief Reset the base readings of the wall sensors.
     */
    void reset_base_readings();

private:
    /**
     * @brief Get the reading from the left wall sensor.
     *
     * @return The reading from the left wall sensor.
     */
    float get_left_value() const;

    /**
     * @brief Get the reading from the right wall sensor.
     *
     * @return The reading from the right wall sensor.
     */
    float get_right_value() const;

    /**
     * @brief Wall sensors of the robot.
     */
    std::shared_ptr<proxy::TWallSensors<4>> wall_sensors;

    /**
     * @brief PID controller for the wall following.
     */
    core::PidController pid;

    /**
     * @brief Base reading of the left wall sensor when aligned with the wall.
     */
    float base_left_reading;

    /**
     * @brief Base reading of the right wall sensor when aligned with the wall.
     */
    float base_right_reading;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_FOLLOW_WALL
