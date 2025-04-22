/**
 * @file
 */

#ifndef MICRAS_NAV_SPEED_CONTROLLER_HPP
#define MICRAS_NAV_SPEED_CONTROLLER_HPP

#include <utility>

#include "micras/core/pid_controller.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Class to calculate a command to go to a point.
 */
class SpeedController {
public:
    /**
     * @brief Configuration struct for the SpeedController class.
     */
    struct Config {
        struct FeedForward {
            float linear_speed;
            float linear_acceleration;
            float angular_speed;
            float angular_acceleration;
        };

        float                       max_linear_acceleration{};
        float                       max_angular_acceleration{};
        core::PidController::Config linear_pid;
        core::PidController::Config angular_pid;
        FeedForward                 left_feed_forward{};
        FeedForward                 right_feed_forward{};
    };

    /**
     * @brief Construct a new SpeedController object.
     *
     * @param config The configuration for the SpeedController class.
     */
    explicit SpeedController(const Config& config);

    /**
     * @brief Calculate the command to go to a point.
     *
     * @param state The current state of the robot.
     * @param goal The goal point.
     * @param elapsed_time The time elapsed since the last update.
     * @return The command to go to the point.
     */
    std::pair<float, float> action(const Twist& current_twist, const Twist& desired_twist, float elapsed_time);

    /**
     * @brief Reset the PID controllers.
     */
    void reset();

private:
    static float feed_forward(const Twist& speed, const Twist& acceleration, const Config::FeedForward& config);

    float max_linear_acceleration;
    float max_angular_acceleration;

    float last_linear_speed{};
    float last_angular_speed{};

    /**
     * @brief PID controller for stopping at the goal.
     */
    core::PidController linear_pid;

    /**
     * @brief PID controller for the orientation.
     */
    core::PidController angular_pid;

    Config::FeedForward left_feed_forward;

    Config::FeedForward right_feed_forward;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_SPEED_CONTROLLER_HPP
