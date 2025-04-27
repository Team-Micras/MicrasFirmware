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
 * @brief Class to calculate a command to follow a pair of linear and angular speeds.
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
     * @brief Calculate the command to achieve the desired speeds.
     *
     * @param current_twist The current speeds of the robot.
     * @param desired_twist The desired speeds of the robot.
     * @param elapsed_time The time since the last update.
     * @return A pair of floats representing the left and right wheel speeds.
     */
    std::pair<float, float> action(const Twist& current_twist, const Twist& desired_twist, float elapsed_time);

    /**
     * @brief Reset the PID controllers.
     */
    void reset();

private:
    /**
     * @brief Calculate the feed-forward term for a motor.
     *
     * @param speed The current speeds of the robot.
     * @param acceleration The current accelerations of the robot.
     * @param config The configuration for the feed-forward term.
     * @return The feed-forward parameters for a motor.
     */
    static float feed_forward(const Twist& speed, const Twist& acceleration, const Config::FeedForward& config);

    /**
     * @brief The maximum linear acceleration of the robot.
     */
    float max_linear_acceleration;

    /**
     * @brief The maximum angular acceleration of the robot.
     */
    float max_angular_acceleration;

    /**
     * @brief The last linear speed of the robot.
     */
    float last_linear_speed{};

    /**
     * @brief The last angular speed of the robot.
     */
    float last_angular_speed{};

    /**
     * @brief PID controller for stopping at the goal.
     */
    core::PidController linear_pid;

    /**
     * @brief PID controller for the orientation.
     */
    core::PidController angular_pid;

    /**
     * @brief Feed-forward parameters for the left motor.
     */
    Config::FeedForward left_feed_forward;

    /**
     * @brief Feed-forward parameters for the right motor.
     */
    Config::FeedForward right_feed_forward;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_SPEED_CONTROLLER_HPP
