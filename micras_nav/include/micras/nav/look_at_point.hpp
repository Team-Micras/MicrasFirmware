/**
 * @file
 */

#ifndef MICRAS_NAV_LOOK_AT_POINT_HPP
#define MICRAS_NAV_LOOK_AT_POINT_HPP

#include "micras/core/pid_controller.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Class to calculate a command to look at a point.
 */
class LookAtPoint {
public:
    /**
     * @brief Configuration struct for the LookAtPoint class.
     */
    struct Config {
        core::PidController::Config linear_pid;
        core::PidController::Config angular_pid;
        float                       distance_tolerance{};
        float                       velocity_tolerance{};
    };

    /**
     * @brief Construct a new LookAtPoint object.
     *
     * @param config The configuration for the LookAtPoint class.
     */
    explicit LookAtPoint(Config config);

    /**
     * @brief Calculate the command to look at a point.
     *
     * @param state The current state of the robot.
     * @param goal The goal point.
     * @param elapsed_time The time elapsed since the last update.
     * @return The command to look at the point.
     */
    Twist action(const State& state, const Point& goal, float elapsed_time);

    /**
     * @brief Reset the PID controllers.
     */
    void reset();

    /**
     * @brief Check if the robot has reached the goal.
     *
     * @param state The current state of the robot.
     * @param goal The goal point.
     * @param stop Whether to stop the robot when it reaches the goal.
     * @return True if the robot has reached the goal, false otherwise.
     */
    bool finished(const State& state, const Point& goal, bool stop = true) const;

private:
    /**
     * @brief PID controller for the linear velocity.
     */
    core::PidController linear_pid;

    /**
     * @brief PID controller for the angular velocity.
     */
    core::PidController angular_pid;

    /**
     * @brief Angular tolerance for the goal.
     */
    float distance_tolerance;

    /**
     * @brief Velocity tolerance for the goal.
     */
    float velocity_tolerance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_LOOK_AT_POINT_HPP
