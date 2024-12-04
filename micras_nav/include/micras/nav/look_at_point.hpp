/**
 * @file
 */

#ifndef MICRAS_NAV_LOOK_AT_POINT_HPP
#define MICRAS_NAV_LOOK_AT_POINT_HPP

#include "micras/nav/pid_controller.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Class to calculate a command to look at a point
 */
class LookAtPoint {
public:
    /**
     * @brief Configuration for the LookAtPoint class
     */
    struct Config {
        PidController::Config linear_pid;
        PidController::Config angular_pid;
        float                 distance_tolerance{};
        float                 velocity_tolerance{};
    };

    /**
     * @brief Constructor for the LookAtPoint class
     *
     * @param config The configuration for the LookAtPoint class
     */
    explicit LookAtPoint(Config config);

    /**
     * @brief Calculates the command to look at a point
     *
     * @param pose The current pose of the robot
     * @param goal The goal point
     * @param elapsed_time The time elapsed since the last update
     *
     * @return The command to look at the point
     */
    Twist action(const State& state, const Point& goal, float elapsed_time);

    /**
     * @brief Resets the PID controllers
     */
    void reset();

    /**
     * @brief Checks if the robot has reached the goal
     *
     * @param pose The current pose of the robot
     * @param goal The goal point
     *
     * @return True if the robot has reached the goal, false otherwise
     */
    bool finished(const State& state, const Point& goal, bool stop = true) const;

private:
    /**
     * @brief PID controller for the linear velocity
     */
    PidController linear_pid;

    /**
     * @brief PID controller for the angular velocity
     */
    PidController angular_pid;

    /**
     * @brief Angular tolerance for the goal
     */
    float distance_tolerance;

    /**
     * @brief Velocity tolerance for the goal
     */
    float velocity_tolerance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_LOOK_AT_POINT_HPP
