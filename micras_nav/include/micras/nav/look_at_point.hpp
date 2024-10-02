/**
 * @file look_at_point.hpp
 *
 * @brief Nav LookAtPoint class declaration
 *
 * @date 10/2024
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
        PidController::Config pid;
        float                 tolerance{};
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
     * @return The command to look at the point
     */
    Twist action(const Pose& pose, const Point& goal);

    /**
     * @brief Checks if the robot has reached the goal
     *
     * @param pose The current pose of the robot
     * @param goal The goal point
     * @return True if the robot has reached the goal, false otherwise
     */
    bool finished(const Pose& pose, const Point& goal) const;

private:
    /**
     * @brief PID controller for the angular velocity
     */
    PidController pid;

    /**
     * @brief Angular tolerance for the goal
     */
    float tolerance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_LOOK_AT_POINT_HPP
