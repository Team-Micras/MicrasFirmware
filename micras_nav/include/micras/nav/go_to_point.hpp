/**
 * @file go_to_point.hpp
 *
 * @brief Nav GoToPoint class declaration
 *
 * @date 10/2024
 */

#ifndef MICRAS_NAV_GO_TO_POINT_HPP
#define MICRAS_NAV_GO_TO_POINT_HPP

#include "micras/nav/pid_controller.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Class to calculate a command to go to a point
 */
class GoToPoint {
public:
    /**
     * @brief Configuration for the GoToPoint class
     */
    struct Config {
        PidController::Config linear_pid;
        PidController::Config angular_pid;
        float                 base_speed{};
        float                 linear_decay_damping{};
        float                 tolerance{};
    };

    /**
     * @brief Constructor for the GoToPoint class
     *
     * @param config The configuration for the GoToPoint class
     */
    explicit GoToPoint(Config config);

    /**
     * @brief Calculates the command to go to a point
     *
     * @param state The current state of the robot
     * @param goal The goal point
     * @return The command to go to the point
     */
    Twist action(const State& state, const Point& goal);

    /**
     * @brief Checks if the robot has reached the goal
     *
     * @param state The current state of the robot
     * @param goal The goal point
     * @return True if the robot has reached the goal, false otherwise
     */
    bool finished(const State& state, const Point& goal) const;

private:
    /**
     * @brief PID controller for the linear velocity
     */
    PidController linear_pid;

    /**
     * @brief PID controller for the orientation
     */
    PidController angular_pid;

    /**
     * @brief Base linear speed for the robot
     */
    float base_speed;

    /**
     * @brief Damping factor for the linear decay
     */
    float linear_decay_damping;

    /**
     * @brief Linear tolerance for the goal point
     */
    float tolerance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_GO_TO_POINT_HPP
