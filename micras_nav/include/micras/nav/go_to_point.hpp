/**
 * @file
 */

#ifndef MICRAS_NAV_GO_TO_POINT_HPP
#define MICRAS_NAV_GO_TO_POINT_HPP

#include "micras/nav/follow_wall.hpp"
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
        PidController::Config stop_pid;
        PidController::Config angular_pid;
        float                 cell_size{};
        float                 min_linear_command{};
        float                 max_linear_command{};
        float                 deceleration_factor{};
        float                 distance_tolerance{};
        float                 velocity_tolerance{};
    };

    /**
     * @brief Constructor for the GoToPoint class
     *
     * @param wall_sensors The wall sensors of the robot
     * @param config The configuration for the GoToPoint class
     * @param follow_wall_config The configuration for the FollowWall class
     */
    GoToPoint(
        const proxy::TWallSensors<4>& wall_sensors, const Config& config, const FollowWall::Config& follow_wall_config
    );

    /**
     * @brief Calculates the command to go to a point
     *
     * @param state The current state of the robot
     * @param goal The goal point
     * @param follow_wall_type The type of wall following to perform
     * @param elapsed_time The time elapsed since the last update
     * @param stop Whether to stop at the goal
     *
     * @return The command to go to the point
     */
    Twist action(
        const State& state, const Point& goal, core::FollowWallType follow_wall_type, float elapsed_time,
        bool stop = true
    );

    /**
     * @brief Checks if the robot has reached the goal
     *
     * @param state The current state of the robot
     * @param goal The goal point
     * @param stop Whether to stop at the goal
     *
     * @return True if the robot has reached the goal, false otherwise
     */
    bool finished(const State& state, const Point& goal, bool stop = true) const;

    /**
     * @brief Resets the PID controllers
     */
    void reset();

    /**
     * @brief Calibrates the wall sensors
     */
    void calibrate();

private:
    /**
     * @brief PID controller for stoppping at the goal
     */
    PidController stop_pid;

    /**
     * @brief PID controller for the orientation
     */
    PidController angular_pid;

    /**
     * @brief FollowWall controller
     */
    FollowWall follow_wall;

    /**
     * @brief Size of the grid cells
     */
    float cell_size;

    /**
     * @brief Minimum linear command
     */
    float min_linear_command;

    /**
     * @brief Maximum linear command
     */
    float max_linear_command;

    /**
     * @brief Deceleration factor for the linear command transition
     */
    float deceleration_factor;

    /**
     * @brief Linear tolerance for the goal point
     */
    float distance_tolerance;

    /**
     * @brief Velocity tolerance at the goal point
     */
    float velocity_tolerance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_GO_TO_POINT_HPP
