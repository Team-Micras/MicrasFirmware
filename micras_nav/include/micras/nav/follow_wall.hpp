/**
 * @file
 */

#ifndef MICRAS_NAV_FOLLOW_WALL
#define MICRAS_NAV_FOLLOW_WALL

#include <memory>

#include "micras/core/pid_controller.hpp"
#include "micras/nav/state.hpp"
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
        float                       max_linear_speed{};
        float                       post_threshold{};
        float                       cell_size{};
        float                       post_margin{};
    };

    /**
     * @brief Construct a new FollowWall object.
     *
     * @param wall_sensors The wall sensors of the robot.
     * @param config The configuration for the FollowWall class.
     */
    FollowWall(
        const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Pose& absolute_pose, const Config& config
    );

    /**
     * @brief Update the PID controller and return the response.
     *
     * @param follow_wall_type The type of wall following to perform.
     * @param elapsed_time The time elapsed since the last update.
     * @return The response of the PID controller.
     */
    float action(float elapsed_time, float linear_speed);

    /**
     * @brief Reset the PID controller.
     */
    void reset();

private:
    void reset_displacement(bool reseted_by_post = false);

    bool check_posts();

    /**
     * @brief Wall sensors of the robot.
     */
    std::shared_ptr<proxy::TWallSensors<4>> wall_sensors;

    /**
     * @brief PID controller for the wall following.
     */
    core::PidController pid;

    float max_linear_speed;

    float post_threshold;

    RelativePose blind_pose;
    float        last_blind_distance{};

    float cell_size;
    float post_margin;

    bool left_wall{true};
    bool right_wall{true};

    float last_right_error{};
    float last_left_error{};

    bool reseted_by_post{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_FOLLOW_WALL
