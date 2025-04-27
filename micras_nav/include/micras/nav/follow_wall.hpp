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
     * @param absolute_pose Reference to the absolute pose of the robot.
     * @param config The configuration for the FollowWall class.
     */
    FollowWall(
        const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Pose& absolute_pose, const Config& config
    );

    /**
     * @brief Update the PID controller and return the response.
     *
     * @param elapsed_time The time elapsed since the last update.
     * @param linear_speed Current linear speed of the robot.
     * @return The response of the PID controller.
     */
    float action(float elapsed_time, float linear_speed);

    /**
     * @brief Reset the PID controller and the relative pose.
     */
    void reset();

private:
    /**
     * @brief Reset the displacement of the robot.
     *
     * @param reseted_by_post Whether the reset was triggered by a post.
     */
    void reset_displacement(bool reseted_by_post = false);

    /**
     * @brief Check if the robot is seeing a post.
     *
     * @return True if the robot is seeing a post, false otherwise.
     *
     * @details This function uses the derivative of the distance sensors reading in relation to the distance
     * traveled by the robot, comparing this value to the threshold to detect posts.
     */
    bool check_posts();

    /**
     * @brief Wall sensors of the robot.
     */
    std::shared_ptr<proxy::TWallSensors<4>> wall_sensors;

    /**
     * @brief PID controller for the wall following.
     */
    core::PidController pid;

    /**
     * @brief Maximum linear speed of the robot.
     */
    float max_linear_speed;

    /**
     * @brief Derivative threshold for detecting posts.
     */
    float post_threshold;

    /**
     * @brief Current pose of the robot relative to the last time it was reset.
     */
    RelativePose blind_pose;

    /**
     * @brief Last distance of the robot to the start point, used to compute the derivative of the distance sensors.
     */
    float last_blind_distance{};

    /**
     * @brief Size of the cells in the map.
     */
    float cell_size;

    /**
     * @brief Margin in front of a post to stop seeing it.
     */
    float post_margin;

    /**
     * @brief Flag to indicate if the robot is currently following the left wall.
     */
    bool left_wall{true};

    /**
     * @brief Flag to indicate if the robot is currently following the right wall.
     */
    bool right_wall{true};

    /**
     * @brief Last error measured by the left wall sensor, used to compute the derivative of the distance sensors.
     */
    float last_left_error{};

    /**
     * @brief Last error measured by the right wall sensor, used to compute the derivative of the distance sensors.
     */
    float last_right_error{};

    /**
     * @brief Flag to indicate if relative pose was reset by a post.
     */
    bool reseted_by_post{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_FOLLOW_WALL
