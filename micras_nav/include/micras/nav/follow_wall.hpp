/**
 * @file
 */

#ifndef MICRAS_NAV_FOLLOW_WALL
#define MICRAS_NAV_FOLLOW_WALL

#include <memory>

#include "micras/core/pid_controller.hpp"
#include "micras/core/types.hpp"
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
        core::WallSensorsIndex      wall_sensor_index{};
        float                       max_linear_speed{};
        float                       max_angular_acceleration{};
        float                       cell_size{};
        float                       post_threshold{};
        float                       post_clearance{};
        float                       post_reference{};
    };

    /**
     * @brief Construct a new FollowWall object.
     *
     * @param wall_sensors The wall sensors of the robot.
     * @param config The configuration for the FollowWall class.
     */
    FollowWall(const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Config& config);

    /**
     * @brief Calculate the desired angular speed to follow wall.
     *
     * @param elapsed_time The time elapsed since the last update.
     * @param state The current state of the robot.
     * @return The desired angular speed to follow wall.
     */
    float compute_angular_correction(float elapsed_time, State& state);

    /**
     * @brief Get the observation of the walls around the robot.
     *
     * @return Observations from all sensors.
     */
    core::Observation get_observation() const;

private:
    /**
     * @brief Check if the robot saw a post.
     *
     * @param cell_advance The distance of the robot from the start of the cell.
     * @return True if the robot saw a post, false otherwise.
     *
     * @details This function uses the derivative of the distance sensors readings with respect
     * to the robot's traveled distance, comparing it to a threshold to detect posts.
     */
    bool check_posts(float cell_advance);

    /**
     * @brief Reset the PID controller and the relative pose.
     */
    void reset();

    /**
     * @brief Clear the positional error of the robot.
     *
     * @param state The state of the robot.
     * @param error The error to be cleared.
     */
    static void clear_position_error(State& state, float error);

    /**
     * @brief Wall sensors of the robot.
     */
    std::shared_ptr<proxy::TWallSensors<4>> wall_sensors;

    /**
     * @brief PID controller for the wall following.
     */
    core::PidController pid;

    /**
     * @brief Index of each of the wall sensors used for wall following.
     */
    core::WallSensorsIndex sensor_index;

    /**
     * @brief Maximum linear speed of the robot.
     */
    float max_linear_speed;

    /**
     * @brief Maximum angular acceleration of the robot.
     */
    float max_angular_acceleration;

    /**
     * @brief Size of the cells in the map.
     */
    float cell_size;

    /**
     * @brief Derivative threshold for detecting posts.
     */
    float post_threshold;

    /**
     * @brief Margin in front of a post to stop seeing it.
     */
    float post_clearance;

    /**
     * @brief Cell advance of the robot when it sees a post.
     */
    float post_reference;

    /**
     * @brief Last grid pose of the robot.
     */
    GridPose last_grid_pose{};

    /**
     * @brief Last cell advance of the robot.
     */
    float last_cell_advance{};

    /**
     * @brief Flag to indicate if the robot is currently following the left wall.
     */
    bool following_left{true};

    /**
     * @brief Flag to indicate if the robot is currently following the right wall.
     */
    bool following_right{true};

    /**
     * @brief Last reading measured by the left wall sensor, used to compute the derivative of the distance sensors.
     */
    float last_left_reading{};

    /**
     * @brief Last reading measured by the right wall sensor, used to compute the derivative of the distance sensors.
     */
    float last_right_reading{};

    /**
     * @brief Last response returned by the Follow Wall.
     */
    float last_response{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_FOLLOW_WALL
