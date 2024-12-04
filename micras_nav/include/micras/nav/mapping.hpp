/**
 * @file
 */

#ifndef MICRAS_NAV_MAPPING_HPP
#define MICRAS_NAV_MAPPING_HPP

#include <array>

#include "micras/nav/maze.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::nav {
/**
 * @brief Class for mapping the maze
 *
 * @tparam width The width of the maze
 * @tparam height The height of the maze
 */
template <uint8_t width, uint8_t height>
class Mapping {
public:
    /**
     * @brief Configuration for the Mapping class
     */
    struct Config {
        float                wall_thickness{};
        float                cell_size{};
        float                alignment_threshold{};
        Pose                 front_sensor_pose{};
        Pose                 side_sensor_pose{};
        float                front_distance_alignment_tolerance{};
        float                side_distance_alignment_tolerance{};
        float                front_orientation_alignment_tolerance{};
        float                side_orientation_alignment_tolerance{};
        std::array<float, 2> front_distance_reading{};
        std::array<float, 2> front_orientation_reading{};
        std::array<float, 2> side_distance_reading{};

        GridPose                      start{};
        std::unordered_set<GridPoint> goal{
            {{width / 2, height / 2},
             {(width - 1) / 2, height / 2},
             {width / 2, (height - 1) / 2},
             {(width - 1) / 2, (height - 1) / 2}}
        };
    };

    /**
     * @brief Type to identify the wall sensors
     */
    enum Sensor : uint8_t {
        FRONT_LEFT,
        LEFT,
        RIGHT,
        FRONT_RIGHT
    };

    /**
     * @brief Type to store an action to be taken
     */
    struct Action {
        /**
         * @brief TPossible types of actions
         */
        enum Type : uint8_t {
            LOOK_AT,
            GO_TO
        };

        /**
         * @brief Type of the action
         */
        Type type;

        /**
         * @brief The next goal point
         */
        Point point;

        /**
         * @brief The orientation of the goal point
         */
        Side side;
    };

    /**
     * @brief Constructor for the Mapping class
     *
     * @param wall_sensors The wall sensors
     * @param config The configuration for the mapping
     */
    Mapping(const proxy::WallSensors<4>& wall_sensors, Config config);

    /**
     * @brief Update the mapping of the maze using the current pose and wall sensors
     *
     * @param pose The current pose of the robot
     */
    void update(const Pose& pose);

    /**
     * @brief Get the action to be taken based on the current pose
     *
     * @param pose The current pose of the robot
     * @return The action to be taken
     */
    Action get_action(const Pose& pose) const;

    /**
     * @brief Fix the pose based on the mapping information
     *
     * @param pose The current pose of the robot
     * @param follow_wall_type The type of wall following being performed
     * @return The fixed pose
     */
    Pose correct_pose(const Pose& pose, core::FollowWallType follow_wall_type) const;

    /**
     * @brief Calibrate front alignment of the wall sensors
     */
    void calibrate_front();

    /**
     * @brief Calibrate side alignment of the wall sensors
     */
    void calibrate_side();

    /**
     * @brief Check if the robot is at a distance from a wall at the front
     *
     * @return True if the robot is aligned, false otherwise
     */
    bool is_distance_front_aligned() const;

    /**
     * @brief Check if the robot orientation is aligned with walls at the front
     *
     * @return True if the robot is aligned, false otherwise
     */
    bool is_orientation_front_aligned() const;

    /**
     * @brief Check if the robot is at a distance from a wall at the sides
     *
     * @return True if the robot is aligned, false otherwise
     */
    bool is_distance_side_aligned() const;

    /**
     * @brief Check if the robot orientation is aligned with walls at the sides
     *
     * @return True if the robot is aligned, false otherwise
     */
    bool is_orientation_side_aligned() const;

    /**
     * @brief Get the type of wall following the robot can do
     *
     * @param pose The current pose of the robot
     * @return The type of wall following
     */
    core::FollowWallType get_follow_wall_type(const Pose& pose) const;

private:
    /**
     * @brief Wall sensors of the robot
     */
    const proxy::WallSensors<4>& wall_sensors;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    /**
     * @brief The maze information the robot has
     */
    Maze<width, height> maze;

    /**
     * @brief Thickness of the maze walls
     */
    float wall_thickness;

    /**
     * @brief Size of the cells in the maze
     */
    float cell_size;

    /**
     * @brief Threshold for the robot alignment with the side walls
     */
    float alignment_threshold;

    /**
     * @brief Pose of the front wall sensors
     */
    Pose front_sensor_pose;

    /**
     * @brief Pose of the side wall sensors
     */
    Pose side_sensor_pose;

    /**
     * @brief Region of the cell to consider the information of the front sensors to the current cell
     */
    float front_sensors_region_division;

    /**
     * @brief Region of the cell to consider the information of the side sensors to the current cell
     */
    float side_sensors_region_division;

    /**
     * @brief Sensor readings tolerance for the front alignment of the robot
     */
    float front_distance_alignment_tolerance;

    /**
     * @brief Sensor readings tolerance for the side alignment of the robot
     */
    float side_distance_alignment_tolerance;

    /**
     * @brief Sensor readings for the front alignment of the robot at the front
     */
    float front_orientation_alignment_tolerance;

    /**
     * @brief Sensor readings for the front alignment of the robot with walls at the side
     */
    float side_orientation_alignment_tolerance;

    /**
     * @brief Sensor readings when the front of the robot is distance aligned
     */
    std::array<float, 2> front_distance_reading{};

    /**
     * @brief Sensor readings when the front of the robot is orientation aligned
     */
    std::array<float, 2> front_orientation_reading{};

    /**
     * @brief Sensor readings when the sides of the robot are distance aligned
     */
    std::array<float, 2> side_distance_reading{};
};
}  // namespace micras::nav

#include "../src/mapping.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAPPING_HPP
