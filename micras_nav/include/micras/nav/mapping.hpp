/**
 * @file mapping.hpp
 *
 * @brief Nav Mapping class declaration
 *
 * @date 10/2024
 */

#ifndef MICRAS_NAV_MAPPING_HPP
#define MICRAS_NAV_MAPPING_HPP

#include <array>

#include "micras/nav/maze.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/distance_sensors.hpp"

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
        float wall_thickness;
        float cell_size;
        float wall_distance_threshold;
        float free_distance_threshold;
        float alignment_threshold;
        Pose  front_sensor_pose;
        Pose  side_sensor_pose;

        GridPose                      start;
        std::unordered_set<GridPoint> goal{
            {{width / 2, height / 2},
             {(width - 1) / 2, height / 2},
             {width / 2, (height - 1) / 2},
             {(width - 1) / 2, (height - 1) / 2}}
        };
    };

    /**
     * @brief Type to identify the distance sensors
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
    };

    /**
     * @brief Constructor for the Mapping class
     *
     * @param distance_sensors The distance sensors
     * @param config The configuration for the mapping
     */
    Mapping(const proxy::DistanceSensors<4>& distance_sensors, Config config);

    /**
     * @brief Update the mapping of the maze using the current pose and distance sensors
     *
     * @param pose The current pose of the robot
     */
    void update(const Pose& pose);

    /**
     * @brief Get the action to take based on the current pose
     *
     * @param pose The current pose of the robot
     * @return The action to take
     */
    Action get_action(const Pose& pose) const;

private:
    /**
     * @brief Get the wall information for a sensor
     *
     * @param sensor The sensor to get the wall information
     * @return The wall information for the sensor
     */
    Information::Existence get_wall_information(Sensor sensor) const;

    /**
     * @brief Distance sensors of the robot
     */
    const proxy::DistanceSensors<4>& distance_sensors;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

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
     * @brief Threshold for the distance to consider a wall
     */
    float wall_distance_threshold;

    /**
     * @brief Threshold for the distance to consider free space
     */
    float free_distance_threshold;

    /**
     * @brief Threshold for the robot alignment with the side walls
     */
    float alignment_threshold;

    /**
     * @brief Pose of the front distance sensors
     */
    Pose front_sensor_pose;

    /**
     * @brief Pose of the side distance sensors
     */
    Pose side_sensor_pose;

    /**
     * @brief Region of the cell to consider the information of the side sensors to the current cell
     */
    float side_sensors_region_division;

    /**
     * @brief Region of the cell to consider the information of the front sensors to the current cell
     */
    float front_sensors_region_division;
};
}  // namespace micras::nav

#include "../src/mapping.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAPPING_HPP
