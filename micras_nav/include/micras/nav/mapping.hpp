/**
 * @file
 */

#ifndef MICRAS_NAV_MAPPING_HPP
#define MICRAS_NAV_MAPPING_HPP

#include <array>
#include <list>
#include <memory>

#include "micras/nav/maze.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/serializable.hpp"
#include "micras/proxy/wall_sensors.hpp"

namespace micras::nav {
/**
 * @brief Class for mapping the maze.
 *
 * @tparam width The width of the maze.
 * @tparam height The height of the maze.
 */
template <uint8_t width, uint8_t height>
class TMapping : public proxy::ISerializable {
public:
    /**
     * @brief Configuration struct for the Mapping class.
     */
    struct Config {
        float                wall_thickness{};
        float                cell_size{};
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
     * @brief Type to identify the wall sensors.
     */
    enum Sensor : uint8_t {
        FRONT_LEFT,
        LEFT,
        RIGHT,
        FRONT_RIGHT
    };

    /**
     * @brief Type to store an action to be taken.
     */
    struct Action {
        /**
         * @brief Possible types of actions.
         */
        enum Type : uint8_t {
            LOOK_AT,     // Rotate the robot to look at a specific point.
            GO_TO,       // Move and turn the robot to go to a specific point.
            ALIGN_BACK,  // Align the back of the robot with a wall.
            FINISHED,    // The robot has reached the goal.
            ERROR,       // An error occurred.
        };

        /**
         * @brief Type of the action.
         */
        Type type;

        /**
         * @brief The next goal point.
         */
        Point point;

        /**
         * @brief The orientation of the goal point.
         */
        Direction direction;
    };

    /**
     * @brief Construct a new Mapping object.
     *
     * @param wall_sensors The wall sensors.
     * @param config The configuration for the mapping.
     */
    TMapping(const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, Config config);

    /**
     * @brief Update the mapping of the maze using the current pose and wall sensors.
     *
     * @param pose The current pose of the robot.
     */
    void update(const Pose& pose);

    /**
     * @brief Get the action to be taken based on the current pose.
     *
     * @param pose The current pose of the robot.
     * @param objective The curent objective of the robot.
     * @return The action to be taken.
     */
    Action get_action(const Pose& pose, core::Objective objective);

    /**
     * @brief Fix the pose based on the mapping information.
     *
     * @param pose The current pose of the robot.
     * @param follow_wall_type The type of wall following being performed.
     * @return The fixed pose.
     */
    Pose correct_pose(const Pose& pose, core::FollowWallType follow_wall_type) const;

    /**
     * @brief Calibrate front alignment of the wall sensors.
     */
    void calibrate_front();

    /**
     * @brief Calibrate side alignment of the wall sensors.
     */
    void calibrate_side();

    /**
     * @brief Check if the robot is at a predefined distance from the wall at the front.
     *
     * @return True if the robot is aligned, false otherwise.
     */
    bool is_distance_front_aligned() const;

    /**
     * @brief Check if the robot orientation is aligned with walls at the front.
     *
     * @return True if the robot is aligned, false otherwise.
     */
    bool is_orientation_front_aligned() const;

    /**
     * @brief Check if the robot is at a predefined distance from the wallz at the sides.
     *
     * @return True if the robot is aligned, false otherwise.
     */
    bool is_distance_side_aligned() const;

    /**
     * @brief Check if the robot orientation is aligned with walls at the sides.
     *
     * @return True if the robot is aligned, false otherwise.
     */
    bool is_orientation_side_aligned() const;

    /**
     * @brief Get the type of wall following the robot can do.
     *
     * @param pose The current pose of the robot.
     * @return The type of wall following.
     */
    core::FollowWallType get_follow_wall_type(const Pose& pose) const;

    /**
     * @brief Serialize the best route to the goal.
     *
     * @return The serialized data.
     */
    std::vector<uint8_t> serialize() const override;

    /**
     * @brief Deserialize the best route to the goal.
     *
     * @param buffer The serialized data.
     * @param size The size of the serialized data.
     */
    void deserialize(const uint8_t* buffer, uint16_t size) override;

    /**
     * @brief Check if the robot can align its back with a wall.
     *
     * @param pose The current pose of the robot.
     * @return True if the robot can align its back, false otherwise.
     */
    bool can_align_back(const Pose& pose) const;

    /**
     * @brief Add diagonal movements to the best route.
     */
    void diagonalize_best_route();

private:
    /**
     * @brief Wall sensors of the robot.
     */
    std::shared_ptr<proxy::TWallSensors<4>> wall_sensors;

    /**
     * @brief The maze information the robot has.
     */
    TMaze<width, height> maze;

    /**
     * @brief Thickness of the maze walls.
     */
    float wall_thickness;

    /**
     * @brief Size of the cells in the maze.
     */
    float cell_size;

    /**
     * @brief Pose of the front wall sensors.
     */
    Pose front_sensor_pose;

    /**
     * @brief Pose of the side wall sensors.
     */
    Pose side_sensor_pose;

    /**
     * @brief Region of the cell to consider the information of the front sensors to the current cell.
     */
    float front_sensors_region_division;

    /**
     * @brief Region of the cell to consider the information of the side sensors to the current cell.
     */
    float side_sensors_region_division;

    /**
     * @brief Sensor readings tolerance for the front alignment of the robot.
     */
    float front_distance_alignment_tolerance;

    /**
     * @brief Sensor readings tolerance for the side alignment of the robot.
     */
    float side_distance_alignment_tolerance;

    /**
     * @brief Sensor readings for the front alignment of the robot at the front.
     */
    float front_orientation_alignment_tolerance;

    /**
     * @brief Sensor readings for the front alignment of the robot with walls at the side.
     */
    float side_orientation_alignment_tolerance;

    /**
     * @brief Sensor readings when the front of the robot is distance aligned.
     */
    std::array<float, 2> front_distance_reading{};

    /**
     * @brief Sensor readings when the front of the robot is orientation aligned.
     */
    std::array<float, 2> front_orientation_reading{};

    /**
     * @brief Sensor readings when the sides of the robot are distance aligned.
     */
    std::array<float, 2> side_distance_reading{};

    /**
     * @brief List of points and directions that represent the best route to the goal.
     */
    std::list<std::pair<Point, Direction>> best_route;

    /**
     * @brief Current iterator of the best route.
     */
    std::list<std::pair<Point, Direction>>::iterator best_route_iterator{best_route.begin()};
};
}  // namespace micras::nav

#include "../src/mapping.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAPPING_HPP
