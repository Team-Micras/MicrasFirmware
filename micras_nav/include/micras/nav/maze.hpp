/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_HPP
#define MICRAS_NAV_MAZE_HPP

#include <array>
#include <cstdint>
#include <map>
#include <unordered_set>

#include "micras/core/serializable.hpp"
#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
/**
 * @brief Class for storing the robot information about the maze.
 *
 * @tparam width The width of the maze.
 * @tparam height The height of the maze.
 */
template <uint8_t width, uint8_t height>
class TMaze : public core::ISerializable {
public:
    /**
     * @brief Construct a new Maze object.
     *
     * @param start The start pose of the robot.
     * @param goal The goal points in the maze.
     */
    TMaze(const GridPose& start, const std::unordered_set<GridPoint>& goal);

    /**
     * @brief Update the maze walls with the current pose and new information.
     *
     * @param pose The pose of the robot.
     * @param observation The observation from the wall sensors.
     */
    void update_walls(const GridPose& pose, const core::Observation& observation);

    /**
     * @brief Return the next point the robot should go to when exploring.
     *
     * @param position The current position of the robot.
     * @return The next point the robot should go to when exploring.
     */
    GridPoint get_next_goal(const GridPoint& position, bool returning) const;

    /**
     * @brief Check the type of wall following the robot can do.
     *
     * @param pose The pose of the robot.
     * @return The type of wall following.
     */
    core::Observation get_observation(const GridPose& pose) const;

    /**
     * @brief Check whether the robot has finished the maze.
     *
     * @param position The current position of the robot.
     * @return True if the robot has finished the maze, false otherwise.
     */
    bool finished(const GridPoint& position, bool returning) const;

    /**
     * @brief Calculate the best route to the goal using the current costmap.
     */
    void calculate_best_route();

    /**
     * @brief Return the best route to the goal.
     *
     * @return The best route to the goal.
     */
    const std::map<uint16_t, GridPose, std::greater<>>& get_best_route() const;

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

private:
    /**
     * @brief Type to store the information of a cell in the maze.
     */
    struct Cell {
        std::array<bool, 4> walls{};
        uint16_t            cost{0xFFFF};
    };

    /**
     * @brief Calculate the costmap for the flood fill algorithm.
     */
    void calculate_costmap();

    /**
     * @brief Return the cell at the given position.
     *
     * @param position The position of the cell.
     * @return The cell at the given position.
     */
    const Cell& get_cell(const GridPoint& position) const;

    /**
     * @brief Return the cell at the given position.
     *
     * @param position The position of the cell.
     * @return The cell at the given position.
     */
    Cell& get_cell(const GridPoint& position);

    /**
     * @brief Update the probability of a wall in the maze.
     *
     * @param pose The pose of the robot.
     * @param wall Whether there is a wall in front of the robot.
     */
    void update_wall(const GridPose& pose, bool wall);

    /**
     * @brief Check whether there is a wall at the front of a given pose.
     *
     * @param pose The pose to check.
     * @return True if there is a wall, false otherwise.
     */
    bool has_wall(const GridPose& pose) const;

    /**
     * @brief Cells matrix representing the maze.
     */
    std::array<std::array<Cell, width>, height> cells{};

    /**
     * @brief Start pose of the robot in the maze.
     */
    GridPose start;

    /**
     * @brief Goal points in the maze.
     */
    std::unordered_set<GridPoint> goal;

    /**
     * @brief Current best found route to the goal.
     */
    std::map<uint16_t, GridPose, std::greater<>> best_route;
};
}  // namespace micras::nav

#include "../src/maze.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAZE_HPP
