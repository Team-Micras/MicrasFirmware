/**
 * @file maze.hpp
 *
 * @brief Nav Maze class declaration
 *
 * @date 10/2024
 */

#ifndef MICRAS_NAV_MAZE_HPP
#define MICRAS_NAV_MAZE_HPP

#include <array>
#include <cstdint>
#include <map>
#include <unordered_set>

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
/**
 * @brief Class for storing the robot information about the maze
 *
 * @tparam width The width of the maze
 * @tparam height The height of the maze
 */
template <uint8_t width, uint8_t height>
class Maze {
public:
    /**
     * @brief Construct a new Maze object
     *
     * @param start The start pose of the robot
     * @param goal The goal points in the maze
     */
    Maze(const GridPose& start, std::unordered_set<GridPoint> goal);

    /**
     * @brief Updates the maze walls with the current pose and new information
     *
     * @param pose The pose of the robot
     * @param information The information from the distance sensors
     */
    void update(const GridPose& pose, Information information);

    /**
     * @brief Returns the next point the robot should go to
     *
     * @param position The current position of the robot
     * @param force_costmap Whether to force the use of the update of the costmap
     * @return The next point the robot should go to
     */
    GridPoint get_current_goal(const GridPoint& position, bool force_costmap = false) const;

private:
    /**
     * @brief Type to store the information of a cell in the maze
     */
    struct Cell {
        std::array<uint32_t, 4> wall_count{};
        std::array<uint32_t, 4> free_count{};
        uint16_t                cost{0xFFFF};
    };

    /**
     * @brief Calculates the costmap for the flood fill algorithm
     */
    void calculate_costmap();

    /**
     * @brief Returns the cell at the given position
     *
     * @param position The position of the cell
     * @return The cell at the given position
     */
    const Cell& get_cell(const GridPoint& position) const;

    /**
     * @brief Returns the cell at the given position
     *
     * @param position The position of the cell
     * @return The cell at the given position
     */
    Cell& get_cell(const GridPoint& position);

    /**
     * @brief Update the probability of a wall in the maze
     *
     * @param pose The pose of the robot
     * @param wall Whether there is a wall in front of the robot
     */
    void update_wall(const GridPose& pose, bool wall);

    /**
     * @brief Checks whether there is a wall at the front of a given pose
     *
     * @param pose The pose to check
     * @return True if there is a wall, false otherwise
     */
    bool has_wall(const GridPose& pose) const;

    /**
     * @brief Cells matrix representing the maze
     */
    std::array<std::array<Cell, width>, height> cells{};

    /**
     * @brief Start pose of the robot in the maze
     */
    GridPose start;

    /**
     * @brief Goal points in the maze
     */
    std::unordered_set<GridPoint> goal;

    /**
     * @brief Whether the robot is returning to the start
     */
    bool returning{};

    /**
     * @brief Whether the robot is exploring the maze
     */
    bool exploring{true};

    /**
     * @brief Current best found route to the goal
     */
    std::map<uint16_t, GridPoint, std::greater<>> best_route;
};
}  // namespace micras::nav

#include "../src/maze.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAZE_HPP
