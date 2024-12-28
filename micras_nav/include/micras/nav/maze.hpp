/**
 * @file
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
 * @brief Class for storing the robot information about the maze.
 *
 * @tparam width The width of the maze.
 * @tparam height The height of the maze.
 */
template <uint8_t width, uint8_t height>
class TMaze {
public:
    /**
     * @brief Construct a new Maze object.
     *
     * @param start The start pose of the robot.
     * @param goal The goal points in the maze.
     */
    TMaze(const GridPose& start, const std::unordered_set<GridPoint>& goal);

    /**
     * @brief Updates the maze walls with the current pose and new information.
     *
     * @param pose The pose of the robot.
     * @param information The information from the wall sensors.
     */
    void update(const GridPose& pose, Information information);

    /**
     * @brief Returns the next point the robot should go to when exploring.
     *
     * @param position The current position of the robot.
     * @return The next point the robot should go to when exploring.
     */
    GridPose get_current_exploration_goal(const GridPoint& position) const;

    /**
     * @brief Returns the next point the robot should go to when returning.
     *
     * @param position The current position of the robot.
     * @return The next point the robot should go to when returning.
     */
    GridPose get_current_returning_goal(const GridPoint& position) const;

    /**
     * @brief Checks the type of wall following the robot can do.
     *
     * @param pose The pose of the robot.
     * @return The type of wall following.
     */
    core::FollowWallType get_follow_wall_type(const GridPose& pose) const;

    /**
     * @brief Checks whether the robot has finished the maze.
     *
     * @param position The current position of the robot.
     * @return True if the robot has finished the maze, false otherwise.
     */
    bool finished(const GridPoint& position) const;

    /**
     * @brief Checks whether the robot has returned to the start.
     *
     * @param position The current position of the robot.
     * @return True if the robot has returned to the start, false otherwise.
     */
    bool returned(const GridPoint& position) const;

    /**
     * @brief Calculates the best route to the goal using the current costmap.
     */
    void calculate_best_route();

    /**
     * @brief Improves the route to the goal.
     */
    void optimize_route();

    /**
     * @brief Returns the best route to the goal.
     *
     * @return The best route to the goal.
     */
    const std::map<uint16_t, GridPose, std::greater<>>& get_best_route() const;

    /**
     * @brief Checks whether there is a wall at the front of a given pose.
     *
     * @param pose The pose to check.
     * @return True if there is a wall, false otherwise.
     */
    bool has_wall(const GridPose& pose) const;

private:
    /**
     * @brief Type to store the information of a cell in the maze.
     */
    struct Cell {
        std::array<uint32_t, 4> wall_count{};
        std::array<uint32_t, 4> free_count{};
        uint16_t                cost{0xFFFF};
    };

    /**
     * @brief Calculates the costmap for the flood fill algorithm.
     */
    void calculate_costmap();

    /**
     * @brief Returns the cell at the given position.
     *
     * @param position The position of the cell.
     * @return The cell at the given position.
     */
    const Cell& get_cell(const GridPoint& position) const;

    /**
     * @brief Returns the cell at the given position.
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
     * @return True if the information is new, false otherwise.
     */
    bool update_wall(const GridPose& pose, bool wall);

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
