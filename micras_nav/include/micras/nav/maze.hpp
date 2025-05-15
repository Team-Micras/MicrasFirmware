/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_HPP
#define MICRAS_NAV_MAZE_HPP

#include <cstdint>
#include <list>
#include <unordered_set>

#include "micras/core/serializable.hpp"
#include "micras/core/types.hpp"
#include "micras/nav/costmap.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze_graph.hpp"

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
    struct Config {
        GridPose                      start{};
        std::unordered_set<GridPoint> goal;
        float                         cost_margin{};
    };

    /**
     * @brief Construct a new Maze object.
     */
    explicit TMaze(
        Config           config,
        EdgeCostFunction edge_cost_function = [](const GridPose& from, const GridPose& to) -> float {
            float edge_cost = std::abs(from.position.x - to.position.x) + std::abs(from.position.y - to.position.y);
            return edge_cost > 1.0F ? 0.0F : edge_cost;
        }
    );

    /**
     * @brief Update the maze walls with the current pose and new information.
     *
     * @param pose The pose of the robot.
     * @param observation The observation from the wall sensors.
     */
    void update_walls(const GridPose& pose, const core::Observation& observation);

    /**
     * @brief Return the next point the robot should go based on the costmap.
     *
     * @param position The current position of the robot.
     * @param returning Whether the robot is returning to the start position.
     * @return The next point the robot should go.
     */
    GridPose get_next_goal(const GridPose& pose, bool returning);

    /**
     * @brief Check whether the robot has finished the maze.
     *
     * @param position The current position of the robot.
     * @param returning Whether the robot is returning to the start position.
     * @return True if the robot has finished the maze, false otherwise.
     */
    bool finished(const GridPoint& position, bool returning) const;

    /**
     * @brief Calculate the best route to the goal using the current costmap.
     */
    void compute_best_route();

    /**
     * @brief Return the best route to the goal.
     *
     * @return The best route to the goal.
     */
    const std::list<GridPose>& get_best_route() const;

    /**
     * @brief Compute the maze graph.
     */
    void compute_graph();

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
     * @brief The layers of the costmap.
     */
    enum Layer : uint8_t {
        EXPLORE = 0,
        RETURN = 1,
        NUM_OF_LAYERS = 2,
    };

    /**
     * @brief Compute the minumum cost from the start to the end considering only discoverd cells.
     */
    void compute_minimum_cost();

    /**
     * @brief Update the cell costs at the given position.
     *
     * @param position The position of the cell.
     */
    void update_cell(const GridPoint& position);

    /**
     * @brief Get the next goal for the robot using a BFS algorithm.
     *
     * @param pose The current pose of the robot.
     * @param discover Whether the robot is discovering new cells.
     * @return A pair containing the next discovery goal for the robot and the total distance.
     */
    std::pair<GridPose, uint16_t> get_next_bfs_goal(const GridPose& pose, bool discover) const;

    /**
     * @brief Check if the cell is a dead end.
     *
     * @param cell The cell to check.
     * @return True if the cell is a dead end, false otherwise.
     */
    static bool is_dead_end(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell);

    /**
     * @brief Check if the cell was visited.
     *
     * @param cell The cell to check.
     * @return True if the cell was visited, false otherwise.
     */
    static bool was_visited(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell);

    /**
     * @brief Check if the cell must be visited.
     *
     * @param cell The cell to check.
     * @param cost_threshold The cost threshold for the cell.
     * @return True if the cell must be visited, false otherwise.
     */
    static bool must_visit(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell, int16_t cost_threshold);

    /**
     * @brief Layered costmap for the maze.
     */
    Costmap<width, height, Layer::NUM_OF_LAYERS> costmap;

    /**
     * @brief Graph of the maze for computing the best route.
     */
    MazeGraph graph;

    /**
     * @brief Start pose of the robot in the maze.
     */
    GridPose start;

    /**
     * @brief Goal points in the maze.
     */
    std::unordered_set<GridPoint> goal;

    /**
     * @brief Cost margin above the minimum cost that the robot should explore.
     */
    float cost_margin;

    /**
     * @brief Minimum cost of path to the goal containing only visited cells.
     */
    int16_t minimum_cost{};

    /**
     * @brief Flag indicating whether the robot has finished discovering the cells of the maze.
     */
    bool finished_discovery{false};

    /**
     * @brief Current best found route to the goal.
     */
    std::list<GridPose> best_route;
};
}  // namespace micras::nav

#include "../src/maze.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAZE_HPP
