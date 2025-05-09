/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_HPP
#define MICRAS_NAV_MAZE_HPP

#include <cstdint>
#include <map>
#include <unordered_set>

#include "micras/core/serializable.hpp"
#include "micras/core/types.hpp"
#include "micras/nav/costmap.hpp"
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
    struct Config {
        GridPose                      start{};
        std::unordered_set<GridPoint> goal;
        uint16_t                      cost_margin{};
    };

    /**
     * @brief Construct a new Maze object.
     */
    explicit TMaze(Config config);

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
     * @param returning Whether the robot is returning to the start position.
     * @return The next point the robot should go to when exploring.
     */
    GridPose get_next_goal(const GridPose& pose, bool returning, bool discover = true) const;

    /**
     * @brief Detects walls around the robot at a given grid pose.
     *
     * @param pose The robot’s current grid pose.
     * @return The walls that are present around the robot.
     */
    core::Observation get_observation(const GridPose& pose) const;

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

    void enable_discovery_costmap();

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
    enum Layer : uint8_t {
        EXPLORE = 0,
        RETURN = 1,
        DISCOVER = 2,
        NUM_OF_LAYERS = 3,
    };

    /**
     * @brief Calculate the costmap for the flood fill algorithm.
     */
    void compute_costmap(const GridPoint& reference);

    static bool is_dead_end(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell);

    static bool was_visited(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell);

    static bool must_visit(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell, int16_t cost_threshold);

    void update_cell(const GridPoint& position);

    Costmap<width, height, Layer::NUM_OF_LAYERS> costmap;

    /**
     * @brief Start pose of the robot in the maze.
     */
    GridPose start;

    /**
     * @brief Goal points in the maze.
     */
    std::unordered_set<GridPoint> goal;

    /**
     * @brief Cost margin for the robot to explore.
     */
    uint16_t cost_margin;

    /**
     * @brief Minimum cost of path to the goal containing only visited cells.
     */
    int16_t minimum_cost{};

    bool discovery_enabled{false};

    /**
     * @brief Current best found route to the goal.
     */
    std::map<uint16_t, GridPose, std::greater<>> best_route;
};
}  // namespace micras::nav

#include "../src/maze.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_MAZE_HPP
