/**
 * @file
 */

#ifndef MICRAS_NAV_COSTMAP_HPP
#define MICRAS_NAV_COSTMAP_HPP

#include <array>
#include <cstdint>

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
constexpr int16_t max_cost{0x7FFF};

/**
 * @brief Class for managing a layered costmap.
 *
 * @tparam width The width of the maze.
 * @tparam height The height of the maze.
 * @tparam layers The number of layers in the costmap.
 */
template <uint8_t width, uint8_t height, uint8_t layers>
class Costmap {
public:
    /**
     * @brief Type to store the state of a wall.
     */
    enum WallState : uint8_t {
        UNKNOWN = 0,
        NO_WALL = 1,
        WALL = 2,
        VIRTUAL = 3,
    };

    /**
     * @brief Type to store the information of a cell in the maze.
     */
    struct Cell {
        std::array<WallState, 4>    walls{WallState::UNKNOWN};
        std::array<int16_t, layers> costs{max_cost};
    };

    /**
     * @brief Construct a new Costmap object.
     */
    Costmap();

    /**
     * @brief Compute the costmap of a given layer from a reference point using the flood fill algorithm.
     *
     * @param reference The reference point to start the flood fill algorithm.
     * @param layer The layer to compute the costmap for.
     */
    void compute(const GridPoint& reference, uint8_t layer);

    /**
     * @brief Reset the costs and recompute the new values locally for a given layer.
     *
     * @param reference The reference point to start resetting the stored cost.
     * @param layer The layer to recompute.
     */
    void recompute(const GridPoint& reference, uint8_t layer);

    /**
     * @brief Return the cell at the given position.
     *
     * @param position The position of the cell.
     * @return The cell at the given position.
     */
    const Cell& get_cell(const GridPoint& position) const;

    /**
     * @brief Get the cost of a cell at a given position and layer.
     *
     * @param position The position of the cell.
     * @param layer The layer to get the cost for.
     * @return The cost of the cell at the given position and layer.
     */
    int16_t get_cost(const GridPoint& position, uint8_t layer) const;

    /**
     * @brief Update the cost of a cell at a given position and layer.
     *
     * @param position The position of the cell.
     * @param layer The layer to update the cost for.
     * @param cost The new cost to set.
     */
    void update_cost(const GridPoint& position, uint8_t layer, int16_t cost);

    /**
     * @brief Check whether there is a wall at the front of a given pose.
     *
     * @param pose The pose to check.\
     * @param consider_virtual Whether to consider virtual walls.
     * @return True if there is a wall, false otherwise.
     */
    bool has_wall(const GridPose& pose, bool consider_virtual = false) const;

    /**
     * @brief Update the existence of a wall in the maze.
     *
     * @param pose The pose of the robot.
     * @param wall Whether there is a wall at the front of a given pose.
     */
    bool update_wall(const GridPose& pose, bool wall);

    /**
     * @brief Add a virtual wall to the costmap at a given position and side.
     *
     * @param pose The pose to add the virtual wall at.
     */
    void add_virtual_wall(const GridPose& pose);

private:
    /**
     * @brief Get the cell at the given position.
     *
     * @param position The position of the cell.
     * @return The cell at the given position.
     */
    Cell& cell_on_position(const GridPoint& position);

    /**
     * @brief Cells matrix representing the maze.
     */
    std::array<std::array<Cell, width>, height> cells{};
};
}  // namespace micras::nav

#include "../src/costmap.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_COSTMAP_HPP
