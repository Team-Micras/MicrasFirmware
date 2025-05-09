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
 * @brief Class for storing the robot information about the maze.
 *
 * @tparam width The width of the maze.
 * @tparam height The height of the maze.
 */
template <uint8_t width, uint8_t height, uint8_t layers>
class Costmap {
public:
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

    Costmap();

    /**
     * @brief Calculate the costmap for the flood fill algorithm.
     */
    void compute(const GridPoint& reference, uint8_t layer);

    /**
     * @brief Return the cell at the given position.
     *
     * @param position The position of the cell.
     * @return The cell at the given position.
     */
    const Cell& get_cell(const GridPoint& position) const;

    int16_t get_cost(const GridPoint& position, uint8_t layer) const;

    void update_cost(const GridPoint& position, uint8_t layer, int16_t cost);

    /**
     * @brief Update the existence of a wall in the maze.
     *
     * @param pose The pose of the robot.
     * @param wall Whether there is a wall at the front of a given pose.
     */
    bool update_wall(const GridPose& pose, bool wall);

    /**
     * @brief Check whether there is a wall at the front of a given pose.
     *
     * @param pose The pose to check.
     * @return True if there is a wall, false otherwise.
     */
    bool has_wall(const GridPose& pose) const;

    void recompute(const GridPoint& reference, uint8_t layer);

    void add_virtual_wall(const GridPose& pose);

private:
    Cell& get_cell(const GridPoint& position);

    /**
     * @brief Cells matrix representing the maze.
     */
    std::array<std::array<Cell, width>, height> cells{};
};
}  // namespace micras::nav

#include "../src/costmap.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_NAV_COSTMAP_HPP
