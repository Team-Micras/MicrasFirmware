/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_HPP
#define MICRAS_NAV_MAZE_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "micras/core/serializable.hpp"
#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
/**
 * @brief What is known about a wall.
 */
enum class WallState : uint8_t {
    UNKNOWN = 0,
    NO_WALL = 1,
    WALL = 2,
};

/**
 * @brief Map of the walls of the maze, with the flood fill that guides the search.
 *
 * @tparam width The width of the maze in cells.
 * @tparam height The height of the maze in cells.
 */
template <uint8_t width, uint8_t height>
class TMaze : public core::ISerializable {
public:
    /**
     * @brief Cost of a cell that cannot reach any target.
     */
    static constexpr uint16_t unreachable{0xFFFF};

    /**
     * @brief Configuration struct for the maze.
     *
     * @note The goal is borrowed, so it has to outlive the maze, which a constant of the
     * configuration does.
     */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init) no defaults, so that a missing field is a warning
    struct Config {
        GridPose                   start;
        std::span<const GridPoint> goal;
    };

    /**
     * @brief Construct a new TMaze object.
     *
     * @param config The configuration for the maze.
     */
    explicit TMaze(const Config& config);

    /**
     * @brief Forget every wall that was observed, keeping what the rules guarantee.
     *
     * @note The rules give the border, the walls around the start cell and the absence of walls
     * between the cells of the goal.
     */
    void reset();

    /**
     * @brief Check if a cell is inside the maze.
     *
     * @param position The position of the cell.
     * @return True if the cell exists.
     */
    static constexpr bool contains(const GridPoint& position) { return position.x < width and position.y < height; }

    /**
     * @brief Get what is known about the wall ahead of a pose.
     *
     * @param pose The cell and the side of it.
     * @return The state of the wall, which is a wall for anything outside of the maze.
     */
    WallState get_wall(const GridPose& pose) const;

    /**
     * @brief Record the observation of a wall that was unknown.
     *
     * @note A wall is decided once: an observation of a wall that is already known changes nothing.
     *
     * @param pose The cell and the side of it.
     * @param present Whether there is a wall.
     * @return True if the map changed.
     */
    bool set_wall(const GridPose& pose, bool present);

    /**
     * @brief Check if the robot cannot go through, as far as it is known.
     *
     * @param pose The cell and the side of it.
     * @return True if there is a wall.
     */
    bool is_blocked(const GridPose& pose) const;

    /**
     * @brief Check if the robot might go through, which is what the search assumes.
     *
     * @param pose The cell and the side of it.
     * @return True unless a wall was seen.
     */
    bool is_possibly_open(const GridPose& pose) const;

    /**
     * @brief Check if the robot can go through for certain, which is what a fast run requires.
     *
     * @param pose The cell and the side of it.
     * @return True if the absence of a wall was seen.
     */
    bool is_known_open(const GridPose& pose) const;

    /**
     * @brief Check if a cell belongs to the goal.
     *
     * @param position The position of the cell.
     * @return True if the cell is part of the goal.
     */
    bool is_goal(const GridPoint& position) const;

    /**
     * @brief Get the pose the robot starts from.
     *
     * @return The start cell and the direction the robot faces in it.
     */
    const GridPose& get_start() const;

    /**
     * @brief Get the cells of the goal.
     *
     * @return The cells of the goal.
     */
    std::span<const GridPoint> get_goal() const;

    /**
     * @brief Get a number that changes every time a wall is recorded.
     *
     * @return The number of changes since the map was last reset.
     */
    uint32_t get_revision() const;

    /**
     * @brief Compute the number of cells from every cell to the nearest target.
     *
     * @note Walls that were not seen count as absent. The whole maze is flooded again on every
     * call, from a queue with room for every cell, so the cost is bounded and nothing is allocated.
     *
     * @param targets The cells with a cost of zero.
     */
    void flood(std::span<const GridPoint> targets);

    /**
     * @brief Get the number of cells from a cell to the nearest target of the last flood.
     *
     * @param position The position of the cell.
     * @return The cost of the cell, or unreachable.
     */
    uint16_t get_cost(const GridPoint& position) const;

    /**
     * @brief Get the neighbor to move to in order to approach the targets of the last flood.
     *
     * @note Going straight is preferred to turning and turning to going back, among the neighbors
     * that are equally close.
     *
     * @param pose The current cell and the direction the robot faces.
     * @return The neighbor and the direction to reach it, or nothing if no target can be reached.
     */
    std::optional<GridPose> get_next(const GridPose& pose) const;

    /**
     * @brief Serialize the walls of the maze.
     *
     * @return The version, the size of the maze and the state of every wall.
     */
    std::vector<uint8_t> serialize() const override;

    /**
     * @brief Load the walls of the maze.
     *
     * @note A record of another version or size is ignored.
     *
     * @param buffer The serialized data.
     * @param size The size of the serialized data.
     */
    void deserialize(const uint8_t* buffer, uint16_t size) override;

private:
    /**
     * @brief Version of the serialized format.
     */
    static constexpr uint8_t format_version{1};

    /**
     * @brief Number of bytes of the serialized walls, at two bits for each of two sides of a cell.
     */
    static constexpr uint16_t walls_size{(width * height + 1) / 2};

    /**
     * @brief Number of bytes of the header of the serialized format.
     */
    static constexpr uint16_t header_size{3};

    /**
     * @brief Write the state of a wall on both of the cells it belongs to.
     *
     * @param pose The cell and the side of it.
     * @param state The state of the wall.
     */
    void write_wall(const GridPose& pose, WallState state);

    /**
     * @brief State of the walls of every cell, indexed by row, column and side.
     */
    std::array<std::array<std::array<WallState, 4>, width>, height> walls{};

    /**
     * @brief Number of cells from every cell to the nearest target of the last flood.
     */
    std::array<std::array<uint16_t, width>, height> costs{};

    /**
     * @brief Queue of the flood fill, with room for every cell.
     */
    std::array<GridPoint, static_cast<std::size_t>(width) * height> queue{};

    /**
     * @brief Pose the robot starts from.
     */
    GridPose start;

    /**
     * @brief Cells of the goal.
     */
    std::span<const GridPoint> goal;

    /**
     * @brief Number of walls recorded since the last reset.
     */
    uint32_t revision{};
};
}  // namespace micras::nav

#include "micras/nav/impl/maze.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_MAZE_HPP
