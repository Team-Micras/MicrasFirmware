/**
 * @file
 */

#ifndef MICRAS_NAV_EXPLORER_HPP
#define MICRAS_NAV_EXPLORER_HPP

#include <array>
#include <cstdint>
#include <span>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/planner.hpp"

namespace micras::nav {
/**
 * @brief Choice of what is still worth visiting, and of when the search is over.
 *
 * @details The fastest route that could exist is the one the planner finds when every wall that was
 * never seen is taken as absent. If that route crosses no unknown wall, the map already proves that
 * nothing faster exists and the search is over. Otherwise the cells next to the unknown walls it
 * crosses are what is worth visiting, and the question is asked again once the map has changed. The
 * test uses the very cost the fast run is chosen by, so a faster route cannot stay hidden, and only
 * cells on some best conceivable route are ever targeted.
 *
 * The fastest route depends on the profile of the run, so the test is repeated for every profile the
 * map has to be good for and the targets are put together.
 *
 * @note The planner is advanced a bounded number of nodes per call, which lets all of this happen
 * while the robot is moving.
 *
 * @tparam width The width of the maze in cells.
 * @tparam height The height of the maze in cells.
 */
template <uint8_t width, uint8_t height>
class TExplorer {
public:
    /**
     * @brief How much slower than the best a candidate route can be and still be looked at.
     *
     * @note The search prices a route to within a few hundredths of a second of what the velocity
     * planner will make of it, so a candidate this close to the best may turn out to be the best.
     */
    static constexpr float candidate_window{0.05F};

    /**
     * @brief Largest number of cells that can be targeted at once.
     */
    static constexpr uint8_t max_targets{64};

    /**
     * @brief Construct a new TExplorer object.
     *
     * @param planner The planner, borrowed for the lifetime of the explorer and shared with whoever
     * plans the fast run, which never happens at the same time.
     * @param profiles The profiles the map has to be good for, borrowed for the same time.
     */
    TExplorer(TPlanner<width, height>& planner, std::span<const RunProfile> profiles);

    /**
     * @brief Forget the targets and start asking again.
     */
    void reset();

    /**
     * @brief Advance the planning.
     *
     * @param maze The map of the walls.
     * @param max_nodes The largest number of nodes the planner may expand in this call.
     * @return True if the targets changed.
     */
    bool update(const TMaze<width, height>& maze, uint32_t max_nodes);

    /**
     * @brief Check if an answer for the current map exists yet.
     *
     * @return True once every profile was planned at least once since the last reset.
     */
    bool has_targets() const;

    /**
     * @brief Check if the map proves that the fastest route of every profile is known.
     *
     * @return True if nothing is left that is worth visiting.
     */
    bool is_complete() const;

    /**
     * @brief Get the cells worth visiting.
     *
     * @return The cells next to the unknown walls of the fastest conceivable routes.
     */
    std::span<const GridPoint> get_targets() const;

private:
    /**
     * @brief Add the cells on both sides of a wall to the targets being collected.
     *
     * @param wall The cell and the side of it where the wall is.
     */
    void add_target(const GridPose& wall);

    /**
     * @brief Planner used to find the fastest conceivable routes.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    TPlanner<width, height>& planner;

    /**
     * @brief Profiles the map has to be good for.
     */
    std::span<const RunProfile> profiles;

    /**
     * @brief Route being inspected, kept to reuse its storage.
     */
    Route route;

    /**
     * @brief Cells worth visiting, as of the last complete round.
     */
    std::array<GridPoint, max_targets> targets{};

    /**
     * @brief Number of cells worth visiting.
     */
    uint8_t number_of_targets{};

    /**
     * @brief Cells collected by the round in progress.
     */
    std::array<GridPoint, max_targets> collected{};

    /**
     * @brief Number of cells collected by the round in progress.
     */
    uint8_t number_collected{};

    /**
     * @brief Index of the profile being planned.
     */
    uint8_t profile_index{};

    /**
     * @brief Whether the planner is in the middle of a search of this explorer.
     */
    bool planning{};

    /**
     * @brief Whether a round was completed since the last reset.
     */
    bool answered{};

    /**
     * @brief Revision of the map the round in progress started from.
     */
    uint32_t round_revision{};

    /**
     * @brief Revision of the map the current targets were computed for.
     */
    uint32_t answered_revision{};
};
}  // namespace micras::nav

#include "micras/nav/impl/explorer.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_EXPLORER_HPP
