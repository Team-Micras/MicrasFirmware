/**
 * @file
 */

#ifndef MICRAS_NAV_WALL_OBSERVER_HPP
#define MICRAS_NAV_WALL_OBSERVER_HPP

#include <array>
#include <cstdint>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/wall_model.hpp"

namespace micras::nav {
/**
 * @brief Decision of which walls exist, from the readings of the wall sensors.
 *
 * @details A reading only says something about a wall while the whole spot the emitter illuminates
 * would lie on the face of that wall, allowing for how uncertain the pose is, which the wall model
 * answers from the geometry of the sensors. Every reading taken in those conditions is a vote:
 * for the wall when something is seen at the range the wall would be at, against it when nothing is
 * seen up to beyond that range. A wall is decided when enough votes agree, and it is decided once.
 * A wall that never gets enough votes stays unknown rather than being guessed.
 *
 * @note Every sensor votes on whatever wall it happens to point at, so the wall ahead is decided by
 * the sensors that look forward as soon as it comes within their range, with no special case.
 *
 * @tparam width The width of the maze in cells.
 * @tparam height The height of the maze in cells.
 */
template <uint8_t width, uint8_t height>
class TWallObserver {
public:
    /**
     * @brief Configuration struct for the wall observer.
     *
     * @note The tolerance on the range is a constant part plus a part proportional to the range.
     * A wall can only be voted absent while it would be closer than the detection range, which has
     * to be inside of what the sensors can measure. The range delay is the delay of the filter the
     * fast ranges went through.
     */
    struct Config {
        float  tolerance;
        float  relative_tolerance;
        float  detection_range;
        float  max_angular_speed;
        float  range_delay;
        int8_t votes_to_decide;
    };

    /**
     * @brief Construct a new TWallObserver object.
     *
     * @param config The configuration for the wall observer.
     */
    explicit TWallObserver(const Config& config);

    /**
     * @brief Forget every vote.
     */
    void reset();

    /**
     * @brief Count the votes of the current readings and record the walls they decide.
     *
     * @param measurements The current measurements.
     * @param localizer The pose estimate and its uncertainty.
     * @param wall_model The geometry of the wall sensors.
     * @param maze The map of the walls, which is updated.
     * @return True if a wall was decided.
     */
    bool update(
        const Measurements& measurements, const Localizer& localizer, const WallModel& wall_model,
        TMaze<width, height>& maze
    );

private:
    /**
     * @brief Number of walls of the maze, including the ones of the border.
     */
    static constexpr uint16_t number_of_walls{(width + 1) * height + width * (height + 1)};

    /**
     * @brief Get the index of a wall.
     *
     * @param wall The cell and the side of it where the wall is.
     * @return The index of the wall, which is the same from both of its sides.
     */
    static constexpr uint16_t get_index(const GridPose& wall);

    /**
     * @brief Parameters of the observer.
     */
    Config config;

    /**
     * @brief Votes for each wall minus votes against it.
     */
    std::array<int8_t, number_of_walls> votes{};
};
}  // namespace micras::nav

#include "micras/nav/impl/wall_observer.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_WALL_OBSERVER_HPP
