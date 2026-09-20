/**
 * @file
 */

#ifndef MICRAS_NAV_SEGMENT_HPP
#define MICRAS_NAV_SEGMENT_HPP

#include <cstdint>

#include "micras/nav/lattice.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Kinds of motion a route is made of.
 */
enum class SegmentKind : uint8_t {
    STRAIGHT = 0,  // Straight line along the grid.
    DIAGONAL = 1,  // Straight line along a diagonal.
    TURN = 2,      // Slalom turn from the turn table, at a constant linear speed.
    SPIN = 3,      // Rotation in place.
    STOP = 4,      // Stand still for a given time.
    ATTACH = 5,    // Stand still facing a wall until the pose settles, or for a given time at most.
};

/**
 * @brief One motion of a route, as a plain value.
 *
 * @details The length is in meters for a straight and negative to drive it backwards, in radians for
 * a turn and for a rotation in place, positive to the left, and in seconds for the two kinds that
 * stand still. The speeds are those at the ends of the segment and are filled in by the velocity
 * planner, as is the largest speed allowed along it.
 *
 * @note The start is the pose the segment nominally starts from in the maze frame. It comes from the
 * route on the grid and never from a measurement, so the references generated from it do not move
 * when the pose estimate is corrected, and whatever error is left at the end of a segment shows up
 * as an error at the start of the next one instead of being forgotten.
 */
struct Segment {
    SegmentKind kind;
    TurnId      turn;
    float       length;
    float       start_speed;
    float       end_speed;
    float       max_speed;
    Pose        start;
};

/**
 * @brief What the robot should be doing at one instant.
 *
 * @note The acceleration is the exact derivative of the twist, which is what lets the feed forward
 * be computed without differentiating anything numerically.
 */
struct Reference {
    Pose  pose;
    Twist twist;
    Twist acceleration;
    float distance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_SEGMENT_HPP
