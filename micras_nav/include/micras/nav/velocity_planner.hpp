/**
 * @file
 */

#ifndef MICRAS_NAV_VELOCITY_PLANNER_HPP
#define MICRAS_NAV_VELOCITY_PLANNER_HPP

#include <span>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"

namespace micras::nav {
/**
 * @brief Choice of the speed at every junction of a route.
 *
 * @details Every turn starts at the speed its shape allows and every straight at the speed limit.
 * A pass from the end to the start then lowers each junction to what can be braked from before the
 * next one, and a pass from the start to the end lowers it to what can be reached from the previous
 * one. A turn is driven at a single speed, so lowering one of its ends lowers the other, which is
 * how two turns too close together slow each other down. What is left is the fastest set of speeds
 * the limits allow.
 */
class VelocityPlanner {
public:
    /**
     * @brief Deleted constructor for static class.
     */
    VelocityPlanner() = delete;

    /**
     * @brief Fill in the speeds of a route.
     *
     * @param route The segments of the route, in order, with their kinds, lengths and turns set.
     * @param dynamics The physical limits of the robot.
     * @param profile The profile of the run.
     * @param start_speed The speed at the start of the first segment.
     * @param end_speed The speed at the end of the last segment.
     * @return The time the route takes in seconds.
     */
    static float plan(
        std::span<Segment> route, const Dynamics& dynamics, const RunProfile& profile, float start_speed,
        float end_speed
    );

    /**
     * @brief Get the time a segment takes, with the speeds it already has.
     *
     * @param segment The segment.
     * @param dynamics The physical limits of the robot.
     * @param profile The profile of the run.
     * @return The duration of the segment in seconds.
     */
    static float get_duration(const Segment& segment, const Dynamics& dynamics, const RunProfile& profile);
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_VELOCITY_PLANNER_HPP
