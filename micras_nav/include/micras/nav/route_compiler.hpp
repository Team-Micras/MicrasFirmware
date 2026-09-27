/**
 * @file
 */

#ifndef MICRAS_NAV_ROUTE_COMPILER_HPP
#define MICRAS_NAV_ROUTE_COMPILER_HPP

#include <vector>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/planner.hpp"
#include "micras/nav/segment.hpp"

namespace micras::nav {
/**
 * @brief Translation of a route on the lattice into the segments that drive it.
 *
 * @details The moves of a route already are the commands, so this is a direct mapping. The straights
 * that a turn carries before and after its curve are merged with the runs around it, which leaves
 * one straight between every two curves, and each segment gets the pose it nominally starts from,
 * computed from the nodes of the lattice so that no error accumulates along the route.
 */
class RouteCompiler {
public:
    /**
     * @brief Shortest straight worth a segment of its own, in meters.
     */
    static constexpr float min_straight{1.0e-4F};

    /**
     * @brief Deleted constructor for static class.
     */
    RouteCompiler() = delete;

    /**
     * @brief Compile a route.
     *
     * @note The speeds of the segments are left for the velocity planner to fill in.
     *
     * @param route The route.
     * @param dynamics The physical limits of the robot, which hold the shape of the turns.
     * @param profile The profile of the run.
     * @param start_distance The distance from where the robot starts to the first node of the route.
     * @param segments The segments of the route, whose storage is reused.
     */
    static void compile(
        const Route& route, const Dynamics& dynamics, const RunProfile& profile, float start_distance,
        std::vector<Segment>& segments
    );
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ROUTE_COMPILER_HPP
