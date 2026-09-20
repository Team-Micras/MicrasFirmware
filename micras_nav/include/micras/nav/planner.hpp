/**
 * @file
 */

#ifndef MICRAS_NAV_PLANNER_HPP
#define MICRAS_NAV_PLANNER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "micras/nav/lattice.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/motion_limits.hpp"

namespace micras::nav {
/**
 * @brief What the planner assumes about the walls that were never observed.
 */
enum class WallAssumption : uint8_t {
    OPTIMISTIC = 0,   // An unknown wall is absent: the fastest route that could still exist.
    PESSIMISTIC = 1,  // An unknown wall is present: the fastest route that is known to exist.
};

/**
 * @brief One move of a route: a number of steps straight ahead, followed by a turn.
 *
 * @note The last move of a route may have no turn, when the goal is entered on a straight.
 */
struct RouteStep {
    uint8_t  run;
    bool     has_turn;
    TurnId   turn;
    TurnSide side;
};

/**
 * @brief A route from the start to the goal, as the moves that drive it.
 *
 * @note The stop distance is measured from the node where the last move ends to the place inside the
 * goal where the robot comes to rest, and the finish distance from the line where the robot enters
 * the goal, which is where the clock of a run stops, to that same place. The time is the one the
 * search found for reaching that line.
 */
struct Route {
    LatticePose            start{};
    std::vector<RouteStep> steps;
    float                  stop_distance{};
    float                  finish_distance{};
    float                  time{std::numeric_limits<float>::infinity()};
};

/**
 * @brief Search for the fastest route from the start to the goal.
 *
 * @details The search is a Dijkstra over the lattice of lattice.hpp. A state is a node together with
 * the turn that was just made to reach it, because that turn fixes the speed the next straight
 * starts at and how much of it the turn already covered. An edge is a motion the robot actually
 * executes, a run of steps ahead followed by a turn, and it exists only if every wall it crosses
 * passes the wall assumption. Its cost is the time of the run, from the evaluator the executor plays
 * back, plus the time of the turn at the speed the run profile gives it, so the fastest path of the
 * graph is the fastest route the robot can drive. Diagonals, the turns of 135 degrees and the two
 * sizes of the turn of 90 degrees are part of the search rather than a rewrite of its result.
 *
 * The clock of a run stops when the robot enters the goal, so a route is priced up to that line,
 * while still being required to come to rest inside the goal.
 *
 * @note The arrays of the search are members, which makes an object of this class large: it belongs
 * in static storage, not on the stack. The search can be advanced a bounded number of nodes at a
 * time, which is how it runs while the robot is moving.
 *
 * @tparam width The width of the maze in cells.
 * @tparam height The height of the maze in cells.
 */
template <uint8_t width, uint8_t height>
class TPlanner {
public:
    /**
     * @brief Number of finished routes kept, for the caller to time exactly and pick from.
     *
     * @note The cost of an edge cannot see two turns so close together that one of them has to be
     * driven slower, which the velocity planner can. Timing the few best candidates with it closes
     * that gap.
     */
    static constexpr uint8_t number_of_candidates{4};

    /**
     * @brief Configuration struct for the planner.
     *
     * @note The start distance is how far the robot is from the first wall it crosses when a run
     * begins.
     */
    struct Config {
        float start_distance;
    };

    /**
     * @brief Construct a new TPlanner object.
     *
     * @param dynamics The physical limits of the robot, borrowed for the lifetime of the planner.
     * @param config The configuration for the planner.
     */
    TPlanner(const Dynamics& dynamics, const Config& config);

    /**
     * @brief Start a search.
     *
     * @param maze The map of the walls, borrowed until the search ends.
     * @param assumption What to assume about the walls that were never observed.
     * @param profile The profile of the run the route is for.
     */
    void begin(const TMaze<width, height>& maze, WallAssumption assumption, const RunProfile& profile);

    /**
     * @brief Advance the search.
     *
     * @param max_nodes The largest number of nodes to expand in this call.
     * @return True if the search has finished.
     */
    bool step(uint32_t max_nodes);

    /**
     * @brief Check if the search has finished.
     *
     * @return True if there is nothing left to expand.
     */
    bool is_finished() const;

    /**
     * @brief Get the number of routes found.
     *
     * @return The number of candidates, which is zero if the goal cannot be reached.
     */
    uint8_t get_number_of_routes() const;

    /**
     * @brief Get one of the routes found.
     *
     * @param index The index of the candidate, the first being the fastest for the search.
     * @param route The route to fill in, whose storage is reused.
     */
    void get_route(uint8_t index, Route& route) const;

    /**
     * @brief Call a function for every wall a route crosses.
     *
     * @tparam F Type of the function, taking the cell and the side of the wall as a GridPose.
     * @param route The route.
     * @param function The function to call.
     */
    template <typename F>
    static void for_each_wall(const Route& route, F&& function);

private:
    /**
     * @brief How a state was reached, which fixes the speed and the offset of the next straight.
     */
    enum class Arrival : uint8_t {
        REST = 0,
        SS90S = 1,
        SS90L = 2,
        SS180 = 3,
        DS45 = 4,
        DS135 = 5,
        SD45 = 6,
        SD135 = 7,
        DD90 = 8,
        NUMBER_OF_ARRIVALS = 9,
    };

    /**
     * @brief Node of the search.
     */
    struct State {
        LatticePose node;
        Arrival     arrival;
    };

    /**
     * @brief End of a route inside the goal.
     */
    struct Terminal {
        float    cost;
        uint16_t state;
        uint8_t  run;
        bool     has_turn;
        TurnId   turn;
        TurnSide side;
        float    stop_distance;
        float    finish_distance;
    };

    /**
     * @brief Cost of an edge, and how fast its turn is driven.
     *
     * @note The speed is a fraction of the speed the run profile gives the turn, in steps of 1/255,
     * and is lower than that only when the run before the turn is too short to reach it.
     */
    struct Edge {
        float   cost;
        uint8_t speed_ratio;
    };

    /**
     * @brief Speed ratio of a turn driven at the speed the run profile gives it.
     */
    static constexpr uint8_t full_speed{255};

    /**
     * @brief Number of states of one wall: two headings along the grid reached in six ways, and
     * four diagonal headings reached in three.
     */
    static constexpr uint16_t states_per_wall{24};

    /**
     * @brief Number of walls of the maze, including the ones of the border.
     */
    static constexpr uint16_t number_of_walls{(width + 1) * height + width * (height + 1)};

    /**
     * @brief Number of states of the search.
     */
    static constexpr uint16_t number_of_states{number_of_walls * states_per_wall};

    /**
     * @brief Largest number of steps of a run.
     */
    static constexpr uint8_t max_run{2 * (width > height ? width : height)};

    /**
     * @brief Number of states whose cost is reset for the price of expanding one node.
     *
     * @note Resetting every state takes longer than an iteration of the control loop may, so it is
     * spread over the calls to step like the search itself.
     */
    static constexpr uint16_t states_per_node_budget{512};

    /**
     * @brief Marker of a state that is not in the queue.
     */
    static constexpr uint16_t not_queued{0xFFFF};

    /**
     * @brief Get the arrival that a turn produces.
     *
     * @param turn The turn.
     * @return The arrival of the state at the exit of the turn.
     */
    static constexpr Arrival to_arrival(TurnId turn);

    /**
     * @brief Get the turn that produced an arrival.
     *
     * @param arrival The arrival, which must not be REST.
     * @return The turn.
     */
    static constexpr TurnId to_turn(Arrival arrival);

    /**
     * @brief Check if a node is inside the maze.
     *
     * @param node The node.
     * @return True if the node is the midpoint of a wall of the maze.
     */
    static constexpr bool is_inside(const LatticePose& node);

    /**
     * @brief Get the index of a state.
     *
     * @param state The state, whose node must be valid and inside the maze.
     * @return The index of the state in the arrays of the search.
     */
    static constexpr uint16_t encode(const State& state);

    /**
     * @brief Get the state behind an index.
     *
     * @param index The index of the state in the arrays of the search.
     * @return The state.
     */
    static constexpr State decode(uint16_t index);

    /**
     * @brief Check if the robot may cross the wall a node is on.
     *
     * @param node The node.
     * @return True if the wall passes the wall assumption.
     */
    bool is_traversable(const LatticePose& node) const;

    /**
     * @brief Check if a cell of the lattice belongs to the goal.
     *
     * @param cell The coordinates of the cell, which may be outside of the maze.
     * @return True if the cell is part of the goal.
     */
    bool is_goal(const LatticePoint& cell) const;

    /**
     * @brief Get how far past a node the robot can stop, when it enters the goal there.
     *
     * @param node The node where the goal is entered, heading along the grid.
     * @return The distance from the node to the center of the last cell of the goal ahead.
     */
    float get_stop_distance(const LatticePose& node) const;

    /**
     * @brief Get the speed a state leaves its node with.
     *
     * @param index The index of the state.
     * @param arrival The arrival of the state.
     * @return The speed in m/s.
     */
    float get_speed(uint16_t index, Arrival arrival) const;

    /**
     * @brief Get the straight left between the curve that reached a state and its node.
     *
     * @param arrival The arrival of the state.
     * @return The distance in meters.
     */
    float get_offset(Arrival arrival) const;

    /**
     * @brief Get the time of a run followed by a turn.
     *
     * @details When the run is too short to change from the speed of one turn to the speed of the
     * other, one of the turns has to be driven slower, which is what the velocity planner will do
     * to it. If it is the turn ahead, it is priced at the speed the run allows and the state it leads
     * to remembers that speed. If it is the turn behind, which was already priced, the time it
     * loses is added to this edge.
     *
     * @param index The index of the state the run starts from.
     * @param arrival The arrival of that state.
     * @param run The number of steps of the run.
     * @param turn The turn after the run.
     * @return The time from the end of the previous curve to the end of this one, and the speed of
     * this one.
     */
    Edge get_edge(uint16_t index, Arrival arrival, uint8_t run, TurnId turn);

    /**
     * @brief Expand a state, relaxing every edge that leaves it.
     *
     * @param index The index of the state.
     */
    void expand(uint16_t index);

    /**
     * @brief Relax the edges of one turn, to both sides, from the end of a run.
     *
     * @param index The index of the state the run starts from.
     * @param state The state the run starts from.
     * @param entry The node where the run ends and the turn starts.
     * @param run The number of steps of the run.
     * @param turn The turn.
     */
    void relax_turn(uint16_t index, const State& state, const LatticePose& entry, uint8_t run, TurnId turn);

    /**
     * @brief Record a route that ends inside the goal, if it is among the best.
     *
     * @param terminal The end of the route.
     */
    void add_terminal(const Terminal& terminal);

    /**
     * @brief Lower the cost of a state, moving it up in the queue.
     *
     * @param index The index of the state.
     * @param cost The new cost, lower than the current one.
     * @param link How the state was reached, for rebuilding the route.
     * @param speed_ratio How fast the turn that reached the state is driven.
     */
    void relax(uint16_t index, float cost, uint16_t link, uint8_t speed_ratio);

    /**
     * @brief Remove the state with the lowest cost from the queue.
     *
     * @return The index of the state.
     */
    uint16_t pop();

    /**
     * @brief Move an entry of the queue up to its place.
     *
     * @param position The position of the entry in the queue.
     */
    void sift_up(uint16_t position);

    /**
     * @brief Move an entry of the queue down to its place.
     *
     * @param position The position of the entry in the queue.
     */
    void sift_down(uint16_t position);

    /**
     * @brief Physical limits of the robot.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const Dynamics& dynamics;

    /**
     * @brief Distance from the start pose to the first node.
     */
    float start_distance;

    /**
     * @brief Map of the walls being searched.
     */
    const TMaze<width, height>* maze{nullptr};

    /**
     * @brief Assumption about the unknown walls.
     */
    WallAssumption assumption{WallAssumption::PESSIMISTIC};

    /**
     * @brief Profile of the run being planned.
     */
    RunProfile run_profile{};

    /**
     * @brief Limits of the motion along a straight, for the run being planned.
     */
    MotionLimits limits{};

    /**
     * @brief Speed of each turn, for the run being planned.
     */
    std::array<float, number_of_turns> turn_speeds{};

    /**
     * @brief Edge of every combination of arrival, run and turn, for an arrival at full speed,
     * filled in when first needed.
     */
    std::array<
        std::array<std::array<Edge, number_of_turns>, max_run + 1>, std::to_underlying(Arrival::NUMBER_OF_ARRIVALS)>
        edges{};

    /**
     * @brief Lowest cost found for each state.
     */
    std::array<float, number_of_states> costs{};

    /**
     * @brief How each state was reached: the run, the previous arrival and the side of the turn.
     */
    std::array<uint16_t, number_of_states> links{};

    /**
     * @brief How fast the turn that reached each state is driven, as a fraction of its speed.
     */
    std::array<uint8_t, number_of_states> speed_ratios{};

    /**
     * @brief Binary heap of the states waiting to be expanded, ordered by cost.
     */
    std::array<uint16_t, number_of_states> queue{};

    /**
     * @brief Position of each state in the queue.
     */
    std::array<uint16_t, number_of_states> positions{};

    /**
     * @brief Number of states in the queue.
     */
    uint16_t queue_size{};

    /**
     * @brief Number of states whose cost was reset since the search began.
     */
    uint16_t cleared{number_of_states};

    /**
     * @brief Best ends of route found, ordered by cost.
     */
    std::array<Terminal, number_of_candidates> terminals{};

    /**
     * @brief Number of ends of route found.
     */
    uint8_t number_of_terminals{};

    /**
     * @brief Start node of the search.
     */
    LatticePose start{};
};
}  // namespace micras::nav

#include "micras/nav/impl/planner.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_PLANNER_HPP
