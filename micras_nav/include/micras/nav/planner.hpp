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
 * @details The search is a Dijkstra over the lattice of lattice.hpp. An edge is a motion the robot
 * actually executes, a run of steps ahead followed by a turn, and it exists only if every wall it
 * crosses passes the wall assumption. Its cost is the time of the run, from the evaluator the
 * executor plays back, plus the time of the turn at the speed the run profile gives it, so the
 * fastest path of the graph is the fastest route the robot can drive. Diagonals, the turns of 135
 * degrees and the two sizes of the turn of 90 degrees are part of the search rather than a rewrite
 * of its result.
 *
 * What the search holds are labels: a way of reaching a node, with its cost, the turn that reached
 * it and how fast that turn is driven, since those fix the speed the next straight starts at and
 * how much of it the turn already covered. A label is dropped only when another label of the same
 * node is at least as fast for every continuation of the route, which is what keeps the search
 * exact with far fewer labels than there are ways of reaching a node.
 *
 * The clock of a run stops when the robot enters the goal, so a route is priced up to that line,
 * while still being required to come to rest inside the goal.
 *
 * @note The labels are members, which makes an object of this class large: it belongs in static
 * storage, not on the stack. The search is advanced a bounded number of edges at a time, which is
 * how it runs while the robot is moving: an edge costs about the same whatever the turns, while a
 * node costs as many edges as there are turns to try from it.
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
     * @brief Largest number of labels held at once.
     *
     * @note On ten contest mazes, whole and with a random part of their walls unknown, the most a
     * search held was 4519, with every turn and profile, and 4 of 5490 searches held more than 4500.
     * A search that needs more drops the costliest labels waiting to be expanded, and reports that it
     * is no longer exact.
     */
    static constexpr uint16_t max_labels{6144};

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
     * @note Before the search itself, every pair of arrivals is compared for the dominance, one run
     * of one pair for each edge of the budget.
     *
     * @param max_edges The largest number of edges to try in this call.
     * @return True if the search has finished.
     */
    bool step(uint32_t max_edges);

    /**
     * @brief Check if the search has finished.
     *
     * @return True if there is nothing left to expand.
     */
    bool is_finished() const;

    /**
     * @brief Check if the search kept every label it needed.
     *
     * @return False if it ran out of labels, in which case the routes found may not be the fastest.
     */
    bool is_exact() const;

    /**
     * @brief Get the largest number of labels the search held at once.
     *
     * @return The number of labels, at most max_labels.
     */
    uint16_t get_peak_labels() const;

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
     * @brief Way of reaching a node.
     *
     * @note The arrival is the turn that reached the node plus one, zero being the start, and the
     * run and the side are those of the edge that ended with that turn. The position is the place
     * of the label in the queue, and is not_queued once it has been expanded.
     */
    struct Label {
        float    cost;
        uint16_t node;
        uint16_t parent;
        uint16_t next;
        uint16_t position;
        uint8_t  arrival;
        uint8_t  speed_ratio;
        uint8_t  run;
        TurnSide side;
    };

    /**
     * @brief End of a route inside the goal.
     */
    struct Terminal {
        float    cost;
        uint16_t label;
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
     * @brief Arrival of the label of the start, which is at rest.
     */
    static constexpr uint8_t rest{0};

    /**
     * @brief Number of ways a node can be reached: at rest, or by one of the turns.
     */
    static constexpr uint8_t number_of_arrivals{number_of_turns + 1};

    /**
     * @brief Speed ratio of a turn driven at the speed the run profile gives it.
     */
    static constexpr uint8_t full_speed{255};

    /**
     * @brief Number of headings a wall can be crossed with: two along the grid and four diagonal.
     */
    static constexpr uint8_t headings_per_wall{6};

    /**
     * @brief Number of walls of the maze, including the ones of the border.
     */
    static constexpr uint16_t number_of_walls{(width + 1) * height + width * (height + 1)};

    /**
     * @brief Number of nodes of the lattice.
     */
    static constexpr uint16_t number_of_nodes{number_of_walls * headings_per_wall};

    /**
     * @brief Largest number of steps of a run, along a diagonal, where a step is half a cell.
     */
    static constexpr uint8_t max_run{2 * (width > height ? width : height)};

    /**
     * @brief Largest number of steps of a run along the grid, where a step is a whole cell.
     */
    static constexpr uint8_t max_straight_run{width > height ? width : height};

    /**
     * @brief Marker of the end of a list of labels, and of a label that is not in the queue.
     */
    static constexpr uint16_t none{0xFFFF};

    /**
     * @brief Pairs of an arrival and a turn that can follow it, which are the edges worth caching.
     *
     * @details A turn follows an arrival when it starts on the kind of heading, diagonal or along
     * the grid, that the arrival ends on. The index of the pair is its row in the cache, or -1.
     */
    struct EdgePairs {
        /**
         * @brief Number the pairs of arrivals and turns.
         */
        constexpr EdgePairs() {
            for (uint8_t arrival = 0; arrival < number_of_arrivals; arrival++) {
                for (uint8_t turn = 0; turn < number_of_turns; turn++) {
                    const bool follows =
                        get_primitive(static_cast<TurnId>(turn)).diagonal_entry == ends_diagonal(arrival);

                    this->index.at(arrival).at(turn) = follows ? this->count : -1;
                    this->count += follows ? 1 : 0;
                }
            }
        }

        std::array<std::array<int16_t, number_of_turns>, number_of_arrivals> index{};
        int16_t                                                              count{};
    };

    /**
     * @brief Index of the cached edge of every pair of an arrival and a turn.
     */
    static constexpr EdgePairs edge_pairs{};

    /**
     * @brief Check if an arrival ends on a diagonal heading.
     *
     * @param arrival The arrival.
     * @return True if the turn of the arrival ends on a diagonal.
     */
    static constexpr bool ends_diagonal(uint8_t arrival);

    /**
     * @brief Get the turn that produced an arrival.
     *
     * @param arrival The arrival, which must not be the start.
     * @return The turn.
     */
    static constexpr TurnId to_turn(uint8_t arrival);

    /**
     * @brief Check if a node is inside the maze.
     *
     * @param node The node.
     * @return True if the node is the midpoint of a wall of the maze.
     */
    static constexpr bool is_inside(const LatticePose& node);

    /**
     * @brief Get the index of a node.
     *
     * @param node The node, which must be valid and inside the maze.
     * @return The index of the node, below number_of_nodes.
     */
    static constexpr uint16_t encode(const LatticePose& node);

    /**
     * @brief Get the node behind an index.
     *
     * @param index The index of the node.
     * @return The node.
     */
    static constexpr LatticePose decode(uint16_t index);

    /**
     * @brief Check if a turn can be driven on the run being planned.
     *
     * @param turn The turn.
     * @return True if the turn fits between its nodes with the margin of the run profile.
     */
    bool is_usable(TurnId turn) const;

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
     * @brief Get the speed a label leaves its node with.
     *
     * @param arrival The arrival of the label.
     * @param speed_ratio How fast the turn of the arrival is driven.
     * @return The speed in m/s.
     */
    float get_speed(uint8_t arrival, uint8_t speed_ratio) const;

    /**
     * @brief Get the straight left between the curve that reached a label and its node.
     *
     * @param arrival The arrival of the label.
     * @return The distance in meters.
     */
    float get_offset(uint8_t arrival) const;

    /**
     * @brief Get the time of a run followed by a turn.
     *
     * @details When the run is too short to change from the speed of one turn to the speed of the
     * other, one of the turns has to be driven slower, which is what the velocity planner will do
     * to it. If it is the turn ahead, it is priced at the speed the run allows and the label it leads
     * to remembers that speed. If it is the turn behind, which was already priced, the time it
     * loses is added to this edge.
     *
     * @param arrival The arrival of the label the run starts from.
     * @param speed_ratio How fast the turn of that arrival is driven.
     * @param run The number of steps of the run.
     * @param turn The turn after the run.
     * @return The time from the end of the previous curve to the end of this one, and the speed of
     * this one.
     */
    Edge get_edge(uint8_t arrival, uint8_t speed_ratio, uint8_t run, TurnId turn);

    /**
     * @brief Check if two arrivals can meet at a node and have to be compared.
     *
     * @param first The first arrival.
     * @param second The second arrival.
     * @return True if they are different, end on the same kind of heading and can both be driven.
     */
    bool is_comparable(uint8_t first, uint8_t second) const;

    /**
     * @brief Compare the next pair of arrivals over the next run, moving on to the next pair after.
     */
    void prepare();

    /**
     * @brief Bound how much slower one arrival at full speed can be than another, over one run.
     *
     * @details Whatever follows a node starts with the same edge from both arrivals. The two routes
     * differ by the time of that first edge, not counting its turn, and by how fast that turn is
     * driven, after which the arrival at the faster turn can only be faster. So when the first
     * arrival never leaves a turn slower than the second, the most its first edge, or its way into
     * the goal, takes longer is the most any route from it can take longer. The bound is the largest
     * over every run, and is infinite if the first arrival leaves any turn slower.
     *
     * @param first The arrival that would be kept.
     * @param second The arrival that would be dropped.
     * @param run The number of steps of the run before the first edge.
     */
    void compare_arrivals(uint8_t first, uint8_t second, uint8_t run);

    /**
     * @brief Check if a label is at least as fast as another of the same node, for every route.
     *
     * @note A label is compared with one of the same arrival by the time before its turn and the
     * speed of its turn, and with one of another arrival only when the first turn is driven at full
     * speed. A second turn driven slower is compared as if it were driven at full speed, less the
     * time it loses, since a turn driven slower never leaves a route faster.
     *
     * @param first The label that would be kept.
     * @param second The label that would be dropped.
     * @return True if the second label can be dropped.
     */
    bool dominates(const Label& first, const Label& second);

    /**
     * @brief Advance the expansion of a label by one edge.
     *
     * @note The run is lengthened when every turn at its end has been tried, and the expansion ends
     * at a wall that may not be crossed, at the goal or after the longest run.
     */
    void advance_expansion();

    /**
     * @brief Relax the edges of one turn, to both sides, from the end of a run.
     *
     * @param index The index of the label the run starts from.
     * @param entry The node where the run ends and the turn starts.
     * @param run The number of steps of the run.
     * @param turn The turn.
     */
    void relax_turn(uint16_t index, const LatticePose& entry, uint8_t run, TurnId turn);

    /**
     * @brief Record a route that ends inside the goal, if it is among the best.
     *
     * @param terminal The end of the route.
     */
    void add_terminal(const Terminal& terminal);

    /**
     * @brief Add a label to its node, unless another one there is at least as fast.
     *
     * @note The labels of the node that the new one is at least as fast as are dropped, unless they
     * have been expanded, since other labels may lead back to them.
     *
     * @param label The label, with every field but the links filled in.
     */
    void insert(Label label);

    /**
     * @brief Take a label from the pool, making room if it is full.
     *
     * @param cost The cost of the label that needs the room.
     * @return The index of the label, or none if every label held is cheaper than the cost.
     */
    uint16_t allocate(float cost);

    /**
     * @brief Remove a label that waits in the queue from its node and give it back to the pool.
     *
     * @param index The index of the label.
     */
    void release(uint16_t index);

    /**
     * @brief Add a label to the queue.
     *
     * @param index The index of the label.
     */
    void push(uint16_t index);

    /**
     * @brief Remove the label with the lowest cost from the queue.
     *
     * @return The index of the label.
     */
    uint16_t pop();

    /**
     * @brief Remove a label from the queue, wherever it is.
     *
     * @param position The position of the label in the queue.
     */
    void remove(uint16_t position);

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
     * @brief Turns that can be driven on the run being planned, from a heading along the grid and
     * from a diagonal one.
     */
    std::array<std::array<TurnId, number_of_turns>, 2> usable_turns{};

    /**
     * @brief Number of usable turns from a heading along the grid and from a diagonal one.
     */
    std::array<uint8_t, 2> number_of_usable_turns{};

    /**
     * @brief Largest number of cells of the goal in a line, which bounds where the robot stops.
     */
    uint8_t goal_length{};

    /**
     * @brief Edge of every pair of an arrival at full speed and a turn, for every run, filled in when
     * first needed.
     */
    std::array<std::array<float, max_run + 1>, edge_pairs.count> edge_costs{};

    /**
     * @brief Speed ratio of the turn of every cached edge.
     */
    std::array<std::array<uint8_t, max_run + 1>, edge_pairs.count> edge_speed_ratios{};

    /**
     * @brief Dominance gap of every pair of arrivals, computed before the search.
     */
    std::array<std::array<float, number_of_arrivals>, number_of_arrivals> gaps{};

    /**
     * @brief First label of each node.
     */
    std::array<uint16_t, number_of_nodes> heads{};

    /**
     * @brief Labels of the search.
     */
    std::array<Label, max_labels> labels{};

    /**
     * @brief Binary heap of the labels waiting to be expanded, ordered by cost.
     */
    std::array<uint16_t, max_labels> queue{};

    /**
     * @brief Number of labels in the queue.
     */
    uint16_t queue_size{};

    /**
     * @brief Number of labels ever taken from the pool in this search.
     */
    uint16_t allocated{};

    /**
     * @brief First label given back to the pool, the others following it.
     */
    uint16_t free_list{none};

    /**
     * @brief Number of labels held.
     */
    uint16_t held{};

    /**
     * @brief Largest number of labels held at once.
     */
    uint16_t peak{};

    /**
     * @brief Whether every label the search needed was kept.
     */
    bool exact{true};

    /**
     * @brief Whether the pairs of arrivals are still being compared.
     */
    bool preparing{};

    /**
     * @brief Pair of arrivals and run the comparison is at.
     */
    ///@{
    uint8_t prepare_first{};
    uint8_t prepare_second{};
    uint8_t prepare_run{};
    ///@}

    /**
     * @brief Label being expanded, or none.
     */
    uint16_t expanding{none};

    /**
     * @brief Node at the end of the run of the expansion, the number of steps of that run and the
     * next turn to try there.
     */
    ///@{
    LatticePose expansion_entry{};
    uint8_t     expansion_run{};
    uint8_t     expansion_turn{};
    ///@}

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
