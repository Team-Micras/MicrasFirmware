/**
 * @file
 */

#ifndef MICRAS_NAV_MISSION_HPP
#define MICRAS_NAV_MISSION_HPP

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "micras/core/types.hpp"
#include "micras/nav/executor.hpp"
#include "micras/nav/explorer.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/planner.hpp"
#include "micras/nav/racing_line.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/wall_model.hpp"
#include "micras/nav/wall_observer.hpp"

namespace micras::nav {
/**
 * @brief Sequencing of a run: what to map, where to go next and which motion gets there.
 *
 * @details The mission owns the outer loop of the navigation: the wall observer fills the map from
 * the readings, the map is flooded towards whatever is worth reaching, and one cell at a time the
 * next move is handed to the executor, which turns it into the reference the controller follows.
 *
 * - Exploring, the robot descends the flood towards the goal and stops at it.
 * - Returning, the explorer names the cells still worth visiting and the robot descends the flood
 *   towards them, until the map proves that the fastest route is known. Then it goes back to the
 *   start cell, squares up against its back wall and parks where a run starts from.
 * - Solving, the route planned beforehand is executed as it is, or the racing line through its cells.
 *
 * The move through a cell is decided when the robot crosses into it, as late as it can be, from the
 * walls that were decided by then. A turn is only made through a wall that was seen absent. A
 * straight may start with the wall ahead still unknown, since that wall only comes into range
 * inside the cell, but then it is watched: if it turns out to be there, or is still unknown at the
 * last point from which the robot can stop at the center of the cell, the robot stops there and
 * decides again. A wall seen late costs a stop instead of a crash.
 *
 * The mission keeps its own account of which cell the robot is in, from the moves it made. Whenever
 * the robot stands still at the center of a cell, that account is checked against the pose estimate,
 * and a run in which the two disagree fails instead of going on to write walls in the wrong cells.
 *
 * @note Every decision takes a bounded time, since it happens while the robot is moving. The search
 * of the planner, which does not, is spread over the iterations.
 *
 * @tparam width The width of the maze in cells.
 * @tparam height The height of the maze in cells.
 */
template <uint8_t width, uint8_t height>
class TMission {
public:
    /**
     * @brief Progress of a run.
     */
    enum class Status : uint8_t {
        RUNNING = 0,
        FINISHED = 1,
        FAILED = 2,
    };

    /**
     * @brief Configuration struct for the mission.
     *
     * @note The start offset is the distance from the back edge of the start cell to the axle of
     * the robot when a run starts. The map profiles are the run profiles the map has to be complete
     * for, borrowed for the lifetime of the mission. The times are how long to stand still before
     * turning in place, at most while squaring up against a wall and between two looks at a wall
     * that is still unknown, of which only so many are taken before giving up. The commit margin is
     * kept between where the robot could still stop at the center of a cell and where it decides to.
     *
     * No field has a default value, here or in any other configuration of the navigation, so that
     * leaving one out of a configuration is a compiler warning instead of a silent zero.
     */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init) see the note above
    struct Config {
        TMaze<width, height>::Config         maze;
        TPlanner<width, height>::Config      planner;
        TWallObserver<width, height>::Config observer;
        TRacingLine<width, height>::Config   racing_line;
        Executor::Config                     executor;
        RunProfile                           search_profile;
        std::span<const RunProfile>          map_profiles;
        float                                start_offset;
        float                                stop_time;
        float                                attach_time;
        float                                look_time;
        uint16_t                             max_looks;
        float                                commit_margin;
        uint32_t                             edges_per_iteration;
    };

    /**
     * @brief Construct a new TMission object.
     *
     * @param dynamics The physical limits of the robot, borrowed for the lifetime of the mission.
     * @param wall_model The geometry of the wall sensors, borrowed for the same time.
     * @param config The configuration for the mission.
     */
    TMission(const Dynamics& dynamics, const WallModel& wall_model, const Config& config);

    /**
     * @brief Get the map of the walls.
     *
     * @return The maze.
     */
    TMaze<width, height>& get_maze();

    /**
     * @brief Get the map of the walls.
     *
     * @return The maze.
     */
    const TMaze<width, height>& get_maze() const;

    /**
     * @brief Get the pose a run starts from.
     *
     * @return The pose in the maze frame.
     */
    Pose get_start_pose() const;

    /**
     * @brief Start planning the route of a fast run.
     *
     * @param profile The profile of the run.
     */
    void begin_plan(const RunProfile& profile);

    /**
     * @brief Advance the planning of the route of a fast run.
     *
     * @note Only to be called with the robot stopped: when the search ends, the best candidates
     * are compiled and timed, which is not bounded to fit in an iteration. When the profile asks
     * for the racing line, it is optimized next, through the cells of the route chosen, and it
     * replaces the route if it is found and faster. With the risky switch too, a second line goes
     * through the route planned without the risky turns, and the risky line is only driven if it
     * is the faster of the two; it is then optimized again, since only one line is kept.
     *
     * @param max_edges The largest number of edges the planner may try in this call.
     * @return True if the planning has finished.
     */
    bool update_plan(uint32_t max_edges);

    /**
     * @brief Check if a route for a fast run exists.
     *
     * @return True if the last planning found a route.
     */
    bool has_route() const;

    /**
     * @brief Get the time the planned route is expected to take.
     *
     * @return The time from the start until the robot comes to rest inside the goal, in seconds.
     */
    float get_route_time() const;

    /**
     * @brief Start a run.
     *
     * @note Exploring and solving start from the start pose. Returning starts from wherever the
     * exploration ended, with the robot stopped at the center of a cell.
     *
     * @param objective What the run is for.
     */
    void start(core::Objective objective);

    /**
     * @brief Advance the run by one iteration.
     *
     * @param measurements The current measurements.
     * @param localizer The pose estimate, which is corrected while the robot is commanded to rest.
     * @param elapsed_time Time since the last iteration, in seconds.
     * @param time_scale How much slower the reference has to be played, from the controller.
     * @return The progress of the run.
     */
    Status update(const Measurements& measurements, Localizer& localizer, float elapsed_time, float time_scale);

    /**
     * @brief Get what the robot should be doing at this instant.
     *
     * @return The reference in the maze frame.
     */
    const Reference& get_reference() const;

    /**
     * @brief Get the playback of the moves, to see which segment is in progress.
     *
     * @return The executor.
     */
    const Executor& get_executor() const;

    /**
     * @brief Get the cell the mission believes the robot is in or entering.
     *
     * @note Comparing it with the cell of the pose estimate tells whether the two still agree.
     *
     * @return The cell and the direction of travel.
     */
    const GridPose& get_cell() const;

private:
    /**
     * @brief What the planning of a fast run is doing.
     */
    enum class PlanStage : uint8_t {
        IDLE = 0,
        SEARCH = 1,
        LINE = 2,
    };

    /**
     * @brief Which racing line is being optimized: through the route of the profile asked for,
     * through the route planned without the risky turns, or the first one again.
     */
    enum class LinePass : uint8_t {
        REQUESTED = 0,
        CAREFUL = 1,
        AGAIN = 2,
    };

    /**
     * @brief Largest number of segments a move through one cell takes.
     */
    static constexpr uint8_t max_move_segments{6};

    /**
     * @brief Shortest straight worth a segment of its own, in meters.
     */
    static constexpr float min_straight{1.0e-4F};

    /**
     * @brief Distance within which a segment is taken to start where the watched cell is entered.
     */
    static constexpr float watch_tolerance{0.001F};

    /**
     * @brief Small list of segments, built on the stack for one move.
     */
    struct Move {
        /**
         * @brief Append a segment.
         *
         * @param segment The segment.
         */
        void add(const Segment& segment) {
            if (this->size < max_move_segments) {
                this->segments.at(this->size++) = segment;
            }
        }

        std::array<Segment, max_move_segments> segments{};
        uint8_t                                size{};
    };

    /**
     * @brief Time the candidate routes the planner found and keep the fastest.
     *
     * @param profile The profile the routes are compiled and timed for.
     * @param route Filled with the fastest route.
     * @param segments Filled with its segments, and left empty if there is no route.
     * @return The time of the fastest route up to the goal line, or infinity.
     */
    float choose_route(const RunProfile& profile, Route& route, std::vector<Segment>& segments);

    /**
     * @brief Get the nominal pose of the robot as it crosses into a cell.
     *
     * @param cell The cell entered and the direction of travel.
     * @return The pose at the middle of the edge of the cell, in the maze frame.
     */
    Pose get_entry_pose(const GridPose& cell) const;

    /**
     * @brief Get the nominal pose of the robot at the center of a cell.
     *
     * @param cell The cell and the direction the robot faces.
     * @return The pose in the maze frame.
     */
    Pose get_center_pose(const GridPose& cell) const;

    /**
     * @brief Flood the map towards whatever the current objective wants to reach.
     */
    void flood();

    /**
     * @brief Plan the speeds of a move and hand it to the executor.
     *
     * @param move The segments of the move.
     * @param start_speed The speed at the start of the move.
     * @param end_speed The speed at the end of the move.
     */
    void execute(Move& move, float start_speed, float end_speed);

    /**
     * @brief Decide the next move, with the robot crossing into a cell at the search speed.
     */
    void decide_at_entry();

    /**
     * @brief Decide the next move, with the robot stopped at the center of a cell.
     */
    void decide_at_center();

    /**
     * @brief Add to a move the segments that stop the robot at the center of the cell it is entering.
     *
     * @note When the wall ahead is known to be there, the robot then squares up against it.
     * Otherwise it just stands still for a moment.
     *
     * @param move The move.
     */
    void add_stop_at_center(Move& move) const;

    /**
     * @brief Check if the run is over now that the robot is entering a cell, and end it if so.
     *
     * @return True if the last move of the run was handed to the executor.
     */
    bool finish_at_entry();

    /**
     * @brief Stop at the center of the cell being crossed instead of going through it.
     */
    void divert_to_center();

    /**
     * @brief Make a segment.
     *
     * @param kind The kind of the segment.
     * @param length The length of the segment, in the unit of its kind.
     * @param start The pose the segment nominally starts from.
     * @return The segment, with no speeds.
     */
    static Segment make_segment(SegmentKind kind, float length, const Pose& start);

    /**
     * @brief Physical limits of the robot.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const Dynamics& dynamics;

    /**
     * @brief Geometry of the wall sensors.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const WallModel& wall_model;

    /**
     * @brief Parameters of the mission.
     */
    Config config;

    /**
     * @brief Map of the walls.
     */
    TMaze<width, height> maze;

    /**
     * @brief Decision of which walls exist.
     */
    TWallObserver<width, height> observer;

    /**
     * @brief Search for the fastest route.
     */
    TPlanner<width, height> planner;

    /**
     * @brief Choice of what is still worth visiting.
     */
    TExplorer<width, height> explorer;

    /**
     * @brief Smoothest line through the cells of the route of a fast run.
     */
    TRacingLine<width, height> racing_line;

    /**
     * @brief Playback of the moves as a reference.
     */
    Executor executor;

    /**
     * @brief Speed the robot searches at, which every move through a cell starts and ends with.
     */
    float search_speed;

    /**
     * @brief What the run in progress is for.
     */
    core::Objective objective{core::Objective::EXPLORE};

    /**
     * @brief Progress of the run in progress.
     */
    Status status{Status::FINISHED};

    /**
     * @brief Cell the robot is in or entering, and the direction of travel.
     */
    GridPose cell{};

    /**
     * @brief Whether the robot is stopped at the center of the cell, rather than crossing into it.
     */
    bool at_center{};

    /**
     * @brief Whether the moves handed to the executor are the last ones of the run.
     */
    bool finishing{};

    /**
     * @brief Whether the robot is crossing a cell whose wall ahead was unknown when it entered.
     */
    bool watching_front{};

    /**
     * @brief Cell being crossed while the wall ahead of it is watched.
     */
    GridPose watched_cell{};

    /**
     * @brief Number of times in a row the robot stood still waiting for a wall to be decided.
     */
    uint16_t looks{};

    /**
     * @brief Profile of the fast run that was planned.
     */
    RunProfile solve_profile{};

    /**
     * @brief Segments of the fast run that was planned.
     */
    std::vector<Segment> solve_segments;

    /**
     * @brief Segments of a candidate route being timed.
     */
    std::vector<Segment> candidate_segments;

    /**
     * @brief Candidate route being compiled, kept to reuse its storage.
     */
    Route candidate_route;

    /**
     * @brief Route of the fast run that was planned, which the racing line goes through.
     */
    Route solve_route;

    /**
     * @brief Route planned without the risky turns, and its segments, for the second racing line.
     */
    ///@{
    Route                careful_route;
    std::vector<Segment> careful_segments;
    ///@}

    /**
     * @brief Time of the racing line through the risky route, to compare the second one with.
     */
    float risky_line_time{};

    /**
     * @brief Time the planned fast run is expected to take.
     */
    float route_time{};

    /**
     * @brief What the planning of a fast run is doing, and which racing line it is at.
     */
    ///@{
    PlanStage stage{PlanStage::IDLE};
    LinePass  pass{LinePass::REQUESTED};
    ///@}

    /**
     * @brief Last reference produced.
     */
    Reference reference{};
};
}  // namespace micras::nav

#include "micras/nav/impl/mission.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_MISSION_HPP
