/**
 * @file
 */

#ifndef MICRAS_NAV_EXECUTOR_HPP
#define MICRAS_NAV_EXECUTOR_HPP

#include <cstddef>
#include <span>
#include <vector>

#include "micras/nav/curve_speed.hpp"
#include "micras/nav/line.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Playback of a route as a reference in time.
 *
 * @details The executor owns the clock of the segment in progress and turns it into a pose, a twist
 * and an acceleration in the maze frame, by composing the nominal start of the segment with the
 * motion inside it. The reference depends on time alone, never on where the robot is, so a
 * correction of the pose estimate cannot make it jump, and the time a route takes is exactly the
 * time the planner computed for it.
 */
class Executor {
public:
    /**
     * @brief Configuration struct for the executor.
     *
     * @note The tolerances tell when the robot has settled against a wall: the pose error and the
     * speeds have to stay below them for the settle time.
     */
    struct Config {
        std::size_t capacity;
        float       settle_distance;
        float       settle_angle;
        float       settle_linear_speed;
        float       settle_angular_speed;
        float       settle_time;
    };

    /**
     * @brief Construct a new Executor object.
     *
     * @param dynamics The physical limits of the robot, borrowed for the lifetime of the executor.
     * @param line The racing line a segment of that kind plays back, borrowed for the same time.
     * @param config The configuration for the executor.
     */
    Executor(const Dynamics& dynamics, const Line& line, const Config& config);

    /**
     * @brief Drop every segment and stand still at a pose.
     *
     * @param pose The pose to hold.
     * @param profile The profile of the run about to start.
     */
    void reset(const Pose& pose, const RunProfile& profile);

    /**
     * @brief Queue segments after the ones already queued.
     *
     * @note Nothing is allocated as long as the number of segments waiting stays within the
     * capacity, which a route planned with the robot stopped may exceed and the few segments
     * queued per cell while exploring never do.
     *
     * @param segments The segments, with their speeds already planned.
     */
    void push(std::span<const Segment> segments);

    /**
     * @brief Advance the clock and get the reference for this instant.
     *
     * @note A time scale below one plays the reference in slow motion: the clock advances by less
     * than the time that passed and the speeds are scaled to match, so the path stays the same.
     *
     * @param elapsed_time Time since the last update, in seconds.
     * @param time_scale Factor applied to the elapsed time, in (0, 1].
     * @param estimate The estimated state of the robot, which only tells when it has settled.
     * @return The reference in the maze frame.
     */
    Reference update(float elapsed_time, float time_scale, const State& estimate);

    /**
     * @brief Check if there is nothing left to execute.
     *
     * @return True if every segment has ended.
     */
    bool is_finished() const;

    /**
     * @brief Check if the last segment queued ends within a time from now.
     *
     * @note Whoever feeds the executor cell by cell asks this every iteration, so as to decide the
     * next move as late as possible and still hand it over before the reference runs out.
     *
     * @param horizon The time ahead to look, in seconds.
     * @return True if everything that is queued ends within the horizon.
     */
    bool is_ending(float horizon) const;

    /**
     * @brief Replace everything after a point of the segment in progress by other segments.
     *
     * @note Used to stop short of a wall that was seen late. The segment in progress must be a
     * straight, which is cut where the robot is, and the new segments start from there.
     *
     * @param segments The segments to execute instead, the first starting at the current speed.
     */
    void divert(std::span<const Segment> segments);

    /**
     * @brief Get the segment in progress.
     *
     * @return The segment, or a null pointer if every segment has ended.
     */
    const Segment* get_current() const;

    /**
     * @brief Get the last reference produced.
     *
     * @return The reference.
     */
    const Reference& get_reference() const;

private:
    /**
     * @brief Start the next segment, carrying over the time the previous one did not use.
     */
    void start_next();

    /**
     * @brief Evaluate the segment in progress at the current clock.
     *
     * @return The reference in the maze frame.
     */
    Reference evaluate() const;

    /**
     * @brief Check if the robot has settled at the reference.
     *
     * @param estimate The estimated state of the robot.
     * @param elapsed_time Time since the last update, in seconds.
     * @return True if the errors and the speeds have been small for long enough.
     */
    bool has_settled(const State& estimate, float elapsed_time);

    /**
     * @brief Physical limits of the robot.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const Dynamics& dynamics;

    /**
     * @brief Racing line, for the segment that drives it.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const Line& line;

    /**
     * @brief Tolerances to consider the robot settled.
     */
    Config config;

    /**
     * @brief Profile of the run in progress.
     */
    RunProfile run_profile{};

    /**
     * @brief Segments queued, the ones before the index being already done.
     */
    std::vector<Segment> segments;

    /**
     * @brief Time each of the segments queued takes.
     */
    std::vector<float> durations;

    /**
     * @brief Index of the segment in progress.
     */
    std::size_t index{};

    /**
     * @brief Time the segments queued after the one in progress take.
     */
    float queued_time{};

    /**
     * @brief Time since the start of the segment in progress.
     */
    float clock{};

    /**
     * @brief Duration of the segment in progress.
     */
    float duration{};

    /**
     * @brief Motion along the segment in progress, for the straights and the rotations in place.
     */
    SpeedProfile speed_profile;

    /**
     * @brief Motion along the segment in progress, for the turns.
     */
    CurveSpeed curve_speed;

    /**
     * @brief Time the robot has been within the settling tolerances.
     */
    float settled_time{};

    /**
     * @brief Last reference produced, which is held when there is nothing to execute.
     */
    Reference reference{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_EXECUTOR_HPP
