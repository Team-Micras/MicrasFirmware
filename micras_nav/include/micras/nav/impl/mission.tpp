/**
 * @file
 */

#ifndef MICRAS_NAV_MISSION_TPP
#define MICRAS_NAV_MISSION_TPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <utility>

#include "micras/core/types.hpp"
#include "micras/nav/executor.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/route_compiler.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/velocity_planner.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TMission<width, height>::TMission(const Dynamics& dynamics, const WallModel& wall_model, const Config& config) :
    dynamics{dynamics},
    wall_model{wall_model},
    config{config},
    maze{config.maze},
    observer{config.observer},
    planner{dynamics, config.planner},
    explorer{planner, config.map_profiles},
    executor{dynamics, config.executor},
    search_speed{std::min(
        dynamics.get_linear_limits(config.search_profile).max_speed,
        dynamics.get_turn_speed(config.search_profile, TurnId::SS90S)
    )} {
    this->solve_segments.reserve(config.executor.capacity);
    this->candidate_segments.reserve(config.executor.capacity);
    this->candidate_route.steps.reserve(config.executor.capacity);
    this->observer.reset();
}

template <uint8_t width, uint8_t height>
TMaze<width, height>& TMission<width, height>::get_maze() {
    return this->maze;
}

template <uint8_t width, uint8_t height>
const TMaze<width, height>& TMission<width, height>::get_maze() const {
    return this->maze;
}

template <uint8_t width, uint8_t height>
Pose TMission<width, height>::get_start_pose() const {
    const float cell_size = this->dynamics.get_model().maze.cell_size;

    return this->get_center_pose(this->maze.get_start())
        .compose({.position = {.x = this->config.start_offset - cell_size / 2.0F, .y = 0.0F}, .orientation = 0.0F});
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::begin_plan(const RunProfile& profile) {
    this->solve_profile = profile;
    this->solve_segments.clear();
    this->route_time = std::numeric_limits<float>::infinity();
    this->planner.begin(this->maze, WallAssumption::PESSIMISTIC, profile);
    this->planning = true;
}

template <uint8_t width, uint8_t height>
bool TMission<width, height>::update_plan(uint32_t max_nodes) {
    if (not this->planning) {
        return true;
    }

    if (not this->planner.step(max_nodes)) {
        return false;
    }

    this->planning = false;

    const MotionLimits limits = this->dynamics.get_linear_limits(this->solve_profile);

    for (uint8_t i = 0; i < this->planner.get_number_of_routes(); i++) {
        this->planner.get_route(i, this->candidate_route);

        RouteCompiler::compile(
            this->candidate_route, this->dynamics, this->solve_profile, this->config.planner.start_distance,
            this->candidate_segments
        );

        const float total_time =
            VelocityPlanner::plan(this->candidate_segments, this->dynamics, this->solve_profile, 0.0F, 0.0F);

        const Segment&     last = this->candidate_segments.back();
        const SpeedProfile braking{last.length, last.start_speed, 0.0F, limits};

        float after_line = braking.duration();

        if (this->candidate_route.finish_distance <= last.length) {
            after_line -= braking.time_at(last.length - this->candidate_route.finish_distance);
        } else {
            after_line += (this->candidate_route.finish_distance - last.length) / last.start_speed;
        }

        if (total_time - after_line < this->route_time) {
            this->route_time = total_time - after_line;
            std::swap(this->solve_segments, this->candidate_segments);
        }
    }

    return true;
}

template <uint8_t width, uint8_t height>
bool TMission<width, height>::has_route() const {
    return not this->planning and not this->solve_segments.empty();
}

template <uint8_t width, uint8_t height>
float TMission<width, height>::get_route_time() const {
    return this->route_time;
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::start(core::Objective objective) {
    const float cell_size = this->dynamics.get_model().maze.cell_size;

    this->objective = objective;
    this->status = Status::RUNNING;
    this->finishing = false;
    this->watching_front = false;
    this->looks = 0;

    switch (objective) {
        case core::Objective::EXPLORE: {
            this->executor.reset(this->get_start_pose(), this->config.search_profile);
            this->cell = this->maze.get_start().front();
            this->at_center = false;
            this->flood();

            Move move{};
            move.add(
                make_segment(SegmentKind::STRAIGHT, cell_size - this->config.start_offset, this->get_start_pose())
            );
            this->execute(move, 0.0F, this->search_speed);
            break;
        }

        case core::Objective::RETURN:
            this->executor.reset(this->get_center_pose(this->cell), this->config.search_profile);
            this->at_center = true;
            this->explorer.reset();
            this->flood();
            break;

        case core::Objective::SOLVE:
            this->executor.reset(this->get_start_pose(), this->solve_profile);
            this->executor.push(this->solve_segments);
            this->finishing = true;
            break;
    }

    this->reference = this->executor.get_reference();
}

template <uint8_t width, uint8_t height>
TMission<width, height>::Status TMission<width, height>::update(
    const Measurements& measurements, Localizer& localizer, float elapsed_time, float time_scale
) {
    if (this->status != Status::RUNNING) {
        return this->status;
    }

    if (this->objective != core::Objective::SOLVE) {
        bool changed = this->observer.update(measurements, localizer, this->wall_model, this->maze);

        if (this->objective == core::Objective::RETURN) {
            changed = this->explorer.update(this->maze, this->config.nodes_per_iteration) or changed;
        }

        if (changed) {
            this->flood();
        }
    }

    const Segment* watched = this->executor.get_current();

    if (this->watching_front and watched != nullptr and watched->kind == SegmentKind::STRAIGHT and
        watched->start.position.distance(this->get_entry_pose(this->watched_cell).position) < watch_tolerance) {
        const float cell_size = this->dynamics.get_model().maze.cell_size;
        const float braking = this->search_speed * this->search_speed /
                              (2.0F * this->dynamics.get_linear_limits(this->config.search_profile).deceleration);

        const WallState wall = this->maze.get_wall(this->watched_cell);

        if (wall == WallState::NO_WALL) {
            this->watching_front = false;
        } else if (
            wall == WallState::WALL or
            this->executor.get_reference().distance >= cell_size / 2.0F - braking - this->config.commit_margin
        ) {
            this->divert_to_center();
        }
    }

    if (this->executor.is_ending(elapsed_time)) {
        if (this->finishing) {
            if (this->executor.is_finished()) {
                this->status = Status::FINISHED;
            }
        } else if (this->at_center) {
            this->decide_at_center();
        } else {
            this->decide_at_entry();
        }
    }

    this->reference = this->executor.update(elapsed_time, time_scale, localizer.get_state());

    const Segment* current = this->executor.get_current();

    if (current == nullptr or current->kind == SegmentKind::STOP or current->kind == SegmentKind::ATTACH) {
        localizer.correct_at_rest(measurements, elapsed_time);

        if (this->objective != core::Objective::SOLVE and this->at_center and current != nullptr and
            localizer.get_cell().position != this->cell.position) {
            this->status = Status::FAILED;
        }
    }

    return this->status;
}

template <uint8_t width, uint8_t height>
const Reference& TMission<width, height>::get_reference() const {
    return this->reference;
}

template <uint8_t width, uint8_t height>
const Executor& TMission<width, height>::get_executor() const {
    return this->executor;
}

template <uint8_t width, uint8_t height>
const GridPose& TMission<width, height>::get_cell() const {
    return this->cell;
}

template <uint8_t width, uint8_t height>
Pose TMission<width, height>::get_entry_pose(const GridPose& cell) const {
    const float cell_size = this->dynamics.get_model().maze.cell_size;

    return this->get_center_pose(cell).compose({.position = {.x = -cell_size / 2.0F, .y = 0.0F}, .orientation = 0.0F});
}

template <uint8_t width, uint8_t height>
Pose TMission<width, height>::get_center_pose(const GridPose& cell) const {
    return {
        .position = cell.position.to_vector(this->dynamics.get_model().maze.cell_size),
        .orientation = static_cast<float>(std::to_underlying(cell.orientation)) * std::numbers::pi_v<float> / 2.0F,
    };
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::flood() {
    if (this->objective != core::Objective::RETURN) {
        this->maze.flood(this->maze.get_goal());
        return;
    }

    std::array<GridPoint, TExplorer<width, height>::max_targets> targets{};
    std::size_t                                                  number_of_targets = 0;

    if (this->explorer.has_targets()) {
        for (const GridPoint& target : this->explorer.get_targets()) {
            const bool worth_visiting = std::ranges::any_of(all_sides, [this, &target](Side side) {
                return this->maze.get_wall({.position = target, .orientation = side}) == WallState::UNKNOWN;
            });

            if (worth_visiting) {
                targets.at(number_of_targets++) = target;
            }
        }
    }

    if (number_of_targets == 0) {
        targets.at(number_of_targets++) = this->maze.get_start().position;
    }

    this->maze.flood(std::span{targets}.first(number_of_targets));
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::execute(Move& move, float start_speed, float end_speed) {
    const std::span<Segment> segments = std::span{move.segments}.first(move.size);

    VelocityPlanner::plan(segments, this->dynamics, this->config.search_profile, start_speed, end_speed);
    this->executor.push(segments);
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::decide_at_entry() {
    const float cell_size = this->dynamics.get_model().maze.cell_size;

    this->watching_front = false;
    this->looks = 0;

    if (this->finish_at_entry()) {
        return;
    }

    const std::optional<GridPose> next = this->maze.get_next(this->cell);

    if (not next.has_value()) {
        this->status = Status::FAILED;
        return;
    }

    const Pose entry = this->get_entry_pose(this->cell);
    Move       move{};

    if (next->orientation == this->cell.orientation) {
        move.add(make_segment(SegmentKind::STRAIGHT, cell_size, entry));
        this->execute(move, this->search_speed, this->search_speed);

        this->watching_front = this->maze.get_wall(this->cell) == WallState::UNKNOWN;
        this->watched_cell = this->cell;
        this->cell = *next;
        return;
    }

    if (next->orientation == this->cell.turned_back().orientation) {
        const GridPose reversed = this->cell.turned_back();

        this->add_stop_at_center(move);
        move.add(make_segment(SegmentKind::SPIN, std::numbers::pi_v<float>, this->get_center_pose(this->cell)));
        move.add(make_segment(SegmentKind::STRAIGHT, cell_size / 2.0F, this->get_center_pose(reversed)));
        this->execute(move, this->search_speed, this->search_speed);

        this->cell = *next;
        return;
    }

    if (this->maze.get_wall({.position = this->cell.position, .orientation = next->orientation}) !=
        WallState::NO_WALL) {
        this->add_stop_at_center(move);
        this->execute(move, this->search_speed, 0.0F);
        this->at_center = true;
        return;
    }

    const TurnShape& shape = this->dynamics.get_turn(this->config.search_profile, TurnId::SS90S);
    const bool       to_left = next->orientation == this->cell.turned_left().orientation;

    Segment turn = make_segment(
        SegmentKind::TURN, to_left ? shape.angle : -shape.angle,
        entry.compose({.position = {.x = shape.pre, .y = 0.0F}, .orientation = 0.0F})
    );
    turn.turn = TurnId::SS90S;

    if (shape.pre > min_straight) {
        move.add(make_segment(SegmentKind::STRAIGHT, shape.pre, entry));
    }

    move.add(turn);

    if (shape.post > min_straight) {
        move.add(make_segment(
            SegmentKind::STRAIGHT, shape.post,
            this->get_entry_pose(*next).compose({.position = {.x = -shape.post, .y = 0.0F}, .orientation = 0.0F})
        ));
    }

    this->execute(move, this->search_speed, this->search_speed);
    this->cell = *next;
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::decide_at_center() {
    const float cell_size = this->dynamics.get_model().maze.cell_size;
    const Pose  center = this->get_center_pose(this->cell);

    Move move{};

    if (this->objective == core::Objective::RETURN and not this->explorer.has_targets()) {
        move.add(make_segment(SegmentKind::STOP, this->config.look_time, center));
        this->execute(move, 0.0F, 0.0F);
        return;
    }

    const std::optional<GridPose> next = this->maze.get_next(this->cell);

    if (not next.has_value()) {
        this->status = Status::FAILED;
        return;
    }

    if (next->orientation != this->cell.orientation) {
        float angle = std::numbers::pi_v<float>;

        if (next->orientation == this->cell.turned_left().orientation) {
            angle = std::numbers::pi_v<float> / 2.0F;
        } else if (next->orientation == this->cell.turned_right().orientation) {
            angle = -std::numbers::pi_v<float> / 2.0F;
        }

        move.add(make_segment(SegmentKind::SPIN, angle, center));
        this->cell.orientation = next->orientation;
        move.add(make_segment(SegmentKind::STOP, this->config.stop_time, this->get_center_pose(this->cell)));
        this->execute(move, 0.0F, 0.0F);
        return;
    }

    if (this->maze.get_wall(this->cell) == WallState::NO_WALL) {
        move.add(make_segment(SegmentKind::STRAIGHT, cell_size / 2.0F, center));
        this->execute(move, 0.0F, this->search_speed);
        this->at_center = false;
        this->looks = 0;
        this->cell = *next;
        return;
    }

    this->looks++;

    if (this->looks > this->config.max_looks) {
        this->status = Status::FAILED;
        return;
    }

    move.add(make_segment(SegmentKind::STOP, this->config.look_time, center));
    this->execute(move, 0.0F, 0.0F);
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::add_stop_at_center(Move& move) const {
    const float cell_size = this->dynamics.get_model().maze.cell_size;
    const Pose  center = this->get_center_pose(this->cell);

    move.add(make_segment(SegmentKind::STRAIGHT, cell_size / 2.0F, this->get_entry_pose(this->cell)));

    if (this->maze.get_wall(this->cell) == WallState::WALL) {
        move.add(make_segment(SegmentKind::ATTACH, this->config.attach_time, center));
    } else {
        move.add(make_segment(SegmentKind::STOP, this->config.stop_time, center));
    }
}

template <uint8_t width, uint8_t height>
bool TMission<width, height>::finish_at_entry() {
    const float cell_size = this->dynamics.get_model().maze.cell_size;

    Move move{};

    if (this->objective == core::Objective::EXPLORE and this->maze.is_goal(this->cell.position)) {
        this->add_stop_at_center(move);
    } else if (
        this->objective == core::Objective::RETURN and this->explorer.is_complete() and
        this->cell.position == this->maze.get_start().position
    ) {
        const GridPose parked{.position = this->cell.position, .orientation = this->maze.get_start().orientation};

        this->add_stop_at_center(move);
        move.add(make_segment(SegmentKind::SPIN, std::numbers::pi_v<float>, this->get_center_pose(this->cell)));
        move.add(make_segment(SegmentKind::STOP, this->config.stop_time, this->get_center_pose(parked)));
        move.add(make_segment(
            SegmentKind::STRAIGHT, this->config.start_offset - cell_size / 2.0F, this->get_center_pose(parked)
        ));
    } else {
        return false;
    }

    this->execute(move, this->search_speed, 0.0F);
    this->at_center = true;
    this->finishing = true;

    return true;
}

template <uint8_t width, uint8_t height>
void TMission<width, height>::divert_to_center() {
    const float      cell_size = this->dynamics.get_model().maze.cell_size;
    const Reference& current = this->executor.get_reference();
    const float      remaining = cell_size / 2.0F - current.distance;

    this->watching_front = false;

    if (remaining <= 0.0F) {
        return;
    }

    const Pose center = this->get_center_pose(this->watched_cell);

    Move move{};
    move.add(make_segment(
        SegmentKind::STRAIGHT, remaining,
        this->get_entry_pose(this->watched_cell)
            .compose({.position = {.x = current.distance, .y = 0.0F}, .orientation = 0.0F})
    ));

    if (this->maze.get_wall(this->watched_cell) == WallState::WALL) {
        move.add(make_segment(SegmentKind::ATTACH, this->config.attach_time, center));
    } else {
        move.add(make_segment(SegmentKind::STOP, this->config.stop_time, center));
    }

    const std::span<Segment> segments = std::span{move.segments}.first(move.size);

    VelocityPlanner::plan(segments, this->dynamics, this->config.search_profile, current.twist.linear, 0.0F);
    this->executor.divert(segments);

    this->cell = this->watched_cell;
    this->at_center = true;
}

template <uint8_t width, uint8_t height>
Segment TMission<width, height>::make_segment(SegmentKind kind, float length, const Pose& start) {
    return {
        .kind = kind,
        .turn = TurnId::SS90S,
        .length = length,
        .start_speed = 0.0F,
        .end_speed = 0.0F,
        .max_speed = 0.0F,
        .start = start,
    };
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MISSION_TPP
