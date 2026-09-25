/**
 * @file
 */

#ifndef MICRAS_NAV_PLANNER_TPP
#define MICRAS_NAV_PLANNER_TPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>

#include "micras/nav/lattice.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/speed_profile.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TPlanner<width, height>::TPlanner(const Dynamics& dynamics, const Config& config) :
    dynamics{dynamics}, start_distance{config.start_distance} { }

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::begin(
    const TMaze<width, height>& maze, WallAssumption assumption, const RunProfile& profile
) {
    this->maze = &maze;
    this->assumption = assumption;
    this->run_profile = profile;
    this->limits = this->dynamics.get_linear_limits(profile);

    for (uint8_t i = 0; i < number_of_turns; i++) {
        this->turn_speeds.at(i) = this->dynamics.get_turn_speed(profile, static_cast<TurnId>(i));
    }

    this->number_of_usable_turns.fill(0);

    for (uint8_t i = 0; i < number_of_turns; i++) {
        const auto    turn = static_cast<TurnId>(i);
        const uint8_t diagonal = get_primitive(turn).diagonal_entry ? 1 : 0;

        if (this->is_usable(turn)) {
            this->usable_turns.at(diagonal).at(this->number_of_usable_turns.at(diagonal)) = turn;
            this->number_of_usable_turns.at(diagonal)++;
        }
    }

    for (auto& costs : this->edge_costs) {
        costs.fill(-1.0F);
    }

    for (auto& row : this->gaps) {
        row.fill(-std::numeric_limits<float>::infinity());
    }

    this->preparing = true;
    this->prepare_first = 0;
    this->prepare_second = 0;
    this->prepare_run = 0;
    this->expanding = none;

    this->goal_length = 1;

    for (const GridPoint& cell : maze.get_goal()) {
        for (const Side side : {Side::RIGHT, Side::UP, Side::LEFT, Side::DOWN}) {
            GridPose pose{.position = cell, .orientation = side};
            uint8_t  length = 1;

            while (TMaze<width, height>::contains(pose.front().position) and maze.is_goal(pose.front().position)) {
                pose = pose.front();
                length++;
            }

            this->goal_length = std::max(this->goal_length, length);
        }
    }

    this->heads.fill(none);
    this->queue_size = 0;
    this->allocated = 0;
    this->free_list = none;
    this->held = 0;
    this->peak = 0;
    this->exact = true;
    this->number_of_terminals = 0;

    const GridPose&   start_pose = maze.get_start();
    const auto        heading = static_cast<uint8_t>(2 * std::to_underlying(start_pose.orientation));
    const LatticePose center{
        .point =
            {.x = static_cast<int8_t>(2 * start_pose.position.x + 1),
             .y = static_cast<int8_t>(2 * start_pose.position.y + 1)},
        .heading = heading,
    };
    const LatticePoint step = center.step();

    this->start = {
        .point =
            {.x = static_cast<int8_t>(center.point.x + step.x / 2),
             .y = static_cast<int8_t>(center.point.y + step.y / 2)},
        .heading = heading,
    };

    if (is_inside(this->start) and this->is_traversable(this->start)) {
        this->insert({
            .cost = 0.0F,
            .node = encode(this->start),
            .parent = none,
            .next = none,
            .position = none,
            .arrival = rest,
            .speed_ratio = full_speed,
            .run = 0,
            .side = TurnSide::LEFT,
        });
    }
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::step(uint32_t max_edges) {
    for (; max_edges > 0; max_edges--) {
        if (this->preparing) {
            this->prepare();
            continue;
        }

        if (this->expanding == none) {
            if (this->queue_size == 0) {
                break;
            }

            const uint16_t index = this->pop();

            if (this->number_of_terminals == number_of_candidates and
                this->labels.at(index).cost >= this->terminals.at(number_of_candidates - 1).cost) {
                this->queue_size = 0;
                break;
            }

            this->expanding = index;
            this->expansion_entry = decode(this->labels.at(index).node);
            this->expansion_run = 0;
            this->expansion_turn = 0;
        }

        this->advance_expansion();
    }

    return this->is_finished();
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_finished() const {
    return not this->preparing and this->expanding == none and this->queue_size == 0;
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_exact() const {
    return this->exact;
}

template <uint8_t width, uint8_t height>
uint16_t TPlanner<width, height>::get_peak_labels() const {
    return this->peak;
}

template <uint8_t width, uint8_t height>
uint8_t TPlanner<width, height>::get_number_of_routes() const {
    return this->number_of_terminals;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::get_route(uint8_t index, Route& route) const {
    const Terminal& terminal = this->terminals.at(index);

    route.start = this->start;
    route.steps.clear();
    route.stop_distance = terminal.stop_distance;
    route.finish_distance = terminal.finish_distance;
    route.time = terminal.cost;

    route.steps.push_back(
        {.run = terminal.run, .has_turn = terminal.has_turn, .turn = terminal.turn, .side = terminal.side}
    );

    for (uint16_t i = terminal.label; this->labels.at(i).arrival != rest; i = this->labels.at(i).parent) {
        const Label& label = this->labels.at(i);
        route.steps.push_back({.run = label.run, .has_turn = true, .turn = to_turn(label.arrival), .side = label.side});
    }

    std::ranges::reverse(route.steps);
}

template <uint8_t width, uint8_t height>
template <typename F>
void TPlanner<width, height>::for_each_wall(const Route& route, F&& function) {
    LatticePose node = route.start;
    function(node.wall());

    for (const RouteStep& step : route.steps) {
        for (uint8_t i = 0; i < step.run; i++) {
            node = node.advanced();
            function(node.wall());
        }

        if (not step.has_turn) {
            continue;
        }

        for (uint8_t i = 0; i < get_primitive(step.turn).number_of_gates; i++) {
            function(LatticePose{.point = get_turn_gate(node, step.turn, step.side, i), .heading = 0}.wall());
        }

        node = get_turn_exit(node, step.turn, step.side);
        function(node.wall());
    }
}

template <uint8_t width, uint8_t height>
constexpr bool TPlanner<width, height>::ends_diagonal(uint8_t arrival) {
    if (arrival == rest) {
        return false;
    }

    const TurnPrimitive& primitive = get_primitive(to_turn(arrival));
    return (primitive.rotation % 2 == 1) != primitive.diagonal_entry;
}

template <uint8_t width, uint8_t height>
constexpr TurnId TPlanner<width, height>::to_turn(uint8_t arrival) {
    return static_cast<TurnId>(arrival - 1);
}

template <uint8_t width, uint8_t height>
constexpr bool TPlanner<width, height>::is_inside(const LatticePose& node) {
    return node.point.x >= 0 and node.point.x <= 2 * width and node.point.y >= 0 and node.point.y <= 2 * height and
           (node.point.on_vertical_wall() or node.point.on_horizontal_wall());
}

template <uint8_t width, uint8_t height>
constexpr uint16_t TPlanner<width, height>::encode(const LatticePose& node) {
    const LatticePoint& point = node.point;

    const int32_t wall = point.on_vertical_wall() ?
                             (point.x / 2) * height + (point.y - 1) / 2 :
                             (width + 1) * height + ((point.x - 1) / 2) * (height + 1) + point.y / 2;

    const int32_t sub = node.heading % 2 == 0 ? node.heading / 4 : 2 + node.heading / 2;

    return static_cast<uint16_t>(wall * headings_per_wall + sub);
}

template <uint8_t width, uint8_t height>
constexpr LatticePose TPlanner<width, height>::decode(uint16_t index) {
    const uint16_t wall = index / headings_per_wall;
    const uint16_t sub = index % headings_per_wall;

    LatticePose node{};

    if (wall < (width + 1) * height) {
        node.point = {.x = static_cast<int8_t>(2 * (wall / height)), .y = static_cast<int8_t>(2 * (wall % height) + 1)};
    } else {
        const uint16_t horizontal = wall - (width + 1) * height;

        node.point = {
            .x = static_cast<int8_t>(2 * (horizontal / (height + 1)) + 1),
            .y = static_cast<int8_t>(2 * (horizontal % (height + 1))),
        };
    }

    node.heading = sub < 2 ? static_cast<uint8_t>((node.point.on_vertical_wall() ? 0 : 2) + 4 * sub) :
                             static_cast<uint8_t>(2 * (sub - 2) + 1);

    return node;
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_usable(TurnId turn) const {
    return this->dynamics.get_turn(this->run_profile, turn).valid;
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_traversable(const LatticePose& node) const {
    const GridPose wall = node.wall();

    return this->assumption == WallAssumption::OPTIMISTIC ? this->maze->is_possibly_open(wall) :
                                                            this->maze->is_known_open(wall);
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_goal(const LatticePoint& cell) const {
    return cell.x >= 0 and cell.y >= 0 and
           this->maze->is_goal({.x = static_cast<uint8_t>(cell.x), .y = static_cast<uint8_t>(cell.y)});
}

template <uint8_t width, uint8_t height>
float TPlanner<width, height>::get_stop_distance(const LatticePose& node) const {
    const float        cell_size = this->dynamics.get_model().maze.cell_size;
    const LatticePoint step = node.step();

    LatticePoint cell = node.cell_ahead();
    LatticePose  wall = node;
    uint8_t      cells = 0;

    while (this->is_goal(cell)) {
        cells++;
        wall = wall.advanced();

        if (not is_inside(wall) or not this->maze->is_known_open(wall.wall())) {
            break;
        }

        cell = {.x = static_cast<int8_t>(cell.x + step.x / 2), .y = static_cast<int8_t>(cell.y + step.y / 2)};
    }

    return (static_cast<float>(cells) - 0.5F) * cell_size;
}

template <uint8_t width, uint8_t height>
float TPlanner<width, height>::get_speed(uint8_t arrival, uint8_t speed_ratio) const {
    if (arrival == rest) {
        return 0.0F;
    }

    return this->turn_speeds.at(arrival - 1) * static_cast<float>(speed_ratio) / static_cast<float>(full_speed);
}

template <uint8_t width, uint8_t height>
float TPlanner<width, height>::get_offset(uint8_t arrival) const {
    return arrival == rest ? this->start_distance : this->dynamics.get_turn(this->run_profile, to_turn(arrival)).post;
}

template <uint8_t width, uint8_t height>
TPlanner<width, height>::Edge
    TPlanner<width, height>::get_edge(uint8_t arrival, uint8_t speed_ratio, uint8_t run, TurnId turn) {
    const bool    at_full_speed = arrival == rest or speed_ratio == full_speed;
    const int16_t pair = edge_pairs.index.at(arrival).at(std::to_underlying(turn));
    float&        cached_cost = this->edge_costs.at(pair).at(run);
    uint8_t&      cached_speed_ratio = this->edge_speed_ratios.at(pair).at(run);

    if (at_full_speed and cached_cost >= 0.0F) {
        return {.cost = cached_cost, .speed_ratio = cached_speed_ratio};
    }

    const float cell_size = this->dynamics.get_model().maze.cell_size;
    const float step = ends_diagonal(arrival) ? cell_size / std::numbers::sqrt2_v<float> : cell_size;

    const TurnShape& shape = this->dynamics.get_turn(this->run_profile, turn);
    const float      distance = this->get_offset(arrival) + static_cast<float>(run) * step + shape.pre;
    const float      nominal_speed = this->turn_speeds.at(std::to_underlying(turn));

    float start_speed = this->get_speed(arrival, speed_ratio);
    float penalty = 0.0F;

    const float brakeable = SpeedProfile::get_brakeable_speed(distance, nominal_speed, this->limits);

    if (start_speed > brakeable) {
        const float previous_length = this->dynamics.get_turn(this->run_profile, to_turn(arrival)).length();
        penalty = previous_length * (1.0F / brakeable - 1.0F / start_speed);
        start_speed = brakeable;
    }

    const float end_speed =
        std::min(nominal_speed, SpeedProfile::get_reachable_speed(distance, start_speed, this->limits));

    const Edge edge{
        .cost = SpeedProfile{distance, start_speed, end_speed, this->limits}.duration() + shape.length() / end_speed +
                penalty,
        .speed_ratio = static_cast<uint8_t>(std::lround(static_cast<float>(full_speed) * end_speed / nominal_speed)),
    };

    if (at_full_speed) {
        cached_cost = edge.cost;
        cached_speed_ratio = edge.speed_ratio;
    }

    return edge;
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_comparable(uint8_t first, uint8_t second) const {
    const auto is_reachable = [this](uint8_t arrival) { return arrival == rest or this->is_usable(to_turn(arrival)); };

    return first != second and ends_diagonal(first) == ends_diagonal(second) and is_reachable(first) and
           is_reachable(second);
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::prepare() {
    while (this->prepare_first < number_of_arrivals and
           not this->is_comparable(this->prepare_first, this->prepare_second)) {
        this->prepare_second++;

        if (this->prepare_second == number_of_arrivals) {
            this->prepare_second = 0;
            this->prepare_first++;
        }
    }

    if (this->prepare_first == number_of_arrivals) {
        this->preparing = false;
        return;
    }

    this->compare_arrivals(this->prepare_first, this->prepare_second, this->prepare_run);
    this->prepare_run++;

    const uint8_t longest = ends_diagonal(this->prepare_first) ? max_run : max_straight_run;

    if (this->prepare_run > longest or std::isinf(this->gaps.at(this->prepare_first).at(this->prepare_second))) {
        this->prepare_run = 0;
        this->prepare_second++;

        if (this->prepare_second == number_of_arrivals) {
            this->prepare_second = 0;
            this->prepare_first++;
        }
    }
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::compare_arrivals(uint8_t first, uint8_t second, uint8_t run) {
    float& gap = this->gaps.at(first).at(second);

    const bool    diagonal = ends_diagonal(first);
    const uint8_t kind = diagonal ? 1 : 0;

    for (uint8_t i = 0; i < this->number_of_usable_turns.at(kind); i++) {
        const TurnId         turn = this->usable_turns.at(kind).at(i);
        const TurnPrimitive& primitive = get_primitive(turn);
        const auto           arrival = static_cast<uint8_t>(std::to_underlying(turn) + 1);

        const Edge kept = this->get_edge(first, full_speed, run, turn);
        const Edge dropped = this->get_edge(second, full_speed, run, turn);

        if (kept.speed_ratio < dropped.speed_ratio) {
            gap = std::numeric_limits<float>::infinity();
            return;
        }

        const TurnShape& shape = this->dynamics.get_turn(this->run_profile, turn);
        const float      kept_speed = this->get_speed(arrival, kept.speed_ratio);
        const float      dropped_speed = this->get_speed(arrival, dropped.speed_ratio);

        gap =
            std::max(gap, (kept.cost - shape.length() / kept_speed) - (dropped.cost - shape.length() / dropped_speed));

        for (uint8_t j = 0; j < primitive.number_of_gates; j++) {
            const float remaining = shape.pre + shape.length() - std::max(shape.gates.at(j), shape.pre);
            gap = std::max(gap, (kept.cost - remaining / kept_speed) - (dropped.cost - remaining / dropped_speed));
        }
    }

    if (diagonal) {
        return;
    }

    const float cell_size = this->dynamics.get_model().maze.cell_size;

    for (uint8_t cells = 1; cells <= this->goal_length; cells++) {
        const float stop_distance = (static_cast<float>(cells) - 0.5F) * cell_size;
        const float kept_line = this->get_offset(first) + static_cast<float>(run) * cell_size;
        const float dropped_line = this->get_offset(second) + static_cast<float>(run) * cell_size;

        const SpeedProfile kept{kept_line + stop_distance, this->get_speed(first, full_speed), 0.0F, this->limits};
        const SpeedProfile dropped{
            dropped_line + stop_distance, this->get_speed(second, full_speed), 0.0F, this->limits
        };

        gap = std::max(gap, kept.time_at(kept_line) - dropped.time_at(dropped_line));
    }
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::dominates(const Label& first, const Label& second) {
    if (first.arrival == second.arrival) {
        if (first.speed_ratio < second.speed_ratio) {
            return false;
        }

        if (first.arrival == rest) {
            return first.cost <= second.cost;
        }

        const float length = this->dynamics.get_turn(this->run_profile, to_turn(first.arrival)).length();

        return first.cost - length / this->get_speed(first.arrival, first.speed_ratio) <=
               second.cost - length / this->get_speed(second.arrival, second.speed_ratio);
    }

    if (first.speed_ratio != full_speed) {
        return false;
    }

    float second_cost = second.cost;

    if (second.speed_ratio != full_speed) {
        const float length = this->dynamics.get_turn(this->run_profile, to_turn(second.arrival)).length();

        second_cost -= length / this->get_speed(second.arrival, second.speed_ratio) -
                       length / this->get_speed(second.arrival, full_speed);
    }

    return first.cost + this->gaps.at(first.arrival).at(second.arrival) <= second_cost;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::advance_expansion() {
    const Label   label = this->labels.at(this->expanding);
    const bool    diagonal = this->expansion_entry.is_diagonal();
    const uint8_t kind = diagonal ? 1 : 0;

    if (this->expansion_turn == 0) {
        if (this->expansion_run > 0) {
            this->expansion_entry = this->expansion_entry.advanced();

            if (not is_inside(this->expansion_entry) or not this->is_traversable(this->expansion_entry)) {
                this->expanding = none;
                return;
            }
        }

        if (this->is_goal(this->expansion_entry.cell_ahead())) {
            if (not diagonal) {
                const float cell_size = this->dynamics.get_model().maze.cell_size;
                const float line =
                    this->get_offset(label.arrival) + static_cast<float>(this->expansion_run) * cell_size;
                const float stop_distance = this->get_stop_distance(this->expansion_entry);

                const SpeedProfile approach{
                    line + stop_distance, this->get_speed(label.arrival, label.speed_ratio), 0.0F, this->limits
                };

                this->add_terminal({
                    .cost = label.cost + approach.time_at(line),
                    .label = this->expanding,
                    .run = this->expansion_run,
                    .has_turn = false,
                    .turn = TurnId::SS90S,
                    .side = TurnSide::LEFT,
                    .stop_distance = stop_distance,
                    .finish_distance = stop_distance,
                });
            }

            this->expanding = none;
            return;
        }
    }

    if (this->expansion_turn < this->number_of_usable_turns.at(kind)) {
        this->relax_turn(
            this->expanding, this->expansion_entry, this->expansion_run,
            this->usable_turns.at(kind).at(this->expansion_turn)
        );
        this->expansion_turn++;
    }

    if (this->expansion_turn >= this->number_of_usable_turns.at(kind)) {
        this->expansion_turn = 0;
        this->expansion_run++;

        if (this->expansion_run > max_run) {
            this->expanding = none;
        }
    }
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::relax_turn(uint16_t index, const LatticePose& entry, uint8_t run, TurnId turn) {
    const TurnPrimitive& primitive = get_primitive(turn);

    Edge edge{.cost = -1.0F, .speed_ratio = 0};

    for (const TurnSide side : {TurnSide::LEFT, TurnSide::RIGHT}) {
        if (primitive.diagonal_entry and side != entry.diagonal_turn_side()) {
            continue;
        }

        const LatticePose exit = get_turn_exit(entry, turn, side);

        if (not is_inside(exit) or not exit.is_valid() or not this->is_traversable(exit)) {
            continue;
        }

        LatticePoint cell = entry.cell_ahead();
        int8_t       finish_gate = -1;
        bool         feasible = true;

        for (uint8_t i = 0; i < primitive.number_of_gates and feasible; i++) {
            const LatticePose gate{.point = get_turn_gate(entry, turn, side, i), .heading = 0};

            if (not is_inside(gate) or not this->is_traversable(gate)) {
                feasible = false;
                break;
            }

            const bool         vertical = gate.point.on_vertical_wall();
            const LatticePoint low{
                .x = static_cast<int8_t>(vertical ? gate.point.x / 2 - 1 : (gate.point.x - 1) / 2),
                .y = static_cast<int8_t>(vertical ? (gate.point.y - 1) / 2 : gate.point.y / 2 - 1),
            };
            const LatticePoint high{
                .x = static_cast<int8_t>(vertical ? low.x + 1 : low.x),
                .y = static_cast<int8_t>(vertical ? low.y : low.y + 1)
            };

            cell = cell == low ? high : low;

            if (this->is_goal(cell)) {
                finish_gate = finish_gate < 0 ? static_cast<int8_t>(i) : finish_gate;
            } else if (finish_gate >= 0) {
                feasible = false;
            }
        }

        if (not feasible) {
            continue;
        }

        const Label& label = this->labels.at(index);

        if (edge.cost < 0.0F) {
            edge = this->get_edge(label.arrival, label.speed_ratio, run, turn);
        }

        const float cost = label.cost + edge.cost;

        if (finish_gate >= 0) {
            if (exit.is_diagonal() or not this->is_goal(exit.cell_ahead())) {
                continue;
            }

            const TurnShape& shape = this->dynamics.get_turn(this->run_profile, turn);
            const float      remaining =
                shape.pre + shape.length() - std::max(shape.gates.at(static_cast<uint8_t>(finish_gate)), shape.pre);
            const float speed = this->get_speed(std::to_underlying(turn) + 1, edge.speed_ratio);

            const float stop_distance = this->get_stop_distance(exit);

            this->add_terminal({
                .cost = cost - remaining / speed,
                .label = index,
                .run = run,
                .has_turn = true,
                .turn = turn,
                .side = side,
                .stop_distance = stop_distance,
                .finish_distance = remaining + shape.post + stop_distance,
            });

            continue;
        }

        this->insert({
            .cost = cost,
            .node = encode(exit),
            .parent = index,
            .next = none,
            .position = none,
            .arrival = static_cast<uint8_t>(std::to_underlying(turn) + 1),
            .speed_ratio = edge.speed_ratio,
            .run = run,
            .side = side,
        });
    }
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::add_terminal(const Terminal& terminal) {
    uint8_t position = this->number_of_terminals;

    while (position > 0 and this->terminals.at(position - 1).cost > terminal.cost) {
        if (position < number_of_candidates) {
            this->terminals.at(position) = this->terminals.at(position - 1);
        }

        position--;
    }

    if (position < number_of_candidates) {
        this->terminals.at(position) = terminal;
        this->number_of_terminals = std::min<uint8_t>(this->number_of_terminals + 1, number_of_candidates);
    }
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::insert(Label label) {
    uint16_t previous = none;
    uint16_t current = this->heads.at(label.node);

    while (current != none) {
        const Label&   other = this->labels.at(current);
        const uint16_t next = other.next;

        if (this->dominates(other, label)) {
            return;
        }

        if (other.position != none and this->dominates(label, other)) {
            if (previous == none) {
                this->heads.at(label.node) = next;
            } else {
                this->labels.at(previous).next = next;
            }

            this->remove(other.position);
            this->labels.at(current).next = this->free_list;
            this->free_list = current;
            this->held--;
        } else {
            previous = current;
        }

        current = next;
    }

    const uint16_t index = this->allocate(label.cost);

    if (index == none) {
        return;
    }

    label.next = this->heads.at(label.node);
    this->labels.at(index) = label;
    this->heads.at(label.node) = index;
    this->push(index);
}

template <uint8_t width, uint8_t height>
uint16_t TPlanner<width, height>::allocate(float cost) {
    if (this->free_list == none and this->allocated == max_labels) {
        this->exact = false;

        uint16_t costliest = none;

        for (uint16_t position = this->queue_size / 2; position < this->queue_size; position++) {
            const uint16_t index = this->queue.at(position);

            if (costliest == none or this->labels.at(index).cost > this->labels.at(costliest).cost) {
                costliest = index;
            }
        }

        if (costliest == none or this->labels.at(costliest).cost <= cost) {
            return none;
        }

        this->release(costliest);
    }

    uint16_t index = this->free_list;

    if (index != none) {
        this->free_list = this->labels.at(index).next;
    } else {
        index = this->allocated;
        this->allocated++;
    }

    this->held++;
    this->peak = std::max(this->peak, this->held);

    return index;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::release(uint16_t index) {
    Label&    label = this->labels.at(index);
    uint16_t* link = &this->heads.at(label.node);

    while (*link != index) {
        link = &this->labels.at(*link).next;
    }

    *link = label.next;
    this->remove(label.position);
    label.next = this->free_list;
    this->free_list = index;
    this->held--;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::push(uint16_t index) {
    this->queue.at(this->queue_size) = index;
    this->labels.at(index).position = this->queue_size;
    this->queue_size++;
    this->sift_up(this->queue_size - 1);
}

template <uint8_t width, uint8_t height>
uint16_t TPlanner<width, height>::pop() {
    const uint16_t top = this->queue.at(0);
    this->remove(0);
    return top;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::remove(uint16_t position) {
    this->labels.at(this->queue.at(position)).position = none;
    this->queue_size--;

    if (position == this->queue_size) {
        return;
    }

    const uint16_t last = this->queue.at(this->queue_size);
    this->queue.at(position) = last;
    this->labels.at(last).position = position;
    this->sift_down(position);
    this->sift_up(this->labels.at(last).position);
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::sift_up(uint16_t position) {
    const uint16_t index = this->queue.at(position);
    const float    cost = this->labels.at(index).cost;

    while (position > 0) {
        const uint16_t parent = (position - 1) / 2;

        if (this->labels.at(this->queue.at(parent)).cost <= cost) {
            break;
        }

        this->queue.at(position) = this->queue.at(parent);
        this->labels.at(this->queue.at(position)).position = position;
        position = parent;
    }

    this->queue.at(position) = index;
    this->labels.at(index).position = position;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::sift_down(uint16_t position) {
    const uint16_t index = this->queue.at(position);
    const float    cost = this->labels.at(index).cost;

    while (2 * position + 1 < this->queue_size) {
        uint16_t child = 2 * position + 1;

        if (child + 1 < this->queue_size and
            this->labels.at(this->queue.at(child + 1)).cost < this->labels.at(this->queue.at(child)).cost) {
            child++;
        }

        if (cost <= this->labels.at(this->queue.at(child)).cost) {
            break;
        }

        this->queue.at(position) = this->queue.at(child);
        this->labels.at(this->queue.at(position)).position = position;
        position = child;
    }

    this->queue.at(position) = index;
    this->labels.at(index).position = position;
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_PLANNER_TPP
