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

    for (auto& runs : this->edges) {
        for (auto& turns : runs) {
            turns.fill({.cost = -1.0F, .speed_ratio = 0});
        }
    }

    this->cleared = 0;
    this->queue_size = 0;
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
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::step(uint32_t max_nodes) {
    while (this->cleared < number_of_states and max_nodes > 0) {
        const uint16_t end = std::min<uint32_t>(this->cleared + states_per_node_budget, number_of_states);

        std::fill(
            this->costs.begin() + this->cleared, this->costs.begin() + end, std::numeric_limits<float>::infinity()
        );
        std::fill(this->positions.begin() + this->cleared, this->positions.begin() + end, not_queued);

        this->cleared = end;
        max_nodes--;

        if (this->cleared == number_of_states and is_inside(this->start) and this->is_traversable(this->start)) {
            this->relax(encode({.node = this->start, .arrival = Arrival::REST}), 0.0F, 0, full_speed);
        }
    }

    for (uint32_t i = 0; i < max_nodes and this->queue_size > 0; i++) {
        const uint16_t index = this->pop();

        if (this->number_of_terminals == number_of_candidates and
            this->costs.at(index) >= this->terminals.at(number_of_candidates - 1).cost) {
            this->queue_size = 0;
            break;
        }

        this->expand(index);
    }

    return this->is_finished();
}

template <uint8_t width, uint8_t height>
bool TPlanner<width, height>::is_finished() const {
    return this->cleared == number_of_states and this->queue_size == 0;
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

    uint16_t index_of_state = terminal.state;

    while (true) {
        const State        state = decode(index_of_state);
        const LatticePose& node = state.node;
        const Arrival      arrival = state.arrival;

        if (arrival == Arrival::REST) {
            break;
        }

        const uint16_t link = this->links.at(index_of_state);
        const auto     run = static_cast<uint8_t>(link & 0x3FU);
        const auto     previous = static_cast<Arrival>((link >> 6U) & 0x0FU);
        const TurnSide side = (link >> 10U) != 0 ? TurnSide::RIGHT : TurnSide::LEFT;
        const TurnId   turn = to_turn(arrival);

        route.steps.push_back({.run = run, .has_turn = true, .turn = turn, .side = side});

        const TurnPrimitive& primitive = get_primitive(turn);
        const uint8_t        rotation =
            side == TurnSide::LEFT ? primitive.rotation : LatticePose::number_of_headings - primitive.rotation;
        const auto entry_heading = static_cast<uint8_t>(
            (node.heading + LatticePose::number_of_headings - rotation) % LatticePose::number_of_headings
        );
        const LatticePoint offset = from_canonical(primitive.exit, primitive.diagonal_entry, entry_heading, side);
        const LatticePoint step = LatticePose{.point = {}, .heading = entry_heading}.step();

        const LatticePose origin{
            .point =
                {.x = static_cast<int8_t>(node.point.x - offset.x - run * step.x),
                 .y = static_cast<int8_t>(node.point.y - offset.y - run * step.y)},
            .heading = entry_heading,
        };

        index_of_state = encode({.node = origin, .arrival = previous});
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
constexpr TPlanner<width, height>::Arrival TPlanner<width, height>::to_arrival(TurnId turn) {
    constexpr std::array<Arrival, number_of_turns> arrivals{
        Arrival::SS90S, Arrival::SS90L, Arrival::SS180, Arrival::SD45,
        Arrival::SD135, Arrival::DS45,  Arrival::DS135, Arrival::DD90,
    };

    return arrivals.at(std::to_underlying(turn));
}

template <uint8_t width, uint8_t height>
constexpr TurnId TPlanner<width, height>::to_turn(Arrival arrival) {
    constexpr std::array<TurnId, std::to_underlying(Arrival::NUMBER_OF_ARRIVALS)> turns{
        TurnId::SS90S, TurnId::SS90S, TurnId::SS90L, TurnId::SS180, TurnId::DS45,
        TurnId::DS135, TurnId::SD45,  TurnId::SD135, TurnId::DD90,
    };

    return turns.at(std::to_underlying(arrival));
}

template <uint8_t width, uint8_t height>
constexpr bool TPlanner<width, height>::is_inside(const LatticePose& node) {
    return node.point.x >= 0 and node.point.x <= 2 * width and node.point.y >= 0 and node.point.y <= 2 * height and
           (node.point.on_vertical_wall() or node.point.on_horizontal_wall());
}

template <uint8_t width, uint8_t height>
constexpr uint16_t TPlanner<width, height>::encode(const State& state) {
    const LatticePoint& point = state.node.point;
    const uint8_t       heading = state.node.heading;

    const int32_t wall = point.on_vertical_wall() ?
                             (point.x / 2) * height + (point.y - 1) / 2 :
                             (width + 1) * height + ((point.x - 1) / 2) * (height + 1) + point.y / 2;

    const int32_t sub = heading % 2 == 0 ? (heading / 4) * 6 + std::to_underlying(state.arrival) :
                                           12 + (heading / 2) * 3 + (std::to_underlying(state.arrival) - 6);

    return static_cast<uint16_t>(wall * states_per_wall + sub);
}

template <uint8_t width, uint8_t height>
constexpr TPlanner<width, height>::State TPlanner<width, height>::decode(uint16_t index) {
    const uint16_t wall = index / states_per_wall;
    const uint16_t sub = index % states_per_wall;

    State state{};

    if (wall < (width + 1) * height) {
        state.node.point = {
            .x = static_cast<int8_t>(2 * (wall / height)), .y = static_cast<int8_t>(2 * (wall % height) + 1)
        };
    } else {
        const uint16_t horizontal = wall - (width + 1) * height;

        state.node.point = {
            .x = static_cast<int8_t>(2 * (horizontal / (height + 1)) + 1),
            .y = static_cast<int8_t>(2 * (horizontal % (height + 1))),
        };
    }

    if (sub < 12) {
        state.node.heading = static_cast<uint8_t>((state.node.point.on_vertical_wall() ? 0 : 2) + 4 * (sub / 6));
        state.arrival = static_cast<Arrival>(sub % 6);
    } else {
        state.node.heading = static_cast<uint8_t>(2 * ((sub - 12) / 3) + 1);
        state.arrival = static_cast<Arrival>(6 + (sub - 12) % 3);
    }

    return state;
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
float TPlanner<width, height>::get_speed(uint16_t index, Arrival arrival) const {
    if (arrival == Arrival::REST) {
        return 0.0F;
    }

    return this->turn_speeds.at(std::to_underlying(to_turn(arrival))) *
           static_cast<float>(this->speed_ratios.at(index)) / static_cast<float>(full_speed);
}

template <uint8_t width, uint8_t height>
float TPlanner<width, height>::get_offset(Arrival arrival) const {
    return arrival == Arrival::REST ? this->start_distance :
                                      this->dynamics.get_turn(this->run_profile, to_turn(arrival)).post;
}

template <uint8_t width, uint8_t height>
TPlanner<width, height>::Edge
    TPlanner<width, height>::get_edge(uint16_t index, Arrival arrival, uint8_t run, TurnId turn) {
    const bool at_full_speed = arrival == Arrival::REST or this->speed_ratios.at(index) == full_speed;
    Edge&      cached = this->edges.at(std::to_underlying(arrival)).at(run).at(std::to_underlying(turn));

    if (at_full_speed and cached.cost >= 0.0F) {
        return cached;
    }

    const float cell_size = this->dynamics.get_model().maze.cell_size;
    const bool  diagonal = std::to_underlying(arrival) >= std::to_underlying(Arrival::SD45);
    const float step = diagonal ? cell_size / std::numbers::sqrt2_v<float> : cell_size;

    const TurnShape& shape = this->dynamics.get_turn(this->run_profile, turn);
    const float      distance = this->get_offset(arrival) + static_cast<float>(run) * step + shape.pre;
    const float      nominal_speed = this->turn_speeds.at(std::to_underlying(turn));

    float start_speed = this->get_speed(index, arrival);
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
        cached = edge;
    }

    return edge;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::expand(uint16_t index) {
    const State state = decode(index);

    const bool  diagonal = state.node.is_diagonal();
    const float cell_size = this->dynamics.get_model().maze.cell_size;

    LatticePose entry = state.node;

    for (uint8_t run = 0; run <= max_run; run++) {
        if (run > 0) {
            entry = entry.advanced();

            if (not is_inside(entry) or not this->is_traversable(entry)) {
                break;
            }
        }

        if (this->is_goal(entry.cell_ahead())) {
            if (not diagonal) {
                const float line = this->get_offset(state.arrival) + static_cast<float>(run) * cell_size;
                const float stop_distance = this->get_stop_distance(entry);

                const SpeedProfile approach{
                    line + stop_distance, this->get_speed(index, state.arrival), 0.0F, this->limits
                };

                this->add_terminal({
                    .cost = this->costs.at(index) + approach.time_at(line),
                    .state = index,
                    .run = run,
                    .has_turn = false,
                    .turn = TurnId::SS90S,
                    .side = TurnSide::LEFT,
                    .stop_distance = stop_distance,
                    .finish_distance = stop_distance,
                });
            }

            break;
        }

        if (diagonal) {
            this->relax_turn(index, state, entry, run, TurnId::DS45);
            this->relax_turn(index, state, entry, run, TurnId::DS135);
            this->relax_turn(index, state, entry, run, TurnId::DD90);
        } else {
            this->relax_turn(index, state, entry, run, TurnId::SS90S);
            this->relax_turn(index, state, entry, run, TurnId::SS90L);
            this->relax_turn(index, state, entry, run, TurnId::SS180);

            if (this->run_profile.diagonal) {
                this->relax_turn(index, state, entry, run, TurnId::SD45);
                this->relax_turn(index, state, entry, run, TurnId::SD135);
            }
        }
    }
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::relax_turn(
    uint16_t index, const State& state, const LatticePose& entry, uint8_t run, TurnId turn
) {
    const TurnPrimitive& primitive = get_primitive(turn);

    if (not this->dynamics.get_turn(this->run_profile, turn).valid) {
        return;
    }

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

        const Edge  edge = this->get_edge(index, state.arrival, run, turn);
        const float cost = this->costs.at(index) + edge.cost;

        if (finish_gate >= 0) {
            if (exit.is_diagonal() or not this->is_goal(exit.cell_ahead())) {
                continue;
            }

            const TurnShape& shape = this->dynamics.get_turn(this->run_profile, turn);
            const float      remaining =
                shape.pre + shape.length() - std::max(shape.gates.at(static_cast<uint8_t>(finish_gate)), shape.pre);
            const float speed = this->turn_speeds.at(std::to_underlying(turn)) * static_cast<float>(edge.speed_ratio) /
                                static_cast<float>(full_speed);

            const float stop_distance = this->get_stop_distance(exit);

            this->add_terminal({
                .cost = cost - remaining / speed,
                .state = index,
                .run = run,
                .has_turn = true,
                .turn = turn,
                .side = side,
                .stop_distance = stop_distance,
                .finish_distance = remaining + shape.post + stop_distance,
            });

            continue;
        }

        const uint16_t target = encode({.node = exit, .arrival = to_arrival(turn)});

        if (cost < this->costs.at(target)) {
            const auto link = static_cast<uint16_t>(
                static_cast<uint32_t>(run) | (static_cast<uint32_t>(std::to_underlying(state.arrival)) << 6U) |
                ((side == TurnSide::RIGHT ? 1U : 0U) << 10U)
            );

            this->relax(target, cost, link, edge.speed_ratio);
        }
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
void TPlanner<width, height>::relax(uint16_t index, float cost, uint16_t link, uint8_t speed_ratio) {
    this->costs.at(index) = cost;
    this->links.at(index) = link;
    this->speed_ratios.at(index) = speed_ratio;

    if (this->positions.at(index) == not_queued) {
        this->positions.at(index) = this->queue_size;
        this->queue.at(this->queue_size) = index;
        this->queue_size++;
    }

    this->sift_up(this->positions.at(index));
}

template <uint8_t width, uint8_t height>
uint16_t TPlanner<width, height>::pop() {
    const uint16_t top = this->queue.at(0);

    this->queue_size--;
    this->positions.at(top) = not_queued;

    if (this->queue_size > 0) {
        this->queue.at(0) = this->queue.at(this->queue_size);
        this->positions.at(this->queue.at(0)) = 0;
        this->sift_down(0);
    }

    return top;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::sift_up(uint16_t position) {
    const uint16_t index = this->queue.at(position);

    while (position > 0) {
        const uint16_t parent = (position - 1) / 2;

        if (this->costs.at(this->queue.at(parent)) <= this->costs.at(index)) {
            break;
        }

        this->queue.at(position) = this->queue.at(parent);
        this->positions.at(this->queue.at(position)) = position;
        position = parent;
    }

    this->queue.at(position) = index;
    this->positions.at(index) = position;
}

template <uint8_t width, uint8_t height>
void TPlanner<width, height>::sift_down(uint16_t position) {
    const uint16_t index = this->queue.at(position);

    while (2 * position + 1 < this->queue_size) {
        uint16_t child = 2 * position + 1;

        if (child + 1 < this->queue_size and
            this->costs.at(this->queue.at(child + 1)) < this->costs.at(this->queue.at(child))) {
            child++;
        }

        if (this->costs.at(index) <= this->costs.at(this->queue.at(child))) {
            break;
        }

        this->queue.at(position) = this->queue.at(child);
        this->positions.at(this->queue.at(position)) = position;
        position = child;
    }

    this->queue.at(position) = index;
    this->positions.at(index) = position;
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_PLANNER_TPP
