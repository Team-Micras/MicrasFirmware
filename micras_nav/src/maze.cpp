/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_CPP
#define MICRAS_NAV_MAZE_CPP

#include <cmath>
#include <utility>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TMaze<width, height>::TMaze(Config config, EdgeCostFunction edge_cost_function) :
    graph{std::move(edge_cost_function)}, start{config.start}, goal{config.goal}, cost_margin{config.cost_margin} {
    this->costmap.update_wall(this->start, false);
    this->costmap.update_wall(start.turned_right(), true);

    // Hardcoded walls at the end region
    if constexpr (width == 16 and height == 16) {
        this->costmap.update_wall({{7, 7}, Side::UP}, false);
        this->costmap.update_wall({{8, 7}, Side::UP}, false);
        this->costmap.update_wall({{8, 7}, Side::LEFT}, false);
        this->costmap.update_wall({{8, 8}, Side::LEFT}, false);

        // this->update_wall({{7, 7}, Side::DOWN}, true);
        // this->update_wall({{8, 7}, Side::DOWN}, true);
        // this->update_wall({{8, 7}, Side::RIGHT}, true);
        // this->update_wall({{8, 8}, Side::RIGHT}, true);
        // this->update_wall({{8, 8}, Side::UP}, true);
        // this->update_wall({{7, 8}, Side::UP}, true);
        // this->update_wall({{7, 8}, Side::LEFT}, true);
        // this->update_wall({{7, 7}, Side::LEFT}, true);
    }

    for (const auto& position : this->goal) {
        this->costmap.update_cost(position, Layer::EXPLORE, 0);
    }

    for (const auto& position : this->goal) {
        this->costmap.compute(position, Layer::EXPLORE);
    }

    this->costmap.update_cost(this->start.position, Layer::RETURN, 0);
    this->costmap.compute(this->start.position, Layer::RETURN);
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::update_walls(const GridPose& pose, const core::Observation& observation) {
    const bool new_left_wall = this->costmap.update_wall(pose.turned_left(), observation.left);
    const bool new_front_wall = this->costmap.update_wall(pose, observation.front);
    const bool new_right_wall = this->costmap.update_wall(pose.turned_right(), observation.right);

    if (new_left_wall) {
        this->update_cell(pose.turned_left().front().position);
    }

    if (new_front_wall) {
        this->update_cell(pose.front().position);
    }

    if (new_right_wall) {
        this->update_cell(pose.turned_right().front().position);
    }

    if (new_left_wall or new_front_wall or new_right_wall) {
        this->update_cell(pose.position);
    }
}

template <uint8_t width, uint8_t height>
GridPose TMaze<width, height>::get_next_goal(const GridPose& pose, bool returning) {
    if (returning and not this->finished_discovery) {
        this->compute_best_route();
        const auto& next_goal = this->get_next_bfs_goal(pose, true);

        if (next_goal != this->start) {
            return next_goal;
        }

        this->finished_discovery = true;
    }

    int16_t  current_cost = max_cost;
    GridPose next_pose = {};

    auto sides_order = {
        pose.turned_right().orientation,
        pose.turned_left().orientation,
        pose.orientation,
        pose.turned_back().orientation,
    };

    for (Side side : sides_order) {
        if (this->costmap.has_wall({pose.position, side})) {
            continue;
        }

        GridPoint     front_position = pose.position + side;
        const int16_t flip_cost = pose.turned_back().orientation == side ? 1 : 0;
        const int16_t front_cost =
            this->costmap.get_cost(front_position, returning ? Layer::RETURN : Layer::EXPLORE) + flip_cost;

        if (front_cost < current_cost) {
            current_cost = front_cost;
            next_pose.position = front_position;
            next_pose.orientation = side;
        }
    }

    return next_pose;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::finished(const GridPoint& position, bool returning) const {
    return returning ? this->start.position == position : this->goal.contains(position);
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::compute_best_route() {
    GridPose current_pose = this->start;
    this->best_route.clear();
    this->best_route.emplace_back(this->start);

    while (not this->goal.contains(current_pose.position)) {
        current_pose = this->get_next_bfs_goal(current_pose, false);
        this->best_route.emplace_back(current_pose);
    }

    this->minimum_cost = this->best_route.size();
}

template <uint8_t width, uint8_t height>
const std::list<GridPose>& TMaze<width, height>::get_best_route() const {
    return this->best_route;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::compute_graph() {
    std::queue<GridPose> queue;
    this->graph.add_node(this->start);
    queue.push(this->start);

    while (not queue.empty()) {
        GridPose current_pose = queue.front();
        queue.pop();

        if (this->goal.contains(current_pose.position)) {
            continue;
        }

        auto sides_order = {
            current_pose.turned_right().orientation,
            current_pose.turned_left().orientation,
            current_pose.orientation,
        };

        for (Side side : sides_order) {
            if (this->costmap.has_wall({current_pose.position, side}, true)) {
                continue;
            }

            GridPose next_pose = {current_pose.position + side, side};

            if (not this->graph.has_node(next_pose)) {
                this->graph.add_node(next_pose);
                queue.push(next_pose);
            }

            this->graph.add_edge(current_pose, next_pose);
        }
    }
}

template <uint8_t width, uint8_t height>
std::vector<uint8_t> TMaze<width, height>::serialize() const {
    std::vector<uint8_t> buffer;
    buffer.reserve(3 * this->best_route.size());

    for (const auto& grid_pose : this->best_route) {
        buffer.emplace_back(grid_pose.position.x);
        buffer.emplace_back(grid_pose.position.y);
        buffer.emplace_back(grid_pose.orientation);
    }

    return buffer;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::deserialize(const uint8_t* buffer, uint16_t size) {
    this->best_route.clear();

    for (uint32_t i = 0; i < size; i += 3) {
        this->best_route.emplace_back(
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            GridPoint{buffer[i], buffer[i + 1]}, static_cast<Side>(buffer[i + 2])
        );
    }
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::update_cell(const GridPoint& position) {
    if (not this->goal.contains(position)) {
        this->costmap.recompute(position, Layer::EXPLORE);
    }

    if (position != this->start.position) {
        this->costmap.recompute(position, Layer::RETURN);
    }

    GridPoint dead_end_position = position;

    while (this->is_dead_end(this->costmap.get_cell(dead_end_position))) {
        for (Side side : {Side::UP, Side::DOWN, Side::LEFT, Side::RIGHT}) {
            if (not this->costmap.has_wall({dead_end_position, side}, true)) {
                this->costmap.add_virtual_wall({dead_end_position, side});
                dead_end_position = dead_end_position + side;
                break;
            }
        }
    }
}

template <uint8_t width, uint8_t height>
GridPose TMaze<width, height>::get_next_bfs_goal(const GridPose& pose, bool discover) const {
    std::queue<GridPose>                        queue;
    std::array<std::array<bool, width>, height> checked{};
    GridPose                                    next_goal = this->start;

    checked.at(pose.position.y).at(pose.position.x) = true;

    auto sides_order = {
        pose.turned_right().orientation,
        pose.turned_left().orientation,
        pose.orientation,
        pose.turned_back().orientation,
    };

    for (Side side : sides_order) {
        if (not this->costmap.has_wall({pose.position, side})) {
            queue.emplace(pose.position + side, side);
        }
    }

    while (not queue.empty()) {
        GridPose current_pose = queue.front();
        queue.pop();
        checked.at(current_pose.position.y).at(current_pose.position.x) = true;

        if ((discover and
             this->must_visit(
                 this->costmap.get_cell(current_pose.position), std::round(this->minimum_cost * this->cost_margin)
             )) or
            (not discover and this->goal.contains(current_pose.position))) {
            next_goal = {pose.position + current_pose.orientation, current_pose.orientation};
            break;
        }

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side side = static_cast<Side>(i);

            if (this->costmap.has_wall({current_pose.position, side})) {
                continue;
            }

            const GridPoint front_position = current_pose.position + side;

            if ((discover or this->was_visited(this->costmap.get_cell(front_position))) and
                not checked.at(front_position.y).at(front_position.x)) {
                queue.emplace(front_position, current_pose.orientation);
            }
        }
    }

    return next_goal;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::is_dead_end(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell) {
    uint8_t wall_count = 0;

    for (const auto& wall : cell.walls) {
        if (wall == Costmap<width, height, Layer::NUM_OF_LAYERS>::WallState::WALL or
            wall == Costmap<width, height, Layer::NUM_OF_LAYERS>::WallState::VIRTUAL) {
            wall_count++;
        }
    }

    return (wall_count == 3);
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::was_visited(const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell) {
    return not(
        cell.walls[Side::UP] == Costmap<width, height, Layer::NUM_OF_LAYERS>::WallState::UNKNOWN or
        cell.walls[Side::DOWN] == Costmap<width, height, Layer::NUM_OF_LAYERS>::WallState::UNKNOWN or
        cell.walls[Side::LEFT] == Costmap<width, height, Layer::NUM_OF_LAYERS>::WallState::UNKNOWN or
        cell.walls[Side::RIGHT] == Costmap<width, height, Layer::NUM_OF_LAYERS>::WallState::UNKNOWN
    );
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::must_visit(
    const Costmap<width, height, Layer::NUM_OF_LAYERS>::Cell& cell, int16_t cost_threshold
) {
    return not was_visited(cell) and (cell.costs[Layer::EXPLORE] + cell.costs[Layer::RETURN] <= cost_threshold);
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_CPP
