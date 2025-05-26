/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_CPP
#define MICRAS_NAV_MAZE_CPP

#include <cmath>
#include <stack>
#include <utility>

#include "micras/nav/maze.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TMaze<width, height>::TMaze(Config config) :
    action_queuer(config.action_queuer_config),
    start{config.start},
    goal{config.goal},
    cost_margin{config.cost_margin} {
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
        this->compute_minimum_cost();
        const auto& [next_goal, _] = this->get_next_bfs_goal(pose, true);

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
void TMaze<width, height>::compute_minimum_cost() {
    this->minimum_cost = this->get_next_bfs_goal(this->start, false).second;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::compute_best_route() {
    std::list<GridPoint> current_route;
    std::stack<GridPose> stack;

    float best_route_time = std::numeric_limits<float>::max();
    stack.push(this->start);

    while (not stack.empty()) {
        GridPose prev_pose = stack.top();
        stack.pop();
        current_route.push_back(prev_pose.position);
        GridPoint current_position = prev_pose.front().position;

        bool dead_end = true;

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            if (this->goal.contains(current_position)) {
                current_route.push_back(current_position);
                float current_route_time = get_route_time(current_route);

                if (current_route_time < best_route_time) {
                    best_route_time = current_route_time;
                    best_route = current_route;
                }

                break;
            }

            Side     side = static_cast<Side>(i);
            GridPose next_pose = {current_position, side};

            if (this->costmap.has_wall(next_pose, true) or
                not this->was_visited(this->costmap.get_cell(next_pose.front().position))) {
                continue;
            }

            if (std::find(current_route.begin(), current_route.end(), next_pose.position) == current_route.end()) {
                stack.push(next_pose);
                dead_end = false;
            }
        }

        if (dead_end and not stack.empty()) {
            const GridPoint& intersection_point = stack.top().position;
            auto intersection_it = std::find(current_route.begin(), current_route.end(), intersection_point);
            current_route.erase(intersection_it, current_route.end());
        }
    }
}

template <uint8_t width, uint8_t height>
const std::list<GridPoint>& TMaze<width, height>::get_best_route() const {
    return this->best_route;
}

template <uint8_t width, uint8_t height>
std::vector<uint8_t> TMaze<width, height>::serialize() const {
    std::vector<uint8_t> buffer;
    buffer.reserve(2 * this->best_route.size());

    for (const auto& grid_point : this->best_route) {
        buffer.emplace_back(grid_point.x);
        buffer.emplace_back(grid_point.y);
    }

    return buffer;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::deserialize(const uint8_t* buffer, uint16_t size) {
    this->best_route.clear();

    for (uint32_t i = 0; i < size; i += 2) {
        this->best_route.emplace_back(
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            GridPoint{buffer[i], buffer[i + 1]}
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
std::pair<GridPose, uint16_t> TMaze<width, height>::get_next_bfs_goal(const GridPose& pose, bool discover) const {
    std::queue<GridPose>                            queue;
    std::array<std::array<uint16_t, width>, height> distance{};
    std::pair<GridPose, uint16_t>                   result_pair = {this->start, 0};

    distance.at(pose.position.y).at(pose.position.x) = 1;

    auto sides_order = {
        pose.turned_right().orientation,
        pose.turned_left().orientation,
        pose.orientation,
        pose.turned_back().orientation,
    };

    for (Side side : sides_order) {
        if (not this->costmap.has_wall({pose.position, side})) {
            GridPoint next_position = pose.position + side;
            queue.emplace(next_position, side);
            distance.at(next_position.y).at(next_position.x) = 1;
        }
    }

    while (not queue.empty()) {
        GridPose current_pose = queue.front();
        queue.pop();

        if ((discover and
             this->must_visit(
                 this->costmap.get_cell(current_pose.position), std::round(this->minimum_cost * this->cost_margin)
             )) or
            (not discover and this->goal.contains(current_pose.position))) {
            result_pair = {
                {pose.position + current_pose.orientation, current_pose.orientation},
                distance.at(current_pose.position.y).at(current_pose.position.x)
            };
            break;
        }

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side side = static_cast<Side>(i);

            if (this->costmap.has_wall({current_pose.position, side})) {
                continue;
            }

            const GridPoint front_position = current_pose.position + side;

            if ((discover or this->was_visited(this->costmap.get_cell(front_position))) and
                distance.at(front_position.y).at(front_position.x) == 0) {
                queue.emplace(front_position, current_pose.orientation);
                distance.at(front_position.y).at(front_position.x) =
                    distance.at(current_pose.position.y).at(current_pose.position.x) + 1;
            }
        }
    }

    return result_pair;
}

template <uint8_t width, uint8_t height>
float TMaze<width, height>::get_route_time(const std::list<GridPoint>& route) {
    this->action_queuer.recompute(route);
    return this->action_queuer.get_total_time();
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
