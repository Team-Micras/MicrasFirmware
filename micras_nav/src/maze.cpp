/**
 * @file maze.cpp
 *
 * @brief Nav Maze class implementation
 *
 * @date 10/2024
 */

#ifndef MICRAS_NAV_MAZE_CPP
#define MICRAS_NAV_MAZE_CPP

#include <iterator>
#include <queue>

#include "micras/nav/maze.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
Maze<width, height>::Maze(const GridPose& start, const std::unordered_set<GridPoint>& goal) : start{start}, goal{goal} {
    for (uint8_t row = 0; row < height; row++) {
        this->cells[row][0].wall_count[Side::LEFT] = 0xFFFF;
        this->cells[row][width - 1].wall_count[Side::RIGHT] = 0xFFFF;
    }

    for (uint8_t col = 0; col < width; col++) {
        this->cells[0][col].wall_count[Side::DOWN] = 0xFFFF;
        this->cells[height - 1][col].wall_count[Side::UP] = 0xFFFF;
    }

    this->cells[start.position.y][start.position.x].wall_count[start.turned_right().orientation] = 0xFFFF;
    GridPose right_pose = start.turned_right().front();
    this->get_cell(right_pose.position).wall_count[right_pose.turned_back().orientation] = 0xFFFF;

    for (const auto& position : this->goal) {
        this->get_cell(position).cost = 0;
    }

    this->calculate_costmap();
}

template <uint8_t width, uint8_t height>
void Maze<width, height>::update(const GridPose& pose, Information information) {
    if (this->goal.contains(pose.position)) {
        this->returning = true;
    } else if (pose == start.turned_back()) {
        this->exploring = false;
        this->returning = false;
    }

    if (not this->exploring) {
        return;
    }

    if (information.left != core::Observation::UNKNOWN) {
        this->update_wall(pose.turned_left(), information.left == core::Observation::WALL);
    }

    if (information.front_left != core::Observation::UNKNOWN) {
        this->update_wall(pose.front().turned_left(), information.front_left == core::Observation::WALL);
    }

    if (information.front != core::Observation::UNKNOWN) {
        this->update_wall(pose, information.front == core::Observation::WALL);
    }

    if (information.front_right != core::Observation::UNKNOWN) {
        this->update_wall(pose.front().turned_right(), information.front_right == core::Observation::WALL);
    }

    if (information.right != core::Observation::UNKNOWN) {
        this->update_wall(pose.turned_right(), information.right == core::Observation::WALL);
    }

    this->calculate_costmap();
}

template <uint8_t width, uint8_t height>
GridPose Maze<width, height>::get_current_goal(const GridPoint& position, bool force_costmap) const {
    uint16_t current_cost = this->get_cell(position).cost;

    if (not force_costmap and (not this->exploring or this->returning) and this->best_route.contains(current_cost) and
        this->best_route.at(current_cost).position == position) {
        return (this->returning ? std::prev(this->best_route.find(current_cost)) :
                                  std::next(this->best_route.find(current_cost)))
            ->second;
    }

    GridPoint next_position = position;
    Side      next_side{};

    for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
        Side      side = static_cast<Side>(i);
        GridPoint front_position = position + side;

        if (not this->has_wall({position, side}) and this->get_cell(front_position).cost <= current_cost) {
            current_cost = this->get_cell(front_position).cost;
            next_position = front_position;
            next_side = side;
        }
    }

    return {next_position, next_side};
}

template <uint8_t width, uint8_t height>
const Maze<width, height>::Cell& Maze<width, height>::get_cell(const GridPoint& position) const {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height>
Maze<width, height>::Cell& Maze<width, height>::get_cell(const GridPoint& position) {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height>
void Maze<width, height>::update_wall(const GridPose& pose, bool wall) {
    if (pose.position.x >= width or pose.position.y >= height) {
        return;
    }

    if (wall) {
        this->get_cell(pose.position).wall_count[pose.orientation]++;
    } else {
        this->get_cell(pose.position).free_count[pose.orientation]++;
    }

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return;
    }

    if (wall) {
        this->get_cell(front_pose.position).wall_count[pose.turned_back().orientation]++;
    } else {
        this->get_cell(front_pose.position).free_count[pose.turned_back().orientation]++;
    }
}

template <uint8_t width, uint8_t height>
bool Maze<width, height>::has_wall(const GridPose& pose) const {
    return this->get_cell(pose.position).wall_count[pose.orientation] >
           this->get_cell(pose.position).free_count[pose.orientation];
}

template <uint8_t width, uint8_t height>
void Maze<width, height>::calculate_costmap() {
    std::array<std::array<bool, width>, height> visited{};
    std::array<std::array<Side, width>, height> origin{};
    std::queue<GridPoint>                       queue;

    for (const auto& position : this->goal) {
        queue.push(position);
        visited.at(position.y).at(position.x) = true;
    }

    while (not queue.empty()) {
        GridPoint current_position = queue.front();
        queue.pop();

        const Cell& current_cell = this->get_cell(current_position);

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side      side = static_cast<Side>(i);
            GridPoint front_position = current_position + side;

            if (not this->has_wall({current_position, side}) and
                not visited.at(front_position.y).at(front_position.x)) {
                visited.at(front_position.y).at(front_position.x) = true;
                origin.at(front_position.y).at(front_position.x) = side;

                this->get_cell(front_position).cost =
                    (current_cell.cost + (this->goal.contains(current_position) or
                                                  (side == origin.at(current_position.y).at(current_position.x)) ?
                                              1 :
                                              2));
                queue.push(front_position);
            }
        }
    }

    if (not this->returning) {
        return;
    }

    GridPose current_pose = this->start;
    this->best_route.clear();
    this->best_route.try_emplace(this->get_cell(this->start.position).cost, this->start);

    while (not this->goal.contains(current_pose.position)) {
        current_pose = this->get_current_goal(current_pose.position, true);
        this->best_route.try_emplace(this->get_cell(current_pose.position).cost, current_pose);
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_CPP
