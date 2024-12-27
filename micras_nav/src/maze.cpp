/**
 * @file
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

    // Hardcoded walls at the end region
    if (width == 16 and height == 16) {
        this->cells.at(7).at(7).wall_count[Side::DOWN] = 0xFFFF;
        this->cells.at(7).at(7).wall_count[Side::LEFT] = 0xFFFF;
        this->cells.at(6).at(7).wall_count[Side::UP] = 0xFFFF;
        this->cells.at(7).at(6).wall_count[Side::RIGHT] = 0xFFFF;
        this->cells.at(7).at(8).wall_count[Side::DOWN] = 0xFFFF;
        this->cells.at(6).at(8).wall_count[Side::UP] = 0xFFFF;
        this->cells.at(7).at(8).free_count[Side::RIGHT] = 0xFFFF;
        this->cells.at(7).at(9).free_count[Side::LEFT] = 0xFFFF;
        this->cells.at(8).at(7).wall_count[Side::LEFT] = 0xFFFF;
        this->cells.at(8).at(7).wall_count[Side::UP] = 0xFFFF;
        this->cells.at(8).at(6).wall_count[Side::RIGHT] = 0xFFFF;
        this->cells.at(9).at(7).wall_count[Side::DOWN] = 0xFFFF;
        this->cells.at(8).at(8).wall_count[Side::UP] = 0xFFFF;
        this->cells.at(8).at(8).wall_count[Side::RIGHT] = 0xFFFF;
        this->cells.at(9).at(8).wall_count[Side::DOWN] = 0xFFFF;
        this->cells.at(8).at(9).wall_count[Side::LEFT] = 0xFFFF;
    }

    for (const auto& position : this->goal) {
        this->get_cell(position).cost = 0;
    }

    this->calculate_costmap();
}

template <uint8_t width, uint8_t height>
void Maze<width, height>::update(const GridPose& pose, Information information) {
    bool new_information{};

    if (information.left != core::Observation::UNKNOWN) {
        new_information |= this->update_wall(pose.turned_left(), information.left == core::Observation::WALL);
    }

    if (information.front_left != core::Observation::UNKNOWN) {
        new_information |=
            this->update_wall(pose.front().turned_left(), information.front_left == core::Observation::WALL);
    }

    if (information.front != core::Observation::UNKNOWN) {
        new_information |= this->update_wall(pose, information.front == core::Observation::WALL);
    }

    if (information.front_right != core::Observation::UNKNOWN) {
        new_information |=
            this->update_wall(pose.front().turned_right(), information.front_right == core::Observation::WALL);
    }

    if (information.right != core::Observation::UNKNOWN) {
        new_information |= this->update_wall(pose.turned_right(), information.right == core::Observation::WALL);
    }

    if (new_information) {
        this->calculate_costmap();
    }
}

template <uint8_t width, uint8_t height>
GridPose Maze<width, height>::get_current_exploration_goal(const GridPoint& position) const {
    uint16_t  current_cost = this->get_cell(position).cost;
    GridPoint next_position = position;
    Side      next_side{};

    for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
        Side      side = static_cast<Side>(i);
        GridPoint front_position = position + side;

        if (not this->has_wall({position, side}) and this->get_cell(front_position).cost < current_cost) {
            current_cost = this->get_cell(front_position).cost;
            next_position = front_position;
            next_side = side;
        }
    }

    return {next_position, next_side};
}

template <uint8_t width, uint8_t height>
GridPose Maze<width, height>::get_current_returning_goal(const GridPoint& position) const {
    uint16_t current_cost = this->get_cell(position).cost;

    if (this->best_route.contains(current_cost) and this->best_route.at(current_cost).position == position) {
        auto current = this->best_route.find(current_cost);
        return {std::prev(current)->second.position, current->second.turned_back().orientation};
    }

    return get_current_exploration_goal(position);
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
bool Maze<width, height>::update_wall(const GridPose& pose, bool wall) {
    if (pose.position.x >= width or pose.position.y >= height) {
        return false;
    }

    bool new_information = this->has_wall(pose);

    if (wall) {
        this->get_cell(pose.position).wall_count[pose.orientation]++;
    } else {
        this->get_cell(pose.position).free_count[pose.orientation]++;
    }

    new_information ^= this->has_wall(pose);

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return new_information;
    }

    bool new_front_information = this->has_wall(front_pose);

    if (wall) {
        this->get_cell(front_pose.position).wall_count[pose.turned_back().orientation]++;
    } else {
        this->get_cell(front_pose.position).free_count[pose.turned_back().orientation]++;
    }

    new_front_information ^= this->has_wall(front_pose);

    return new_information or new_front_information;
}

template <uint8_t width, uint8_t height>
bool Maze<width, height>::has_wall(const GridPose& pose) const {
    return this->get_cell(pose.position).wall_count[pose.orientation] >
           this->get_cell(pose.position).free_count[pose.orientation];
}

template <uint8_t width, uint8_t height>
core::FollowWallType Maze<width, height>::get_follow_wall_type(const GridPose& pose) const {
    if (pose.position.x >= width or pose.position.y >= height) {
        return core::FollowWallType::NONE;
    }

    if (this->has_wall(pose)) {
        return core::FollowWallType::FRONT;
    }

    if (this->has_wall(pose.turned_left()) and this->has_wall(pose.turned_right())) {
        return core::FollowWallType::PARALLEL;
    }

    if (this->has_wall(pose.turned_left())) {
        return core::FollowWallType::LEFT;
    }

    if (this->has_wall(pose.turned_right())) {
        return core::FollowWallType::RIGHT;
    }

    return core::FollowWallType::NONE;
}

template <uint8_t width, uint8_t height>
bool Maze<width, height>::finished(const GridPoint& position) const {
    return this->goal.contains(position);
}

template <uint8_t width, uint8_t height>
bool Maze<width, height>::returned(const GridPoint& position) const {
    return this->start.position == position;
}

template <uint8_t width, uint8_t height>
void Maze<width, height>::calculate_best_route() {
    GridPose current_pose = this->start;
    this->best_route.clear();
    this->best_route.try_emplace(this->get_cell(this->start.position).cost, this->start);

    while (not this->goal.contains(current_pose.position)) {
        current_pose = this->get_current_exploration_goal(current_pose.position);
        this->best_route.try_emplace(this->get_cell(current_pose.position).cost, current_pose);
    }
}

template <uint8_t width, uint8_t height>
void Maze<width, height>::optimize_route() {
    for (auto it = std::next(this->best_route.begin()); it != this->best_route.end();) {
        auto next = std::next(it);

        if (next != this->best_route.end()) {
            if (it->second.orientation == next->second.orientation) {
                it = this->best_route.erase(it);
                continue;
            }
        }

        it++;
    }
}

template <uint8_t width, uint8_t height>
const std::map<uint16_t, GridPose, std::greater<>>& Maze<width, height>::get_best_route() const {
    return this->best_route;
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
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_CPP
