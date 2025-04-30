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
TMaze<width, height>::TMaze(Config config) : start{config.start}, goal{config.goal} {
    for (uint8_t row = 0; row < height; row++) {
        this->cells[row][0].walls[Side::LEFT] = true;
        this->cells[row][width - 1].walls[Side::RIGHT] = true;
    }

    for (uint8_t col = 0; col < width; col++) {
        this->cells[0][col].walls[Side::DOWN] = true;
        this->cells[height - 1][col].walls[Side::UP] = true;
    }

    this->update_wall(start.turned_right(), true);

    // Hardcoded walls at the end region
    if (width == 16 and height == 16) {
        this->update_wall({{7, 7}, Side::DOWN}, true);
        this->update_wall({{8, 7}, Side::DOWN}, true);
        // this->update_wall({{8, 7}, Side::RIGHT}, true);
        this->update_wall({{8, 8}, Side::RIGHT}, true);
        this->update_wall({{8, 8}, Side::UP}, true);
        this->update_wall({{7, 8}, Side::UP}, true);
        this->update_wall({{7, 8}, Side::LEFT}, true);
        this->update_wall({{7, 7}, Side::LEFT}, true);
    }

    for (const auto& position : this->goal) {
        this->get_cell(position).cost = 0;
    }

    this->compute_costmap();
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::update_walls(const GridPose& pose, const core::Observation& observation) {
    this->update_wall(pose.turned_left(), observation.left);
    this->update_wall(pose, observation.front);
    this->update_wall(pose.turned_right(), observation.right);

    this->compute_costmap();
}

template <uint8_t width, uint8_t height>
GridPose TMaze<width, height>::get_next_goal(const GridPoint& position, bool returning) const {
    uint16_t current_cost = this->get_cell(position).cost;

    if (returning) {
        if (this->best_route.contains(current_cost) and this->best_route.at(current_cost).position == position) {
            auto current = this->best_route.find(current_cost);
            return {std::prev(current)->second.position, current->second.turned_back().orientation};
        }

        return get_next_goal(position, false);
    }

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
const TMaze<width, height>::Cell& TMaze<width, height>::get_cell(const GridPoint& position) const {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height>
TMaze<width, height>::Cell& TMaze<width, height>::get_cell(const GridPoint& position) {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::update_wall(const GridPose& pose, bool wall) {
    if (pose.position.x >= width or pose.position.y >= height) {
        return;
    }

    this->get_cell(pose.position).walls[pose.orientation] |= wall;

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return;
    }

    this->get_cell(front_pose.position).walls[pose.turned_back().orientation] |= wall;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::has_wall(const GridPose& pose) const {
    return this->get_cell(pose.position).walls[pose.orientation];
}

template <uint8_t width, uint8_t height>
core::Observation TMaze<width, height>::get_observation(const GridPose& pose) const {
    return {this->has_wall(pose.turned_left()), this->has_wall(pose), this->has_wall(pose.turned_right())};
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::finished(const GridPoint& position, bool returning) const {
    return returning ? this->start.position == position : this->goal.contains(position);
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::compute_best_route() {
    GridPose current_pose = this->start;
    this->best_route.clear();
    this->best_route.try_emplace(this->get_cell(this->start.position).cost, this->start);

    while (not this->goal.contains(current_pose.position)) {
        current_pose = this->get_next_goal(current_pose.position, false);
        this->best_route.try_emplace(this->get_cell(current_pose.position).cost, current_pose);
    }
}

template <uint8_t width, uint8_t height>
const std::map<uint16_t, GridPose, std::greater<>>& TMaze<width, height>::get_best_route() const {
    return this->best_route;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::compute_costmap() {
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

template <uint8_t width, uint8_t height>
std::vector<uint8_t> TMaze<width, height>::serialize() const {
    std::vector<uint8_t> buffer;
    buffer.reserve(3 * this->best_route.size());

    for (const auto& [cost, grid_pose] : this->best_route) {
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
        this->best_route.try_emplace(
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            (size - i) / 3 - 1, GridPose{{buffer[i], buffer[i + 1]}, static_cast<Side>(buffer[i + 2])}
        );
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_CPP
