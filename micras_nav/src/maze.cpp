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
    if constexpr (width == 16 and height == 16) {
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

    this->get_cell(this->start.position).visited = true;
    this->compute_costmap(*(this->goal.begin()));
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::update_walls(const GridPose& pose, const core::Observation& observation) {
    this->get_cell(pose.position).visited = true;

    this->update_wall(pose.turned_left(), observation.left);
    this->update_wall(pose, observation.front);
    this->update_wall(pose.turned_right(), observation.right);

    for (auto& row : this->cells) {
        for (auto& cell : row) {
            cell.cost = 0x7FFF;
        }
    }

    if (this->returning) {
        this->get_cell(this->start.position).cost = 0;
    } else {
        for (const auto& position : this->goal) {
            this->get_cell(position).cost = 0;
        }
    }

    this->compute_costmap(this->returning ? this->start.position : *(this->goal.begin()));
}

template <uint8_t width, uint8_t height>
GridPose TMaze<width, height>::get_next_goal(const GridPoint& position) const {
    uint16_t current_cost = this->get_cell(position).cost;
    GridPose next_pose = {position, {}};

    for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
        Side      side = static_cast<Side>(i);
        GridPoint front_position = position + side;

        if (not this->has_wall({position, side}) and this->get_cell(front_position).cost < current_cost) {
            current_cost = this->get_cell(front_position).cost;
            next_pose.position = front_position;
            next_pose.orientation = side;
        }
    }

    return next_pose;
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
bool TMaze<width, height>::update_wall(const GridPose& pose, bool wall) {
    if (pose.position.x >= width or pose.position.y >= height) {
        return false;
    }

    bool updated = wall and not this->get_cell(pose.position).walls[pose.orientation];
    this->get_cell(pose.position).walls[pose.orientation] |= wall;

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return updated;
    }

    this->get_cell(front_pose.position).walls[pose.turned_back().orientation] |= wall;

    return updated;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::has_wall(const GridPose& pose) const {
    return this->get_cell(pose.position).walls[pose.orientation];
}

template <uint8_t width, uint8_t height>
core::Observation TMaze<width, height>::get_observation(const GridPose& pose) const {
    return {
        .left = this->has_wall(pose.turned_left()),
        .front = this->has_wall(pose),
        .right = this->has_wall(pose.turned_right())
    };
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::finished(const GridPoint& position) const {
    return this->returning ? this->start.position == position : this->goal.contains(position);
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::compute_return_costmap() {
    for (auto& row : this->cells) {
        for (auto& cell : row) {
            cell.cost = 0x7FFF;
        }
    }

    this->returning = true;

    this->get_cell(this->start.position).cost = 0;
    this->compute_costmap(this->start.position);
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
void TMaze<width, height>::compute_costmap(const GridPoint& reference) {
    std::queue<GridPoint> queue;
    queue.push(reference);

    while (not queue.empty()) {
        GridPoint current_position = queue.front();
        queue.pop();

        const Cell& current_cell = this->get_cell(current_position);

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side      side = static_cast<Side>(i);
            GridPoint front_position = current_position + side;

            if (not this->has_wall({current_position, side})) {
                uint16_t new_cost =
                    current_cell.cost + 1 -
                    (this->returning and current_cell.visited and not this->get_cell(front_position).visited ? 10 : 0);

                if (this->get_cell(front_position).cost > new_cost) {
                    this->get_cell(front_position).cost = new_cost;
                    queue.push(front_position);
                }
            }
        }
    }
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::update_cost(const GridPoint& reference) {
    for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
        Side      side = static_cast<Side>(i);
        GridPoint front_position = reference + side;

        if (not this->has_wall({reference, side}) and
            this->get_cell(reference).cost == this->get_cell(front_position).cost + 1) {
            this->compute_costmap(reference);
            return;
        }
    }

    uint16_t old_cost = this->get_cell(reference).cost;
    this->get_cell(reference).cost = 0x7FFF;

    for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
        Side      side = static_cast<Side>(i);
        GridPoint front_position = reference + side;

        if (not this->has_wall({reference, side}) and this->get_cell(front_position).cost == old_cost + 1) {
            this->update_cost(front_position);
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
