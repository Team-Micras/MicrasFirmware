/**
 * @file
 */

#ifndef MICRAS_NAV_COSTMAP_CPP
#define MICRAS_NAV_COSTMAP_CPP

#include <cmath>
#include <queue>

#include "micras/nav/costmap.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height, uint8_t layers>
Costmap<width, height, layers>::Costmap() {
    for (uint8_t row = 0; row < height; row++) {
        this->cells[row][0].walls[Side::LEFT] = WallState::WALL;
        this->cells[row][width - 1].walls[Side::RIGHT] = WallState::WALL;
    }

    for (uint8_t col = 0; col < width; col++) {
        this->cells[0][col].walls[Side::DOWN] = WallState::WALL;
        this->cells[height - 1][col].walls[Side::UP] = WallState::WALL;
    }
}

template <uint8_t width, uint8_t height, uint8_t layers>
void Costmap<width, height, layers>::compute(const GridPoint& reference, uint8_t layer) {
    std::queue<GridPoint> queue;
    queue.push(reference);

    while (not queue.empty()) {
        GridPoint current_position = queue.front();
        queue.pop();

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side      side = static_cast<Side>(i);
            GridPoint front_position = current_position + side;

            if (not this->has_wall({current_position, side})) {
                int16_t new_cost = this->get_cost(current_position, layer) + 1;

                if (this->get_cost(front_position, layer) > new_cost) {
                    this->update_cost(front_position, layer, new_cost);
                    queue.push(front_position);
                }
            }
        }
    }
}

template <uint8_t width, uint8_t height, uint8_t layers>
void Costmap<width, height, layers>::recompute(const GridPoint& reference, uint8_t layer) {
    std::queue<GridPoint> queue;
    queue.push(reference);

    while (not queue.empty()) {
        GridPoint current_position = queue.front();
        queue.pop();
        int16_t lowest_cost = max_cost;

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side side = static_cast<Side>(i);

            if (not this->has_wall({current_position, side})) {
                lowest_cost = std::min(lowest_cost, this->get_cost(current_position + side, layer));
            }
        }

        if (this->get_cost(current_position, layer) == lowest_cost + 1) {
            continue;
        }

        uint16_t old_cost = this->get_cost(current_position, layer);
        this->update_cost(current_position, layer, lowest_cost + 1);

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side      side = static_cast<Side>(i);
            GridPoint front_position = current_position + side;

            if (not this->has_wall({current_position, side}) and
                this->get_cost(front_position, layer) == old_cost + 1) {
                queue.push(front_position);
            }
        }
    }
}

template <uint8_t width, uint8_t height, uint8_t layers>
const Costmap<width, height, layers>::Cell& Costmap<width, height, layers>::get_cell(const GridPoint& position) const {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height, uint8_t layers>
int16_t Costmap<width, height, layers>::get_cost(const GridPoint& position, uint8_t layer) const {
    return this->get_cell(position).costs[layer];
}

template <uint8_t width, uint8_t height, uint8_t layers>
void Costmap<width, height, layers>::update_cost(const GridPoint& position, uint8_t layer, int16_t cost) {
    this->cell_on_position(position).costs.at(layer) = cost;
}

template <uint8_t width, uint8_t height, uint8_t layers>
bool Costmap<width, height, layers>::has_wall(const GridPose& pose, bool consider_virtual) const {
    return this->get_cell(pose.position).walls[pose.orientation] == WallState::WALL or
           (consider_virtual and this->get_cell(pose.position).walls[pose.orientation] == WallState::VIRTUAL);
}

template <uint8_t width, uint8_t height, uint8_t layers>
bool Costmap<width, height, layers>::update_wall(const GridPose& pose, bool wall) {
    if (this->has_wall(pose, true)) {
        return false;
    }

    const bool updated = wall;
    this->cell_on_position(pose.position).walls[pose.orientation] = wall ? WallState::WALL : WallState::NO_WALL;

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return updated;
    }

    this->cell_on_position(front_pose.position).walls[pose.turned_back().orientation] =
        wall ? WallState::WALL : WallState::NO_WALL;

    return updated;
}

template <uint8_t width, uint8_t height, uint8_t layers>
void Costmap<width, height, layers>::add_virtual_wall(const GridPose& pose) {
    this->cell_on_position(pose.position).walls[pose.orientation] = WallState::VIRTUAL;

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return;
    }

    this->cell_on_position(front_pose.position).walls[pose.turned_back().orientation] = WallState::VIRTUAL;
}

template <uint8_t width, uint8_t height, uint8_t layers>
Costmap<width, height, layers>::Cell& Costmap<width, height, layers>::cell_on_position(const GridPoint& position) {
    return this->cells.at(position.y).at(position.x);
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_COSTMAP_CPP
