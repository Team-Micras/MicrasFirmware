/**
 * @file
 */

#ifndef MICRAS_NAV_COSTMAP_CPP
#define MICRAS_NAV_COSTMAP_CPP

#include <cmath>
#include <list>
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
const Costmap<width, height, layers>::Cell& Costmap<width, height, layers>::get_cell(const GridPoint& position) const {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height, uint8_t layers>
Costmap<width, height, layers>::Cell& Costmap<width, height, layers>::get_cell(const GridPoint& position) {
    return this->cells.at(position.y).at(position.x);
}

template <uint8_t width, uint8_t height, uint8_t layers>
void Costmap<width, height, layers>::update_cost(const GridPoint& position, uint8_t layer, int16_t cost) {
    this->get_cell(position).costs.at(layer) = cost;
}

template <uint8_t width, uint8_t height, uint8_t layers>
bool Costmap<width, height, layers>::update_wall(const GridPose& pose, bool wall) {
    if (pose.position.x >= width or pose.position.y >= height or this->has_wall(pose)) {
        return false;
    }

    const bool updated = wall;
    this->get_cell(pose.position).walls[pose.orientation] = wall ? WallState::WALL : WallState::NO_WALL;

    GridPose front_pose = pose.front();

    if (front_pose.position.x >= width or front_pose.position.y >= height) {
        return updated;
    }

    this->get_cell(front_pose.position).walls[pose.turned_back().orientation] =
        wall ? WallState::WALL : WallState::NO_WALL;

    return updated;
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
    std::list<GridPoint>  compute_list;
    queue.push(reference);

    while (not queue.empty()) {
        GridPoint current_position = queue.front();
        queue.pop();
        bool reset_cost = true;

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side      side = static_cast<Side>(i);
            GridPoint front_position = current_position + side;

            if (not this->has_wall({current_position, side}) and
                this->get_cost(current_position, layer) == this->get_cost(front_position, layer) + 1) {
                compute_list.emplace_back(current_position);
                reset_cost = false;
                break;
            }
        }

        if (not reset_cost) {
            continue;
        }

        uint16_t old_cost = this->get_cost(current_position, layer);
        this->update_cost(current_position, layer, max_cost);

        for (uint8_t i = Side::RIGHT; i <= Side::DOWN; i++) {
            Side      side = static_cast<Side>(i);
            GridPoint front_position = current_position + side;

            if (not this->has_wall({current_position, side}) and
                this->get_cost(front_position, layer) == old_cost + 1) {
                queue.push(front_position);
            }
        }
    }

    for (const auto& position : compute_list) {
        this->compute(position, layer);
    }
}

template <uint8_t width, uint8_t height, uint8_t layers>
bool Costmap<width, height, layers>::has_wall(const GridPose& pose) const {
    return this->get_cell(pose.position).walls[pose.orientation] == WallState::WALL;
}

template <uint8_t width, uint8_t height, uint8_t layers>
int16_t Costmap<width, height, layers>::get_cost(const GridPoint& position, uint8_t layer) const {
    return this->get_cell(position).costs[layer];
}

template <uint8_t width, uint8_t height, uint8_t layers>
void Costmap<width, height, layers>::add_virtual_wall(const GridPose& pose) {
    this->get_cell(pose.position).walls[pose.orientation] = WallState::VIRTUAL;
    this->get_cell(pose.position).walls[pose.turned_back().orientation] = WallState::VIRTUAL;
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_COSTMAP_CPP
