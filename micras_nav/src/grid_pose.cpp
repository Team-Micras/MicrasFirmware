/**
 * @file
 */

#include <cmath>

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
Side angle_to_grid(float angle) {
    const int32_t grid_angle = std::lround(2.0F * angle / std::numbers::pi_v<float>);
    return static_cast<Side>(grid_angle < 0 ? (4 + (grid_angle % 4)) % 4 : grid_angle % 4);
}

Side GridPoint::direction(const GridPoint& next) const {
    if (next.x > this->x) {
        return Side::RIGHT;
    }

    if (next.y > this->y) {
        return Side::UP;
    }

    if (next.x < this->x) {
        return Side::LEFT;
    }

    if (next.y < this->y) {
        return Side::DOWN;
    }

    return Side::UP;
}

GridPoint GridPoint::from_vector(const core::Vector& point, float cell_size) {
    return {static_cast<uint8_t>(point.x / cell_size), static_cast<uint8_t>(point.y / cell_size)};
}

core::Vector GridPoint::to_vector(float cell_size) const {
    return {cell_size * (this->x + 0.5F), cell_size * (this->y + 0.5F)};
}

GridPoint GridPoint::operator+(const Side& side) const {
    switch (side) {
        case Side::RIGHT:
            return {static_cast<uint8_t>(this->x + 1), this->y};
        case Side::UP:
            return {this->x, static_cast<uint8_t>(this->y + 1)};
        case Side::LEFT:
            return {static_cast<uint8_t>(this->x - 1), this->y};
        case Side::DOWN:
            return {this->x, static_cast<uint8_t>(this->y - 1)};
    }

    return *this;
}

bool GridPoint::operator==(const GridPoint& other) const {
    return this->x == other.x and this->y == other.y;
}

GridPose GridPose::front() const {
    return {this->position + this->orientation, this->orientation};
}

GridPose GridPose::turned_back() const {
    return {this->position, static_cast<Side>((this->orientation + 2) % 4)};
}

GridPose GridPose::turned_left() const {
    return {this->position, static_cast<Side>((this->orientation + 1) % 4)};
}

GridPose GridPose::turned_right() const {
    return {this->position, static_cast<Side>((this->orientation + 3) % 4)};
}

Side GridPose::get_relative_side(const GridPoint& other) const {
    if (((this->orientation == Side::RIGHT or this->orientation == Side::LEFT) and this->position.y == other.y) or
        ((this->orientation == Side::UP or this->orientation == Side::DOWN) and this->position.x == other.x)) {
        return this->position.direction(other) == this->orientation ? Side::UP : Side::DOWN;
    }

    switch (this->orientation) {
        case Side::RIGHT:
            return this->position.y < other.y ? Side::LEFT : Side::RIGHT;
        case Side::UP:
            return this->position.x > other.x ? Side::LEFT : Side::RIGHT;
        case Side::LEFT:
            return this->position.y > other.y ? Side::LEFT : Side::RIGHT;
        case Side::DOWN:
        default:
            return this->position.x < other.x ? Side::LEFT : Side::RIGHT;
    }
}

bool GridPose::operator==(const GridPose& other) const {
    return this->position == other.position and this->orientation == other.orientation;
}
}  // namespace micras::nav
