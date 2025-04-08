/**
 * @file
 */

#include <cmath>

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
Side angle_to_grid(float angle) {
    const int32_t grid_angle = std::lround(2.0F * angle / std::numbers::pi_v<float>);
    return static_cast<Side>(grid_angle < 0 ? 4 + (grid_angle % 4) : grid_angle % 4);
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

bool GridPose::operator==(const GridPose& other) const {
    return this->position == other.position and this->orientation == other.orientation;
}
}  // namespace micras::nav
