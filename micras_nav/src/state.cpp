/**
 * @file
 */

#include <cmath>
#include <numbers>

#include "micras/nav/state.hpp"

namespace micras::nav {
float Point::distance(const Point& other) const {
    return std::hypot(other.x - this->x, other.y - this->y);
}

float Point::angle_between(const Point& other) const {
    return std::atan2(other.y - this->y, other.x - this->x);
}

GridPoint Point::to_grid(float cell_size) const {
    return {static_cast<uint8_t>(this->x / cell_size), static_cast<uint8_t>(this->y / cell_size)};
}

Point Point::rotate(Side angle) {
    switch (angle) {
        case Side::RIGHT:
            return {-this->y, this->x};
        case Side::UP:
            return *this;
        case Side::LEFT:
            return {this->y, -this->x};
        case Side::DOWN:
            return {-this->x, -this->y};
    }

    return *this;
}

Point Point::operator-(const Point& other) const {
    return {this->x - other.x, this->y - other.y};
}

bool Point::operator==(const Point& other) const {
    return this->x == other.x and this->y == other.y;
}

Point Point::operator%(float value) const {
    return {std::fmod(this->x, value), std::fmod(this->y, value)};
}

Point Point::from_grid(const GridPoint& grid_point, float cell_size) {
    return {cell_size * (grid_point.x + 0.5F), cell_size * (grid_point.y + 0.5F)};
}

GridPose Pose::to_grid(float cell_size) const {
    return {this->position.to_grid(cell_size), angle_to_grid(this->orientation)};
};

Point Pose::to_cell(float cell_size) const {
    Point cell_position = this->position % cell_size;

    float cell_advance = 0.0F;
    float cell_alignment = 0.0F;

    switch (angle_to_grid(this->orientation)) {
        case Side::RIGHT:
            cell_advance = cell_position.x;
            cell_alignment = cell_size / 2.0F - cell_position.y;
            break;
        case Side::UP:
            cell_advance = cell_position.y;
            cell_alignment = cell_position.x - cell_size / 2.0F;
            break;
        case Side::LEFT:
            cell_advance = cell_size - cell_position.x;
            cell_alignment = cell_position.y - cell_size / 2.0F;
            break;
        case Side::DOWN:
            cell_advance = cell_size - cell_position.y;
            cell_alignment = cell_size / 2.0F - cell_position.x;
            break;
    }

    return {cell_alignment, cell_advance};
}
}  // namespace micras::nav
