/**
 * @file state.cpp
 *
 * @brief Types for robot state representation
 *
 * @date 10/2024
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

Point Point::operator%(float value) const {
    return {std::fmod(this->x, value), std::fmod(this->y, value)};
}

Point Point::from_grid(const GridPoint& grid_point, float cell_size) {
    return {cell_size * (grid_point.x + 0.5F), cell_size * (grid_point.y + 0.5F)};
}

GridPose Pose::to_grid(float cell_size) const {
    return {this->position.to_grid(cell_size), angle_to_grid(this->orientation)};
};
}  // namespace micras::nav
