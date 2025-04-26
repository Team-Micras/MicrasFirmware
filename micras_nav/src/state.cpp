/**
 * @file
 */

#include <cmath>
#include <numbers>

#include "micras/core/utils.hpp"
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

Point Point::rotate(Direction angle) {
    constexpr float sqrt2_2 = std::numbers::sqrt2_v<float> / 2.0F;

    switch (angle) {
        case Direction::EAST:
            return {-this->y, this->x};
        case Direction::NORTHEAST:
            return {sqrt2_2 * (this->x - this->y), sqrt2_2 * (this->x + this->y)};
        case Direction::NORTH:
            return *this;
        case Direction::NORTHWEST:
            return {sqrt2_2 * (this->x + this->y), sqrt2_2 * (-this->x + this->y)};
        case Direction::WEST:
            return {this->y, -this->x};
        case Direction::SOUTHWEST:
            return {sqrt2_2 * (-this->x + this->y), sqrt2_2 * (-this->x - this->y)};
        case Direction::SOUTH:
            return {-this->x, -this->y};
        case Direction::SOUTHEAST:
            return {sqrt2_2 * (-this->x - this->y), sqrt2_2 * (this->x - this->y)};
    }

    return *this;
}

Point Point::move_towards(const Point& other, float distance) const {
    const float angle = this->angle_between(other);
    return {this->x + distance * std::cos(angle), this->y + distance * std::sin(angle)};
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
    const Point cell_position = this->position % cell_size;

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

RelativePose::RelativePose(const Pose& absolute_pose) : absolute_pose{absolute_pose} { }

Pose RelativePose::get() const {
    return {
        this->absolute_pose.position - this->reference_pose.position,
        core::assert_angle(this->absolute_pose.orientation - this->reference_pose.orientation)
    };
}

void RelativePose::reset_reference() {
    this->reference_pose = this->absolute_pose;
}
}  // namespace micras::nav
