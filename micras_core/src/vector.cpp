/**
 * @file
 */

#include <cmath>
#include <numbers>

#include "micras/core/vector.hpp"

namespace micras::core {
float Vector::distance(const Vector& other) const {
    return std::hypot(other.x - this->x, other.y - this->y);
}

float Vector::magnitude() const {
    return std::hypot(this->x, this->y);
}

float Vector::angle_between(const Vector& other) const {
    return std::atan2(other.y - this->y, other.x - this->x);
}

Vector Vector::move_towards(const Vector& other, float distance) const {
    const float angle = this->angle_between(other);
    return {this->x + distance * std::cos(angle), this->y + distance * std::sin(angle)};
}

Vector Vector::operator+(const Vector& other) const {
    return {this->x + other.x, this->y + other.y};
}

Vector Vector::operator-(const Vector& other) const {
    return {this->x - other.x, this->y - other.y};
}

bool Vector::operator==(const Vector& other) const {
    return this->x == other.x and this->y == other.y;
}

Vector Vector::operator%(float value) const {
    return {std::fmod(this->x, value), std::fmod(this->y, value)};
}
}  // namespace micras::core
