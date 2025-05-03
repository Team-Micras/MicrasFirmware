/**
 * @file
 */

#ifndef MICRAS_CORE_VECTOR_HPP
#define MICRAS_CORE_VECTOR_HPP

namespace micras::core {
/**
 * @brief Type to store a point in 2D space.
 */
struct Vector {
    /**
     * @brief Calculate the distance to another point.
     *
     * @param other The other point.
     * @return The distance to the other point.
     */
    float distance(const Vector& other) const;

    /**
     * @brief Calculate the distance of the point to the origin.
     *
     * @return The distance of the point to the origin.
     */
    float magnitude() const;

    /**
     * @brief Calculate the angle between two points.
     *
     * @param other The other point.
     * @return The angle between the two points.
     */
    float angle_between(const Vector& other) const;

    /**
     * @brief Move the point towards another point by a given distance.
     *
     * @param other The point to move towards.
     * @param distance The distance to move.
     * @return The point after moving towards the other point.
     */
    Vector move_towards(const Vector& other, float distance) const;

    /**
     * @brief Add a point to another.
     *
     * @param other The point to add.
     * @return The result of the addition.
     */
    Vector operator+(const Vector& other) const;

    /**
     * @brief Subtract a point from another.
     *
     * @param other The point to subtract.
     * @return The result of the subtraction.
     */
    Vector operator-(const Vector& other) const;

    /**
     * @brief Compare two points for equality.
     *
     * @param other The other point to compare.
     * @return True if the points are equal, false otherwise.
     */
    bool operator==(const Vector& other) const;

    /**
     * @brief Calculate the remainder of the division of the point by a value.
     *
     * @param value The value to divide the point by.
     * @return The remainder of the division.
     */
    Vector operator%(float value) const;

    /**
     * @brief The x coordinate of the point in 2D space.
     */
    float x;

    /**
     * @brief The y coordinate of the point in 2D space.
     */
    float y;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_VECTOR_HPP
