/**
 * @file
 */

#ifndef MICRAS_NAV_POSE_HPP
#define MICRAS_NAV_POSE_HPP

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
/**
 * @brief Type to store a point in 2D space
 */
struct Point {
    /**
     * @brief Calculates the distance to another point
     *
     * @param other The other point
     * @return The distance to the other point
     */
    float distance(const Point& other) const;

    /**
     * @brief Calculates the angle between two points
     *
     * @param other The other point
     * @return The angle between the two points
     */
    float angle_between(const Point& other) const;

    /**
     * @brief Converts the point to a grid point
     *
     * @param cell_size The size of the grid cells
     * @return The grid point corresponding to the point
     */
    GridPoint to_grid(float cell_size) const;

    /**
     * @brief Rotates the point by a given angle
     *
     * @param angle The angle to rotate the point by
     * @return The rotated point
     */
    Point rotate(Side angle);

    /**
     * @brief Subtracts a point from another
     *
     * @param other The point to subtract
     * @return The result of the subtraction
     */
    Point operator-(const Point& other) const;

    /**
     * @brief Compares two points for equality
     *
     * @param other The other point to compare
     * @return True if the points are equal, false otherwise
     */
    bool operator==(const Point& other) const;

    /**
     * @brief Calculates the remainder of the division of the point by a value
     *
     * @param value The value to divide the point by
     * @return The remainder of the division
     */
    Point operator%(float value) const;

    /**
     * @brief Converts a grid point to a point
     *
     * @param grid_point The grid point to convert
     * @param cell_size The size of the grid cells
     * @return The point corresponding to the grid point
     */
    static Point from_grid(const GridPoint& grid_point, float cell_size);

    /**
     * @brief The x coordinate of the point in 2D space
     */
    float x;

    /**
     * @brief The y coordinate of the point in 2D space
     */
    float y;
};

/**
 * @brief Type to store a pose in 2D space
 */
struct Pose {
    /**
     * @brief Converts the pose to a grid pose
     *
     * @param cell_size The size of the grid cells
     * @return The grid pose corresponding to the pose
     */
    GridPose to_grid(float cell_size) const;

    /**
     * @brief Converts the pose to a cell reference
     *
     * @param cell_size The size of the grid cells
     * @return The point inside the cell reference
     */
    Point to_cell(float cell_size) const;

    /**
     * @brief The position of the pose
     */
    Point position;

    /**
     * @brief The orientation of the pose
     */
    float orientation;
};

/**
 * @brief Type to store a twist in 2D space
 */
struct Twist {
    /**
     * @brief The linear velocity of the twist
     */
    float linear;

    /**
     * @brief The angular velocity of the twist
     */
    float angular;
};

/**
 * @brief Type to store the state of the robot
 */
struct State {
    /**
     * @brief The pose of the robot
     */
    Pose pose;

    /**
     * @brief The velocity of the robot
     */
    Twist velocity;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_POSE_HPP
