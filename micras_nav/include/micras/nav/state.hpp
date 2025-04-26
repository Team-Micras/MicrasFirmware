/**
 * @file
 */

#ifndef MICRAS_NAV_POSE_HPP
#define MICRAS_NAV_POSE_HPP

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
/**
 * @brief Possible directions in the maze.
 */
enum Direction : uint8_t {
    EAST = 0,
    NORTHEAST = 1,
    NORTH = 2,
    NORTHWEST = 3,
    WEST = 4,
    SOUTHWEST = 5,
    SOUTH = 6,
    SOUTHEAST = 7,
};

/**
 * @brief Type to store a point in 2D space.
 */
struct Point {
    /**
     * @brief Calculate the distance to another point.
     *
     * @param other The other point.
     * @return The distance to the other point.
     */
    float distance(const Point& other) const;

    /**
     * @brief Calculate the angle between two points.
     *
     * @param other The other point.
     * @return The angle between the two points.
     */
    float angle_between(const Point& other) const;

    /**
     * @brief Convert the point to a grid point.
     *
     * @param cell_size The size of the grid cells.
     * @return The grid point corresponding to the point.
     */
    GridPoint to_grid(float cell_size) const;

    /**
     * @brief Rotate the point by a given angle.
     *
     * @param angle The angle to rotate the point by.
     * @return The rotated point.
     */
    Point rotate(Direction angle);

    /**
     * @brief Move the point towards another point by a given distance.
     *
     * @param other The point to move towards.
     * @param distance The distance to move.
     * @return The point after moving towards the other point.
     */
    Point move_towards(const Point& other, float distance) const;

    /**
     * @brief Subtract a point from another.
     *
     * @param other The point to subtract.
     * @return The result of the subtraction.
     */
    Point operator-(const Point& other) const;

    /**
     * @brief Compare two points for equality.
     *
     * @param other The other point to compare.
     * @return True if the points are equal, false otherwise.
     */
    bool operator==(const Point& other) const;

    /**
     * @brief Calculate the remainder of the division of the point by a value.
     *
     * @param value The value to divide the point by.
     * @return The remainder of the division.
     */
    Point operator%(float value) const;

    /**
     * @brief Convert a grid point to a point.
     *
     * @param grid_point The grid point to convert.
     * @param cell_size The size of the grid cells.
     * @return The point corresponding to the grid point.
     */
    static Point from_grid(const GridPoint& grid_point, float cell_size);

    /**
     * @brief The x coordinate of the point in 2D space.
     */
    float x;

    /**
     * @brief The y coordinate of the point in 2D space.
     */
    float y;
};

/**
 * @brief Type to store a pose in 2D space.
 */
struct Pose {
    /**
     * @brief Convert the pose to a grid pose.
     *
     * @param cell_size The size of the grid cells.
     * @return The grid pose corresponding to the pose.
     */
    GridPose to_grid(float cell_size) const;

    /**
     * @brief Convert the pose to a cell reference.
     *
     * @param cell_size The size of the grid cells.
     * @return The point inside the cell reference.
     */
    Point to_cell(float cell_size) const;

    /**
     * @brief The position of the pose.
     */
    Point position;

    /**
     * @brief The orientation of the pose.
     */
    float orientation;
};

/**
 * @brief Type to store a twist in 2D space.
 */
struct Twist {
    /**
     * @brief The linear velocity of the twist.
     */
    float linear;

    /**
     * @brief The angular velocity of the twist.
     */
    float angular;
};

/**
 * @brief Type to store the state of the robot.
 */
struct State {
    /**
     * @brief The pose of the robot.
     */
    Pose pose;

    /**
     * @brief The velocity of the robot.
     */
    Twist velocity;
};

class RelativePose {
public:
    /**
     * @brief Construct a new Relative Pose object.
     *
     * @param absolute_pose The absolute pose to be used as a reference.
     */
    RelativePose(const Pose& absolute_pose);

    /**
     * @brief Get the relative pose.
     *
     * @return The pose relative to the reference.
     */
    Pose get() const;

    /**
     * @brief Reset the reference to the current absolute pose.
     */
    void reset_reference();

private:
    /**
     * @brief A reference to the absolute pose.
     */
    const Pose& absolute_pose;

    /**
     * @brief The reference pose to be used for calculations.
     */
    Pose reference_pose{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_POSE_HPP
