/**
 * @file
 */

#ifndef MICRAS_NAV_POSE_HPP
#define MICRAS_NAV_POSE_HPP

#include "micras/core/vector.hpp"
#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
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
    core::Vector to_cell(float cell_size) const;

    /**
     * @brief The position of the pose.
     */
    core::Vector position;

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
    explicit RelativePose(const Pose& absolute_pose);

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
    const Pose* absolute_pose;

    /**
     * @brief The reference pose to be used for calculations.
     */
    Pose reference_pose{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_POSE_HPP
