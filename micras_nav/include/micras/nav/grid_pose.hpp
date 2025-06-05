/**
 * @file
 */

#ifndef MICRAS_NAV_GRID_POSE_HPP
#define MICRAS_NAV_GRID_POSE_HPP

#include <cstdint>
#include <functional>

#include "micras/core/vector.hpp"
#include "micras/core/serializable.hpp"

namespace micras::nav {
/**
 * @brief Possible sides in the grid.
 */
enum Side : uint8_t {
    RIGHT = 0,
    UP = 1,
    LEFT = 2,
    DOWN = 3
};

/**
 * @brief Convert an angle in radians to a Grid side.
 *
 * @param angle The angle in radians.
 * @return The corresponding Grid side.
 */
Side angle_to_grid(float angle);

/**
 * @brief Type to store a point in the grid.
 */
struct GridPoint {
    /**
     * @brief Return the direction from this point to the next one.
     *
     * @param next The next point.
     * @return The direction from this point to the next one.
     */
    Side direction(const GridPoint& next) const;

    /**
     * @brief Convert the point to a grid point.
     *
     * @param point The point to convert.
     * @param cell_size The size of the grid cells.
     * @return The grid point corresponding to the point.
     */
    static GridPoint from_vector(const core::Vector& point, float cell_size);

    /**
     * @brief Convert a grid point to a point.
     *
     * @param cell_size The size of the grid cells.
     * @return The point corresponding to the grid point.
     */
    core::Vector to_vector(float cell_size) const;

    /**
     * @brief Move in the grid in the direction of the side.
     *
     * @param side The side to move to.
     * @return The new point after moving.
     */
    GridPoint operator+(const Side& side) const;

    /**
     * @brief Compare two points for equality.
     *
     * @param other The other point to compare.
     * @return True if the points are equal, false otherwise.
     */
    bool operator==(const GridPoint& other) const;

    /**
     * @brief The x coordinate of the point on the grid.
     */
    uint8_t x;

    /**
     * @brief The y coordinate of the point on the grid.
     */
    uint8_t y;
};

struct GridPose : public core::ISerializable {
    /**
     * @brief Default constructor.
     */
    GridPose() = default;

    /**
     * @brief Constructor with position and orientation.
     *
     * @param pos The position of the pose.
     * @param orient The orientation of the pose.
     */
    GridPose(const GridPoint& pos, Side orient) : position(pos), orientation(orient) { }

    /**
     * @brief Return the pose after moving forward.
     *
     * @return The pose after moving forward.
     */
    GridPose front() const;

    /**
     * @brief Return the pose after turning back.
     *
     * @return The pose after turning back.
     */
    GridPose turned_back() const;

    /**
     * @brief Return the pose after turning left.
     *
     * @return The pose after turning left.
     */
    GridPose turned_left() const;

    /**
     * @brief Return the pose after turning right.
     *
     * @return The pose after turning right.
     */
    GridPose turned_right() const;

    /**
     * @brief Gets the relative side of a grid point compared to current pose.
     *
     * @param other The grid point to check.
     * @return LEFT/RIGHT for left/right of pose, UP for in front, DOWN for behind.
     */
    Side get_relative_side(const GridPoint& other) const;

    /**
     * @brief Compare two poses for equality.
     *
     * @param other The other pose to compare.
     * @return True if the poses are equal, false otherwise.
     */
    bool operator==(const GridPose& other) const;

    /**
     * @brief The position of the pose on the grid.
     */
    GridPoint position;

    /**
     * @brief The orientation of the pose on the grid.
     */
    Side orientation;

    /**
     * @brief Serialize the grid pose.
     *
     * @return Serialized data.
     */
    std::vector<uint8_t> serialize() const override;

    /**
     * @brief Deserialize the grid pose.
     *
     * @param serial_data Serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override;
};
}  // namespace micras::nav

namespace std {
/**
 * @brief Hash specialization for the GridPoint type.
 *
 * @tparam T GridPoint type.
 */
template <>
struct hash<micras::nav::GridPoint> {
    size_t operator()(const micras::nav::GridPoint& point) const noexcept {
        const size_t h1 = hash<uint8_t>{}(point.x);
        const size_t h2 = hash<uint8_t>{}(point.y);
        return h1 ^ (h2 << 1);
    }
};

/**
 * @brief Hash specialization for the GridPose type.
 *
 * @tparam T GridPose type.
 */
template <>
struct hash<micras::nav::GridPose> {
    size_t operator()(const micras::nav::GridPose& pose) const noexcept {
        const size_t h1 = hash<micras::nav::GridPoint>{}(pose.position);
        const size_t h2 = hash<uint8_t>{}(pose.orientation);
        return h1 ^ (h2 << 1);
    }
};
}  // namespace std

#endif  // MICRAS_NAV_GRID_POSE_HPP
