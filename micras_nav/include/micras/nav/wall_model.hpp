/**
 * @file
 */

#ifndef MICRAS_NAV_WALL_MODEL_HPP
#define MICRAS_NAV_WALL_MODEL_HPP

#include <array>
#include <cstdint>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief What the optical axis of a wall sensor meets.
 *
 * @details The wall is the first one along the axis that is not known to be absent, so its state is
 * either a wall or unknown. The range is the distance from the sensor to the face of that wall, the
 * offset is where along the wall the axis lands, measured from the end of the wall with the lowest
 * coordinate, and the cosine is that of the angle between the axis and the perpendicular of the
 * wall. The jacobian is the derivative of the range with respect to the x, y and orientation of
 * the robot. The next three fields place the wall in the maze frame: whether it runs along y, the
 * coordinate of its face across it, and the coordinate along it where the offset is measured from.
 * The ends are the offsets between which the face is known to be one continuous surface: a known
 * wall continues over its posts and over the known walls in line with it, while an unknown one is
 * only considered between its posts.
 *
 * @note A hit is not valid when the axis meets nothing within range, or meets a post that has no
 * known wall flush with it.
 */
struct RayHit {
    bool                 valid;
    GridPose             wall;
    WallState            state;
    float                range;
    float                offset;
    float                cosine;
    std::array<float, 3> jacobian;
    bool                 vertical;
    float                face;
    float                base;
    float                low_end;
    float                high_end;
};

/**
 * @brief Where the optical axis of a sensor crosses the plane of a wall, extended past its ends.
 *
 * @note The offset is measured along the wall like the one of a RayHit, the range is the distance
 * from the sensor to the plane along the axis, and the jacobian is the derivative of the offset with
 * respect to the x, y and orientation of the robot.
 */
struct PlaneCrossing {
    float                offset;
    float                range;
    std::array<float, 3> jacobian;
};

/**
 * @brief Geometry of the wall sensors against the map of the walls.
 *
 * @details It answers one question, what should a sensor see from a pose, for the two users that
 * need it. The localizer compares the answer with the reading to correct the pose, and the wall
 * observer compares it to decide whether a wall that is still unknown is there. Both only trust a
 * reading when the whole spot the emitter illuminates, enlarged by how uncertain the pose is, lies
 * on the face of a single wall: not on a post, not across the gap at the end of the wall.
 */
class WallModel {
public:
    /**
     * @brief Configuration struct for the wall model.
     *
     * @note A wall is only looked for between the two ranges, which are what the sensors measure
     * well. The edge margin is kept between the illuminated spot and the ends of a wall, and the
     * confidence is how many standard deviations of the pose the spot is enlarged by.
     */
    struct Config {
        RobotModel model;
        float      min_range;
        float      max_range;
        float      edge_margin;
        float      confidence;
    };

    /**
     * @brief Construct a new Wall Model object.
     *
     * @param config The configuration for the wall model.
     */
    explicit WallModel(const Config& config);

    /**
     * @brief Find what the optical axis of a sensor meets.
     *
     * @tparam width The width of the maze in cells.
     * @tparam height The height of the maze in cells.
     * @param pose The pose of the robot in the maze frame.
     * @param sensor The index of the sensor.
     * @param maze The map of the walls.
     * @return The first wall along the axis that is not known to be absent.
     */
    template <uint8_t width, uint8_t height>
    RayHit cast(const Pose& pose, uint8_t sensor, const TMaze<width, height>& maze) const;

    /**
     * @brief Find where the optical axis of a sensor crosses the plane of a wall it met before.
     *
     * @param pose The pose of the robot in the maze frame.
     * @param sensor The index of the sensor.
     * @param hit The hit that identified the wall.
     * @return The crossing of the axis with the plane of the face of that wall.
     */
    PlaneCrossing cross(const Pose& pose, uint8_t sensor, const RayHit& hit) const;

    /**
     * @brief Check if a sensor looks far enough to the side to sweep along the walls beside the robot.
     *
     * @param sensor The index of the sensor.
     * @return True if the sensor can see the ends of the side walls go by.
     */
    bool is_side_looking(uint8_t sensor) const;

    /**
     * @brief Check if the spot a sensor illuminates lies entirely on the face of the wall it meets.
     *
     * @param hit What the axis of the sensor meets.
     * @param sensor The index of the sensor.
     * @param position_deviation The standard deviation of the position of the robot, in meters.
     * @param orientation_deviation The standard deviation of the orientation of the robot, in radians.
     * @return True if a reading of the sensor can only come from that wall.
     */
    bool is_footprint_clear(
        const RayHit& hit, uint8_t sensor, float position_deviation, float orientation_deviation
    ) const;

    /**
     * @brief Get the range a sensor reads with the robot centered in a cell, facing along it.
     *
     * @note This is the pose the sensors are calibrated in: the side sensors against the walls of
     * a corridor and the front ones against a wall ahead.
     *
     * @param sensor The index of the sensor.
     * @return The range to the wall the sensor points at, or zero if it points at none.
     */
    float get_centered_range(uint8_t sensor) const;

    /**
     * @brief Get the standard deviation of a range reading.
     *
     * @param range The range in meters.
     * @return The standard deviation in meters.
     */
    float get_range_deviation(float range) const;

private:
    /**
     * @brief Sine of the angle of the optical axis from which a sensor counts as looking sideways.
     */
    static constexpr float side_looking_sine{0.3F};

    /**
     * @brief Dimensions of the maze.
     */
    RobotModel::Maze maze_geometry;

    /**
     * @brief Mounting of each sensor.
     */
    std::array<RobotModel::WallSensor, number_of_wall_sensors> sensors;

    /**
     * @brief Constant part of the range noise.
     */
    float range_noise;

    /**
     * @brief Part of the range noise proportional to the range.
     */
    float range_noise_per_meter;

    /**
     * @brief Shortest range a wall is looked for at.
     */
    float min_range;

    /**
     * @brief Largest range a wall is looked for at.
     */
    float max_range;

    /**
     * @brief Distance kept between the illuminated spot and the ends of a wall.
     */
    float edge_margin;

    /**
     * @brief Number of standard deviations of the pose the spot is enlarged by.
     */
    float confidence;
};
}  // namespace micras::nav

#include "micras/nav/impl/wall_model.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_WALL_MODEL_HPP
