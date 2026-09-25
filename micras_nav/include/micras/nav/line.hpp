/**
 * @file
 */

#ifndef MICRAS_NAV_LINE_HPP
#define MICRAS_NAV_LINE_HPP

#include <array>
#include <cstdint>

#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
class TRacingLine;

/**
 * @brief A path given by samples at equal steps of distance, with the speed of the robot at each.
 *
 * @details This is the racing line as the executor plays it back. Between two samples the position
 * follows the cubic that leaves each sample along its heading, the heading and the curvature change
 * linearly and the acceleration is constant, so the reference is smooth and its derivatives are the
 * ones the path and the speeds imply. The heading of a sample is the direction from the one before
 * it to the one after it.
 *
 * @note The samples live in static arrays, which is most of the memory the racing line takes.
 */
class Line {
public:
    /**
     * @brief Largest number of samples, which at 10 mm is a path of 25.6 m.
     */
    static constexpr uint16_t max_samples{2560};

    /**
     * @brief Point of the path at a distance from its start.
     */
    struct Point {
        Pose  pose;
        float curvature;
        float sharpness;
    };

    /**
     * @brief Check if the line was planned and timed.
     *
     * @return True if it can be driven.
     */
    bool is_ready() const;

    /**
     * @brief Get the length of the path.
     *
     * @return The length in meters.
     */
    float length() const;

    /**
     * @brief Get the time the motion along the path takes, from rest to rest.
     *
     * @return The duration in seconds.
     */
    float duration() const;

    /**
     * @brief Get the time at which the robot crosses into the goal.
     *
     * @return The time in seconds, which is how a fast run is timed.
     */
    float get_finish_time() const;

    /**
     * @brief Get the pose at the start of the path.
     *
     * @return The pose in the maze frame.
     */
    Pose get_start() const;

    /**
     * @brief Evaluate the motion along the path at an instant.
     *
     * @param time Time since the start of the motion, clamped to its duration.
     * @return The distance covered, the speed and the acceleration at that instant.
     */
    SpeedProfile::Sample sample_motion(float time) const;

    /**
     * @brief Evaluate the path at a distance from its start.
     *
     * @param distance The distance along the path, clamped to its length.
     * @return The pose in the maze frame, the curvature and the sharpness there.
     */
    Point sample_point(float distance) const;

private:
    template <uint8_t width, uint8_t height>
    friend class TRacingLine;

    /**
     * @brief Get the heading of the path at a sample.
     *
     * @param index The index of the sample.
     * @return The heading in radians.
     */
    float get_heading(uint16_t index) const;

    /**
     * @brief Position of every sample in the maze frame.
     *
     * @note While the line is optimized, the curvatures and the speeds hold the positions being
     * resampled.
     */
    ///@{
    std::array<float, max_samples> xs{};
    std::array<float, max_samples> ys{};
    ///@}

    /**
     * @brief Curvature at every sample.
     */
    std::array<float, max_samples> curvatures{};

    /**
     * @brief Speed at every sample.
     */
    std::array<float, max_samples> speeds{};

    /**
     * @brief Time at which every sample is reached.
     */
    std::array<float, max_samples> times{};

    /**
     * @brief Number of samples.
     */
    uint16_t size{};

    /**
     * @brief Distance between two samples.
     */
    float spacing{};

    /**
     * @brief Time at which the robot crosses into the goal.
     */
    float finish_time{};

    /**
     * @brief Whether the line was planned and timed.
     */
    bool ready{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_LINE_HPP
