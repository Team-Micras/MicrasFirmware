/**
 * @file
 */

#ifndef MICRAS_NAV_CURVE_SPEED_HPP
#define MICRAS_NAV_CURVE_SPEED_HPP

#include <array>
#include <cstdint>
#include <span>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
/**
 * @brief Fastest motion along a curve between two speeds, as a function of time.
 *
 * @details The curve is sampled at equal steps of distance. Each sample gets the largest speed its
 * bending allows, a pass from the start then lowers every sample to what can be reached from the one
 * before, with the grip the curve leaves there, and a pass from the end lowers it to what can be
 * braked from before the one after. Between two samples the acceleration is constant. A turn is
 * therefore braked into as long as its curvature leaves grip to spare and accelerated out of as
 * soon as it does, instead of being driven at the speed of its tightest point.
 *
 * The passes are written over any curve, given as a function of the index of a sample that returns
 * the curvature at that sample and the sharpness between it and the next one, and over storage that
 * the caller owns, which is how the racing line uses them over thousands of samples. An object of
 * this class holds the samples of one turn, which is all the executor needs to play it back.
 */
class CurveSpeed {
public:
    /**
     * @brief Largest number of steps a turn is sampled with.
     */
    static constexpr uint8_t max_intervals{128};

    /**
     * @brief Distance between two samples of a turn, in meters, which the longest turns stretch.
     */
    static constexpr float turn_spacing{0.005F};

    /**
     * @brief Construct an empty motion, of zero duration.
     */
    CurveSpeed() = default;

    /**
     * @brief Construct the motion along a turn.
     *
     * @note Speeds the turn does not allow are lowered, like the velocity planner would lower them,
     * so the motion always covers the turn.
     *
     * @param shape The shape of the turn.
     * @param start_speed The speed at the start of the curve.
     * @param end_speed The speed wanted at the end of the curve.
     * @param limits The limits of the run.
     */
    CurveSpeed(const TurnShape& shape, float start_speed, float end_speed, const CurveLimits& limits);

    /**
     * @brief Get the time the motion takes.
     *
     * @return The duration in seconds.
     */
    float duration() const;

    /**
     * @brief Evaluate the motion at an instant.
     *
     * @param time Time since the start of the motion, clamped to its duration.
     * @return The distance covered, the speed and the acceleration at that instant.
     */
    SpeedProfile::Sample sample(float time) const;

    /**
     * @brief Get the largest speed a turn can be entered at.
     *
     * @param shape The shape of the turn.
     * @param end_speed The speed wanted at the end of the curve.
     * @param limits The limits of the run.
     * @return The speed at the start when braking all the way to the end speed.
     */
    static float get_entry_speed(const TurnShape& shape, float end_speed, const CurveLimits& limits);

    /**
     * @brief Get the largest speed a turn can be left at.
     *
     * @param shape The shape of the turn.
     * @param start_speed The speed at the start of the curve.
     * @param limits The limits of the run.
     * @return The speed at the end when accelerating all the way from the start speed.
     */
    static float get_exit_speed(const TurnShape& shape, float start_speed, const CurveLimits& limits);

    /**
     * @brief Fill in the speed of every sample of a curve.
     *
     * @tparam F Type of the function giving the bending at a sample, as described for the class.
     * @param bending The bending of the curve, called for every sample.
     * @param speeds The speed of each sample, one more than there are steps.
     * @param spacing The distance between two samples.
     * @param start_speed The speed at the first sample.
     * @param end_speed The speed wanted at the last sample.
     * @param limits The limits of the run.
     */
    template <typename F>
    static void plan(
        F&& bending, std::span<float> speeds, float spacing, float start_speed, float end_speed,
        const CurveLimits& limits
    );

    /**
     * @brief Get the time at which every sample of a curve is reached.
     *
     * @param speeds The speed of each sample.
     * @param spacing The distance between two samples.
     * @param times The time of each sample, the first being zero.
     */
    static void integrate(std::span<const float> speeds, float spacing, std::span<float> times);

    /**
     * @brief Evaluate the motion along a curve at an instant.
     *
     * @param speeds The speed of each sample.
     * @param times The time of each sample.
     * @param spacing The distance between two samples.
     * @param time Time since the start of the motion, clamped to its duration.
     * @return The distance covered, the speed and the acceleration at that instant.
     */
    static SpeedProfile::Sample
        sample(std::span<const float> speeds, std::span<const float> times, float spacing, float time);

private:
    /**
     * @brief Smallest sum of the speeds of two samples a step is timed with, in m/s, which only
     * matters for a curve that starts or ends at rest.
     */
    static constexpr float min_speed_sum{1.0e-3F};

    /**
     * @brief Get the number of steps a turn is sampled with.
     *
     * @param shape The shape of the turn.
     * @return The number of steps.
     */
    static uint8_t get_intervals(const TurnShape& shape);

    /**
     * @brief Get the largest speed at a sample.
     *
     * @param before The bending between the sample before and this one.
     * @param at The bending at this sample and between it and the next one.
     * @param limits The limits of the run.
     * @return The speed limit, which takes the sharper side of the sample.
     */
    static float get_limit(const Bending& before, const Bending& at, const CurveLimits& limits);

    /**
     * @brief Speed at every sample.
     */
    std::array<float, max_intervals + 1> speeds{};

    /**
     * @brief Time at which every sample is reached.
     */
    std::array<float, max_intervals + 1> times{};

    /**
     * @brief Number of steps between the samples.
     */
    uint8_t intervals{};

    /**
     * @brief Distance between two samples.
     */
    float spacing{};
};
}  // namespace micras::nav

#include "micras/nav/impl/curve_speed.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_CURVE_SPEED_HPP
