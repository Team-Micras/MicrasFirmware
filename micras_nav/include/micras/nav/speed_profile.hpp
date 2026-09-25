/**
 * @file
 */

#ifndef MICRAS_NAV_SPEED_PROFILE_HPP
#define MICRAS_NAV_SPEED_PROFILE_HPP

#include <array>
#include <cstdint>

#include "micras/nav/motion_limits.hpp"

namespace micras::nav {
/**
 * @brief Fastest motion along one axis between two speeds, as a function of time.
 *
 * @details The motion accelerates as hard as the limits allow, cruises if it reaches the speed
 * limit and brakes at the last moment, which is the time optimal way to cover a distance. It is the
 * single evaluator of the navigation: the planner prices a straight with duration(), the velocity
 * planner chains straights with get_reachable_speed() and get_brakeable_speed(), and the executor
 * plays the very same object back with sample(), so that the time a route is chosen by is the time
 * it takes. The axis is a distance for a straight and an angle for a rotation in place.
 */
class SpeedProfile {
public:
    /**
     * @brief State of the motion at one instant.
     */
    struct Sample {
        float distance;
        float speed;
        float acceleration;
    };

    /**
     * @brief Construct an empty profile, of zero duration.
     */
    SpeedProfile() = default;

    /**
     * @brief Construct a new Speed Profile object.
     *
     * @note Speeds that the distance does not allow are not an error. A start speed too high to
     * brake from is braked harder than the limit, and an end speed that cannot be reached leaves
     * the motion accelerating all the way, so the profile always covers exactly the distance. It is
     * the velocity planner that keeps the speeds it asks for feasible.
     *
     * @param distance The distance to cover, which must not be negative.
     * @param start_speed The speed at the start.
     * @param end_speed The speed wanted at the end.
     * @param limits The limits of the motion.
     */
    SpeedProfile(float distance, float start_speed, float end_speed, const MotionLimits& limits);

    /**
     * @brief Get the time the motion takes.
     *
     * @return The duration in seconds.
     */
    float duration() const;

    /**
     * @brief Get the distance the motion covers.
     *
     * @return The distance.
     */
    float distance() const;

    /**
     * @brief Evaluate the motion at an instant.
     *
     * @param time Time since the start of the motion, clamped to its duration.
     * @return The distance covered, the speed and the acceleration at that instant.
     */
    Sample sample(float time) const;

    /**
     * @brief Get the instant at which a distance has been covered.
     *
     * @param distance The distance, clamped to the one the motion covers.
     * @return The time since the start of the motion in seconds.
     */
    float time_at(float distance) const;

    /**
     * @brief Get the largest speed that can be reached over a distance.
     *
     * @param distance The distance available to accelerate.
     * @param start_speed The speed at the start.
     * @param limits The limits of the motion.
     * @return The speed at the end when accelerating all the way, capped at the speed limit.
     */
    static float get_reachable_speed(float distance, float start_speed, const MotionLimits& limits);

    /**
     * @brief Get the largest speed that can be braked from over a distance.
     *
     * @param distance The distance available to brake.
     * @param end_speed The speed at the end.
     * @param limits The limits of the motion.
     * @return The speed at the start when braking all the way, capped at the speed limit.
     */
    static float get_brakeable_speed(float distance, float end_speed, const MotionLimits& limits);

private:
    /**
     * @brief Stretch of the motion with one law for the acceleration.
     *
     * @note The rate is zero for a constant acceleration. Otherwise the acceleration decays as the
     * speed approaches the target, as `rate * (target - speed)`, which is the motor limited stretch.
     */
    struct Phase {
        float duration{};
        float start_distance{};
        float start_speed{};
        float acceleration{};
        float rate{};
        float target{};
    };

    /**
     * @brief Number of phases: traction limited, motor limited, cruise and braking.
     */
    static constexpr uint8_t number_of_phases{4};

    /**
     * @brief Get the distance needed to accelerate between two speeds.
     *
     * @param start_speed The speed at the start.
     * @param end_speed The speed at the end, not lower than the start and below the free speed.
     * @param limits The limits of the motion.
     * @return The distance covered while accelerating as hard as the limits allow.
     */
    static float get_acceleration_distance(float start_speed, float end_speed, const MotionLimits& limits);

    /**
     * @brief Append a phase, advancing the state of the motion to its end.
     *
     * @param phase The phase, with its duration, acceleration, rate and target filled in.
     */
    void append(Phase phase);

    /**
     * @brief Evaluate a phase at an instant.
     *
     * @param phase The phase.
     * @param time Time since the start of the phase.
     * @return The distance covered since the start of the motion, the speed and the acceleration.
     */
    static Sample evaluate(const Phase& phase, float time);

    /**
     * @brief Phases of the motion, in order, the unused ones with a null duration.
     */
    std::array<Phase, number_of_phases> phases{};

    /**
     * @brief Number of phases in use.
     */
    uint8_t size{};

    /**
     * @brief Time the motion takes.
     */
    float total_duration{};

    /**
     * @brief Distance the motion covers.
     */
    float total_distance{};

    /**
     * @brief Speed at the end of the last phase appended.
     */
    float last_speed{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_SPEED_PROFILE_HPP
