/**
 * @file
 */

#ifndef MICRAS_NAV_SPEED_RAMP_HPP
#define MICRAS_NAV_SPEED_RAMP_HPP

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Ramp of the speeds of the robot down to rest, as hard as the limits allow, with no path.
 *
 * @details Each axis, linear and angular, starts at the speed it was measured at and falls towards
 * zero at the deceleration its limits give at the speed the ramp has reached, so it follows the
 * tires down to where the motors limit the braking. The ramp is only speeds and accelerations, with
 * no pose, for a loop on the speeds the wheels and the gyroscope measure.
 */
class SpeedRamp {
public:
    /**
     * @brief Speeds of the ramp at an instant, and the accelerations that lead to them.
     */
    struct Sample {
        Twist twist;         ///< The speeds.
        Twist acceleration;  ///< The accelerations that lead to them.
    };

    /**
     * @brief Start the ramp.
     *
     * @param speed The speeds the robot is at.
     * @param linear The limits of the linear motion.
     * @param angular The limits of the rotation.
     */
    void start(const Twist& speed, const MotionLimits& linear, const MotionLimits& angular);

    /**
     * @brief Advance the ramp by one iteration.
     *
     * @param elapsed_time Time since the last iteration, in seconds.
     * @return The speeds the robot should be at now, and their accelerations.
     */
    Sample update(float elapsed_time);

    /**
     * @brief Check if the ramp has reached rest.
     *
     * @return True once both speeds are zero.
     */
    bool is_finished() const;

private:
    /**
     * @brief Bring the speed of one axis down by one iteration.
     *
     * @param speed The speed of the axis, which is updated.
     * @param limits The limits of the axis.
     * @param elapsed_time Time since the last iteration, in seconds.
     * @return The acceleration of the axis over the iteration.
     */
    static float slow_down(float& speed, const MotionLimits& limits, float elapsed_time);

    /**
     * @brief Speeds the ramp has reached.
     */
    Twist speed{};

    /**
     * @brief Limits of the linear motion.
     */
    MotionLimits linear{};

    /**
     * @brief Limits of the rotation.
     */
    MotionLimits angular{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_SPEED_RAMP_HPP
