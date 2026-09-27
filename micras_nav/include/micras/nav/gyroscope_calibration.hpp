/**
 * @file
 */

#ifndef MICRAS_NAV_GYROSCOPE_CALIBRATION_HPP
#define MICRAS_NAV_GYROSCOPE_CALIBRATION_HPP

#include <cstdint>

#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Procedure that measures the scale factor of the gyroscope.
 *
 * @details The robot is placed facing a wall and turns a whole number of times in place. Its
 * orientation relative to the wall is measured before and after with the two sensors that look
 * forward, from the line through the two points they see, so the angle it really turned is known
 * to a fraction of a degree however many turns it made. Dividing it by what the gyroscope
 * integrated over the same time gives the factor that corrects the sensitivity of the gyroscope,
 * which is the constant to type into the robot model.
 *
 * @note More turns divide the error of the two wall measurements by a larger angle, so the result
 * gets better with the number of turns, as long as the robot stays in front of the wall.
 */
class GyroscopeCalibration {
public:
    /**
     * @brief Configuration struct for the calibration.
     *
     * @note The sensors are the two that look forward, to the left and to the right of the center
     * line. The robot stands still for the settle time before and after turning, and the wall is
     * measured over the second half of it.
     */
    struct Config {
        RobotModel model;
        uint8_t    left_sensor;
        uint8_t    right_sensor;
        float      turns;
        float      settle_time;
    };

    /**
     * @brief Construct a new Gyroscope Calibration object.
     *
     * @param config The configuration for the calibration.
     */
    explicit GyroscopeCalibration(const Config& config);

    /**
     * @brief Start the procedure.
     *
     * @param pose The pose of the robot, which is held while it turns.
     * @param limits The limits of the rotation in place.
     */
    void start(const Pose& pose, const MotionLimits& limits);

    /**
     * @brief Advance the procedure by one iteration.
     *
     * @param measurements The current measurements.
     * @param bias The bias of the gyroscope, in rad/s after the scale correction.
     * @param elapsed_time Time since the last iteration, in seconds.
     * @return What the robot should be doing at this instant.
     */
    Reference update(const Measurements& measurements, float bias, float elapsed_time);

    /**
     * @brief Check if the procedure has ended.
     *
     * @return True once the wall was measured for the second time.
     */
    bool is_finished() const;

    /**
     * @brief Check if the wall was seen by both sensors during both measurements.
     *
     * @return True if the result can be trusted.
     */
    bool is_valid() const;

    /**
     * @brief Get the scale factor that was measured.
     *
     * @return The value for the gyroscope scale of the robot model.
     */
    float get_scale() const;

private:
    /**
     * @brief Steps of the procedure.
     */
    enum class Phase : uint8_t {
        BEFORE = 0,
        TURNING = 1,
        AFTER = 2,
        FINISHED = 3,
    };

    /**
     * @brief Get the orientation of the robot relative to the wall ahead of it.
     *
     * @param measurements The current measurements.
     * @return The angle between the heading of the robot and the perpendicular of the wall.
     */
    float get_wall_angle(const Measurements& measurements) const;

    /**
     * @brief Parameters of the procedure, with the physical description of the robot.
     */
    Config config;

    /**
     * @brief Current step of the procedure.
     */
    Phase phase{Phase::FINISHED};

    /**
     * @brief Time since the current step started.
     */
    float phase_time{};

    /**
     * @brief Pose held by the robot, whose orientation is the one before turning.
     */
    Pose pose{};

    /**
     * @brief Rotation in place.
     */
    SpeedProfile spin;

    /**
     * @brief Sum and number of the measurements of the wall being averaged.
     */
    ///@{
    float    angle_sum{};
    uint32_t angle_count{};
    ///@}

    /**
     * @brief Orientation relative to the wall before turning.
     */
    float angle_before{};

    /**
     * @brief Rotation reported by the raw gyroscope while turning, and the time it took.
     */
    ///@{
    float raw_rotation{};
    float turning_time{};
    ///@}

    /**
     * @brief Bias of the gyroscope when the robot started to turn.
     */
    float bias{};

    /**
     * @brief Scale factor that was measured.
     */
    float scale{1.0F};

    /**
     * @brief Whether both measurements of the wall were good.
     */
    bool valid{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_GYROSCOPE_CALIBRATION_HPP
