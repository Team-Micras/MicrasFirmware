/**
 * @file
 */

#ifndef MICRAS_NAV_CONTROLLER_HPP
#define MICRAS_NAV_CONTROLLER_HPP

#include <array>

#include "micras/nav/robot_model.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Computation of the motor commands that make the robot follow a reference.
 *
 * @details The command has two axes, forward and rotation. On each of them a feed forward, which is
 * the model of the drive train fed with the speed and the acceleration of the reference, supplies
 * almost all of the voltage, and a feedback closes the loop on the integrated quantities, the
 * position along the path and the orientation, with a derivative term on the speed:
 *
 *     forward  = feed_forward(v, a)         + K_s * e_s                       + D_s * (v - v_measured)
 *     rotation = feed_forward(omega, alpha) + K_o * (e_o + steering(e_y))     + D_o * (omega - omega_measured)
 *
 * The errors are those of the estimated pose as seen from the pose of the reference: e_s along the
 * path, e_y across it and e_o in orientation. There is no integrator, so there is nothing to wind up
 * or to reset between segments. The steering is a small and bounded offset of the orientation,
 * proportional to the error across the path, which turns the robot back towards it at a rate set by
 * the distance traveled rather than by time. It works wherever the pose is known, walls or not.
 *
 * In a curve the tires slide to the outside at a speed the lateral compliance of the model gives,
 * so the robot has to point into the curve by that speed over its own, which is the compliance
 * times the angular speed of the reference, to move along the path. That angle is added to the
 * orientation the feedback aims at, and its rate to the angular speed.
 *
 * @note The gains are not tuned: they follow from the model of the drive train and from the natural
 * frequency and damping asked of each axis.
 */
class Controller {
public:
    /**
     * @brief Closed loop behavior asked of one axis.
     *
     * @note The natural frequency is in rad/s. The error is clamped before it is used, which keeps a
     * large error from swamping the rest of the command. It does not keep the motors from
     * saturating when the robot is held, since the proportional term alone can reach the supply
     * within the clamp; the robot stops on the saturation timeout for that.
     */
    struct Axis {
        float natural_frequency;
        float damping;
        float max_error;
    };

    /**
     * @brief Configuration struct for the controller.
     *
     * @note The steering gain is in radians of offset per meter of error and the offset is limited
     * to the largest steering. Below the blend speed the steering fades out, since turning in place
     * cannot reduce an error across the path. The friction speed is the wheel speed over which the
     * static friction compensation goes from nothing to all of it. A change of the time scale is
     * itself an acceleration of the reference, of its rate times the speed, so the rate of the
     * scale times the speed of the faster wheel is kept within the largest time scale acceleration.
     */
    struct Config {
        RobotModel model;
        Axis       linear;
        Axis       angular;
        float      steering_gain;
        float      max_steering;
        float      steering_blend_speed;
        float      friction_speed;
        float      voltage_reserve;
        float      max_time_scale_acceleration;
    };

    /**
     * @brief Command for the locomotion, in percent of the motor supply.
     */
    struct Command {
        float forward;
        float rotation;
    };

    /**
     * @brief Errors and terms of the last update, for a monitor.
     */
    struct Status {
        float along_error;
        float across_error;
        float orientation_error;
        float forward_feed_forward;
        float rotation_feed_forward;
        float forward_feedback;
        float rotation_feedback;
    };

    /**
     * @brief Construct a new Controller object.
     *
     * @param config The configuration for the controller.
     */
    explicit Controller(const Config& config);

    /**
     * @brief Compute the command for this instant.
     *
     * @note The reference is played at the time scale found for it, which scales its speeds by the
     * scale and its accelerations by the square of the scale, plus the rate of the scale times the
     * speed.
     *
     * @param unscaled What the robot should be doing, in the maze frame, at the full pace.
     * @param estimate What the robot is doing, as far as it is known.
     * @param elapsed_time Time since the previous update, in seconds.
     * @return The command for the locomotion.
     */
    Command update(const Reference& unscaled, const State& estimate, float elapsed_time);

    /**
     * @brief Get how much slower the reference has to be played for the motors to follow it.
     *
     * @note The feed forward alone may ask for more voltage than there is. Slowing the clock of
     * the reference scales its linear and angular speeds together, so the robot stays on the same
     * path and only takes longer, instead of cutting the turn it is in. While the grip limits the
     * planned motions this stays at one; where the motors limit them, a straight uses the whole of
     * the voltage on the forward axis, and any rotation on top needs it. The scale is the largest
     * one at which the feed forward of both wheels fits in the voltage available, found from the
     * reference at the full pace, so it does not depend on the scale of the previous iteration.
     *
     * @return The factor to apply to the elapsed time of the reference, in (0, 1].
     */
    float get_time_scale() const;

    /**
     * @brief Get the errors and terms of the last update.
     *
     * @return The status.
     */
    const Status& get_status() const;

private:
    /**
     * @brief Feedback gains of one axis.
     */
    struct Gains {
        float proportional;
        float derivative;
    };

    /**
     * @brief Compute the feedback gains of an axis from the model of the drive train.
     *
     * @param axis The behavior asked of the axis.
     * @param speed_constant The voltage per unit of speed of the axis.
     * @param acceleration_constant The voltage per unit of acceleration of the axis.
     * @return The gains, in volts per unit of position and of speed.
     */
    static Gains compute_gains(const Axis& axis, float speed_constant, float acceleration_constant);

    /**
     * @brief Find the time scale of this iteration, moving towards the one the reference fits at.
     *
     * @note The scale falls as fast as the time scale acceleration allows. It rises no faster than
     * that either, and only as fast as the voltage left over at the new scale lets through the
     * acceleration the rise adds, so that rising never saturates the motors by itself.
     *
     * @param reference What the robot should be doing, at the full pace.
     * @param elapsed_time Time since the last iteration.
     * @return The time scale, in (0, 1].
     */
    float find_next_time_scale(const Reference& reference, float elapsed_time) const;

    /**
     * @brief Find the largest time scale at which the feed forward fits in the voltage available.
     *
     * @note The feed forward of each wheel is a quadratic in the scale: the friction does not scale,
     * the speed term scales with it and the acceleration term with its square. So the scale is the
     * largest root in (0, 1] of the wheels reaching the limit, or one if they never do.
     *
     * @param reference What the robot should be doing, at the full pace.
     * @return The time scale, in (0, 1].
     */
    float find_time_scale(const Reference& reference) const;

    /**
     * @brief Get the terms of the feed forward of each wheel, left then right.
     *
     * @note Each wheel has four terms, in volts: the static friction, which does not scale, the
     * speed term, which scales with the time scale, the acceleration term, which scales with its
     * square, and the term of the rate of the time scale, which is the acceleration a change of the
     * scale adds, per unit of that rate.
     *
     * @param reference What the robot should be doing, at the full pace.
     * @return The terms of the feed forward of each wheel.
     */
    std::array<std::array<float, 4>, 2> get_wheel_terms(const Reference& reference) const;

    /**
     * @brief Get the voltage the feed forward may use, with the reserve for the feedback left out.
     *
     * @return The voltage available.
     */
    float get_available_voltage() const;

    /**
     * @brief Parameters of the controller, with the physical description of the robot.
     */
    Config config;

    /**
     * @brief Feedback gains of the forward axis.
     */
    Gains linear_gains;

    /**
     * @brief Feedback gains of the rotation axis.
     */
    Gains angular_gains;

    /**
     * @brief Factor to apply to the elapsed time of the reference.
     */
    float time_scale{1.0F};

    /**
     * @brief Smallest time scale, which is only reached if the friction alone takes more than the
     * voltage available.
     */
    static constexpr float min_time_scale{0.1F};

    /**
     * @brief Errors and terms of the last update.
     */
    Status status{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_CONTROLLER_HPP
