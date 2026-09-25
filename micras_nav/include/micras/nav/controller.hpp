/**
 * @file
 */

#ifndef MICRAS_NAV_CONTROLLER_HPP
#define MICRAS_NAV_CONTROLLER_HPP

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
 * orientation the feedback aims at, and its rate to the angular speed, instead of being left for
 * an error across the path to build up.
 *
 * @note The gains are not tuned: they follow from the model of the drive train and from the natural
 * frequency and damping asked of each axis.
 */
class Controller {
public:
    /**
     * @brief Closed loop behavior asked of one axis.
     *
     * @note The natural frequency is in rad/s. The error is clamped before it is used, which bounds
     * what the feedback can ask for when the robot is held or has crashed.
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
     * static friction compensation goes from nothing to all of it.
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
     * @param reference What the robot should be doing, in the maze frame.
     * @param estimate What the robot is doing, as far as it is known.
     * @return The command for the locomotion.
     */
    Command update(const Reference& reference, const State& estimate);

    /**
     * @brief Get how much slower the reference has to be played for the motors to follow it.
     *
     * @note The feed forward alone may ask for more voltage than there is. Slowing the clock of
     * the reference scales its linear and angular speeds together, so the robot stays on the same
     * path and only takes longer, instead of cutting the turn it is in. With the limits of a run
     * derived from the same model this should stay at one. The reference this is computed from was
     * already played at the scale in force, so the scale is corrected from that one rather than
     * computed afresh, and settles where the demand meets the voltage available instead of
     * alternating between one and a fraction every iteration.
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
     * @brief Errors and terms of the last update.
     */
    Status status{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_CONTROLLER_HPP
