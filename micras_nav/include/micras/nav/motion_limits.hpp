/**
 * @file
 */

#ifndef MICRAS_NAV_MOTION_LIMITS_HPP
#define MICRAS_NAV_MOTION_LIMITS_HPP

#include "micras/nav/robot_model.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
/**
 * @brief How a run trades time for safety.
 *
 * @note The utilization is the fraction of the available traction the run may ask for, in (0, 1].
 * Whether the run is risky selects the turn table built with the smaller safety margin. The speed
 * limit is what makes the search run slow: a fast run leaves it above what the robot can reach.
 */
struct RunProfile {
    bool  diagonal;
    bool  fan;
    bool  risky;
    float utilization;
    float max_speed;
};

/**
 * @brief Limits of a motion along one axis, linear or angular.
 *
 * @details The acceleration is bounded by the tires up to the speed where the motors take over, and
 * from there it falls linearly with the speed, reaching zero at the free running speed, which is
 * how a DC motor fed from a fixed voltage behaves. Braking is not limited by the motors, since the
 * back EMF helps it.
 */
struct MotionLimits {
    /**
     * @brief Get the largest acceleration available at a speed.
     *
     * @param speed The current speed.
     * @return The acceleration limit at that speed.
     */
    float acceleration_at(float speed) const;

    /**
     * @brief Get the speed from which the motors, rather than the tires, limit the acceleration.
     *
     * @return The crossover speed, which is negative if the motors limit it from rest.
     */
    float crossover_speed() const;

    float max_speed;
    float acceleration;
    float deceleration;
    float motor_acceleration;
    float motor_speed;
};

/**
 * @brief Derivation of the limits of a run from the physical model of the robot.
 */
class Dynamics {
public:
    /**
     * @brief Configuration struct for the dynamics.
     *
     * @note The speed limits are not physical: they bound what the sensing and the control have
     * been proven at, and only matter where the physics would allow more. The voltage reserve is
     * the fraction of the motor supply kept for the feedback and for steering, which the planned
     * accelerations may not use.
     */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init) no defaults, so that a missing field is a warning
    struct Config {
        RobotModel model;
        TurnTable  turns;
        TurnTable  risky_turns;
        float      max_linear_speed;
        float      max_angular_speed;
        float      voltage_reserve;
    };

    /**
     * @brief Construct a new Dynamics object.
     *
     * @param config The configuration for the dynamics.
     */
    explicit Dynamics(const Config& config);

    /**
     * @brief Get the limits of the motion along a straight.
     *
     * @param profile The profile of the run.
     * @return The linear limits in m/s and m/s^2.
     */
    MotionLimits get_linear_limits(const RunProfile& profile) const;

    /**
     * @brief Get the limits of a rotation in place.
     *
     * @param profile The profile of the run.
     * @return The angular limits in rad/s and rad/s^2.
     */
    MotionLimits get_angular_limits(const RunProfile& profile) const;

    /**
     * @brief Get the shape of a turn for a run.
     *
     * @param profile The profile of the run.
     * @param turn The turn.
     * @return The shape of the turn.
     */
    const TurnShape& get_turn(const RunProfile& profile, TurnId turn) const;

    /**
     * @brief Get the speed a turn is driven at.
     *
     * @details The largest speed at which neither the lateral acceleration at the peak curvature
     * nor the angular acceleration along the ramps exceeds what the tires can give to this run.
     *
     * @param profile The profile of the run.
     * @param turn The turn.
     * @return The linear speed along the turn in m/s.
     */
    float get_turn_speed(const RunProfile& profile, TurnId turn) const;

    /**
     * @brief Get the physical model of the robot.
     *
     * @return The model.
     */
    const RobotModel& get_model() const;

private:
    /**
     * @brief Physical description of the robot.
     */
    RobotModel model;

    /**
     * @brief Shape of the turns with the normal safety margin.
     */
    TurnTable turns;

    /**
     * @brief Shape of the turns with the reduced safety margin.
     */
    TurnTable risky_turns;

    /**
     * @brief Largest linear speed planned, in m/s.
     */
    float max_linear_speed;

    /**
     * @brief Largest angular speed planned for a rotation in place, in rad/s.
     */
    float max_angular_speed;

    /**
     * @brief Voltage the planned motions may count on.
     */
    float available_voltage;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MOTION_LIMITS_HPP
