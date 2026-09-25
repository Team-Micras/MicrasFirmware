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
 * Whether the run is risky selects the turn table built with the smaller safety margin. The racing
 * line replaces the route by the smoothest line through
 * its cells, which only a fast run planned with the robot stopped can drive. The speed limit is what
 * makes the search run slow: a fast run leaves it above what the robot can reach.
 */
struct RunProfile {
    bool  racing_line;
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
 * @brief Limits of a motion along a curve, where the tires share their grip between turning and
 * changing speed.
 *
 * @details The speed at a point is limited by the lateral acceleration its curvature asks for and
 * by the angular acceleration its sharpness asks for, each against what the tires can give, which
 * is the rule a turn has always been driven by. What is left of the grip there can change the
 * speed: the lateral part takes its share as on a friction circle, and the angular part, which the
 * two tires make by pushing in opposite directions, takes it from the push each tire has left. A
 * change of speed on a curve also changes the angular speed, and that angular acceleration is
 * counted too.
 *
 * @note On a straight nothing is taken and the limits are those of the linear motion.
 */
struct CurveLimits {
    /**
     * @brief Get the largest speed at a point.
     *
     * @param bending How the path bends there.
     * @return The speed in m/s.
     */
    float get_speed_limit(const Bending& bending) const;

    /**
     * @brief Get the largest acceleration at a point, at a speed.
     *
     * @param speed The speed there.
     * @param bending How the path bends there.
     * @return The acceleration along the path in m/s^2, which is zero where the grip is all used.
     */
    float get_acceleration(float speed, const Bending& bending) const;

    /**
     * @brief Get the largest deceleration at a point, at a speed.
     *
     * @param speed The speed there.
     * @param bending How the path bends there.
     * @return The deceleration along the path in m/s^2, as a positive number.
     */
    float get_deceleration(float speed, const Bending& bending) const;

    MotionLimits linear;
    float        lateral;
    float        angular;
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
     * @brief Get the limits of a motion along a curve.
     *
     * @param profile The profile of the run.
     * @return The limits, the linear ones being those of a straight.
     */
    CurveLimits get_curve_limits(const RunProfile& profile) const;

    /**
     * @brief Get the shape of a turn for a run.
     *
     * @param profile The profile of the run.
     * @param turn The turn.
     * @return The shape of the turn.
     */
    const TurnShape& get_turn(const RunProfile& profile, TurnId turn) const;

    /**
     * @brief Get the speed a turn is priced at.
     *
     * @details The largest speed at which neither the lateral acceleration at the peak curvature
     * nor the angular acceleration along the ramps exceeds what the tires can give to this run. It
     * is the slowest point of the turn, which the robot may drive faster where the turn allows.
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
