/**
 * @file
 */

#ifndef MICRAS_NAV_DRIVE_IDENTIFICATION_HPP
#define MICRAS_NAV_DRIVE_IDENTIFICATION_HPP

#include <array>
#include <cstdint>

#include "micras/nav/controller.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/robot_model.hpp"

namespace micras::nav {
/**
 * @brief Procedure that measures the constants of the drive train on the robot itself.
 *
 * @details The robot is driven open loop through a fixed sequence: a slow ramp until the wheels
 * start to turn, a step forward and the same step backward, then a step of rotation to each side.
 * Every step has two levels, the full command and then a fraction of it, because the response to a
 * single level cannot tell the friction from the speed constant.
 * Each axis is modeled as `u = k_s * sign(v) + k_v * v + k_a * dv/dt`, and the three constants are
 * fitted to the whole response by least squares. The model is fitted in its integrated form,
 * `integral(u) = k_s * integral(sign(v)) + k_v * x + k_a * (v - v0)`, which uses the position and
 * the speed as they are measured and never differentiates a noisy signal.
 *
 * The constants come out in volts, and get_model() turns them into the physical constants of the
 * robot model, so the result of a run is a set of numbers to type into the configuration.
 *
 * @note The robot travels forward by about the distance limit and comes back, so it needs that much
 * free floor ahead of it. It should run with the fan as it is used in a fast run.
 */
class DriveIdentification {
public:
    /**
     * @brief Configuration struct for the identification.
     *
     * @note The commands are in percent of the motor supply. The ramp rate is in percent per
     * second, and the wheels count as turning once they are faster than the breakaway speed. Each
     * level of a step lasts for the step time, a step forward or backward also ends when the
     * distance limit is reached, and the robot rests for the rest time between two steps.
     */
    struct Config {
        RobotModel model;
        float      ramp_rate;
        float      breakaway_speed;
        float      linear_command;
        float      angular_command;
        float      step_time;
        float      rest_time;
        float      max_distance;
    };

    /**
     * @brief Constants of one axis, in volts per unit of speed and of acceleration.
     */
    struct Axis {
        float static_friction;
        float speed_constant;
        float acceleration_constant;
    };

    /**
     * @brief Construct a new Drive Identification object.
     *
     * @param config The configuration for the identification.
     */
    explicit DriveIdentification(const Config& config);

    /**
     * @brief Start the procedure.
     *
     * @param measurements The current measurements.
     */
    void start(const Measurements& measurements);

    /**
     * @brief Advance the procedure by one iteration.
     *
     * @param measurements The current measurements.
     * @param elapsed_time Time since the last iteration, in seconds.
     * @return The command to apply to the locomotion.
     */
    Controller::Command update(const Measurements& measurements, float elapsed_time);

    /**
     * @brief Check if the procedure has ended.
     *
     * @return True once every step was driven and the constants were fitted.
     */
    bool is_finished() const;

    /**
     * @brief Check if the fit of both axes was well conditioned.
     *
     * @return True if the result can be trusted.
     */
    bool is_valid() const;

    /**
     * @brief Get the voltage at which the wheels started to turn.
     *
     * @return The breakaway voltage, per motor.
     */
    float get_breakaway_voltage() const;

    /**
     * @brief Get the constants fitted for the forward axis.
     *
     * @return The constants, in V, V*s/m and V*s^2/m.
     */
    const Axis& get_linear() const;

    /**
     * @brief Get the constants fitted for the rotation axis.
     *
     * @return The constants, in V, V*s/rad and V*s^2/rad.
     */
    const Axis& get_angular() const;

    /**
     * @brief Get the robot model with the drive constants replaced by the ones that were measured.
     *
     * @note The torque constant follows from the speed constant, the resistance from the
     * acceleration constant and the mass, and the yaw inertia from the angular acceleration
     * constant. The mass, the wheel radius, the track width and the gear ratio are taken as given.
     *
     * @return The model.
     */
    RobotModel get_model() const;

private:
    /**
     * @brief Command of the second level of a step, as a fraction of the first.
     */
    static constexpr float second_level{0.5F};

    /**
     * @brief Steps of the procedure.
     */
    enum class Phase : uint8_t {
        RAMP = 0,
        FORWARD = 1,
        BACKWARD = 2,
        LEFT = 3,
        RIGHT = 4,
        FINISHED = 5,
    };

    /**
     * @brief Accumulator of a least squares fit with three unknowns.
     */
    struct Fit {
        /**
         * @brief Add one sample of the integrated model.
         *
         * @param regressors The integral of the sign of the speed, the position and the speed.
         * @param response The integral of the voltage.
         */
        void add(const std::array<float, 3>& regressors, float response);

        /**
         * @brief Solve for the constants.
         *
         * @param axis The constants, which are only written if the problem is well conditioned.
         * @return True if the constants were found.
         */
        bool solve(Axis& axis) const;

        std::array<std::array<double, 3>, 3> normal{};
        std::array<double, 3>                moment{};
    };

    /**
     * @brief Move on to the next phase, after a rest.
     *
     * @param measurements The current measurements.
     */
    void advance(const Measurements& measurements);

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
     * @brief Time left to rest before the current step drives the motors.
     */
    float rest_left{};

    /**
     * @brief Wheel angles at the last iteration.
     */
    ///@{
    float last_left_angle{};
    float last_right_angle{};
    ///@}

    /**
     * @brief Integrals of the current step: voltage, sign of the speed and position.
     */
    ///@{
    float voltage_integral{};
    float sign_integral{};
    float position{};
    ///@}

    /**
     * @brief Speed at the start of the current step.
     */
    float start_speed{};

    /**
     * @brief Distance traveled forward since the procedure started.
     */
    float distance{};

    /**
     * @brief Voltage at which the wheels started to turn.
     */
    float breakaway_voltage{};

    /**
     * @brief Fit of each axis.
     */
    ///@{
    Fit linear_fit{};
    Fit angular_fit{};
    ///@}

    /**
     * @brief Constants fitted for each axis.
     */
    ///@{
    Axis linear{};
    Axis angular{};
    ///@}

    /**
     * @brief Whether both fits were well conditioned.
     */
    bool valid{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_DRIVE_IDENTIFICATION_HPP
