/**
 * @file
 */

#ifndef MICRAS_CORE_PID_CONTROLLER_HPP
#define MICRAS_CORE_PID_CONTROLLER_HPP

namespace micras::core {
/**
 * @brief Implementation of simple PID controller.
 *
 * @details The response is `kp * (error + ki * integral(error) - kd * d/dt(state))`. The derivative
 * acts on the controlled variable and not on the error, so a step of the setpoint does not kick it.
 */
class PidController {
public:
    /**
     * @brief Configuration struct for the PID controller.
     *
     * @note A negative saturation or maximum integral disables the respective limit. The monitored
     * flag makes the controller publish its terms to the file scope variables of its source file,
     * where a variable monitor can read them. There is one set of those for every instance, so only
     * one controller at a time should have it set.
     */
    struct Config {
        float kp{};
        float ki{};
        float kd{};
        float setpoint{};
        float saturation{-1.0F};
        float max_integral{-1.0F};
        bool  monitored{false};
    };

    /**
     * @brief Construct a new Pid Controller object.
     *
     * @param config Controller parameters.
     */
    explicit PidController(Config config);

    /**
     * @brief Set the desired setpoint.
     *
     * @param setpoint Desired state.
     */
    void set_setpoint(float setpoint);

    /**
     * @brief Reset prev_error and error_acc objects.
     */
    void reset();

    /**
     * @brief Update PID with new state and return response.
     *
     * @param state Current value of the controlled variable.
     * @param elapsed_time Time since the last update.
     * @return Response of the controller.
     */
    float compute_response(float state, float elapsed_time);

    /**
     * @brief Update PID with new state and return response.
     *
     * @param state Current value of the controlled variable.
     * @param elapsed_time Time since the last update.
     * @param state_change Derivative of the controlled variable.
     * @return Response of the controller.
     */
    float compute_response(float state, float elapsed_time, float state_change);

private:
    /**
     * @brief Proportional constant.
     */
    float kp;

    /**
     * @brief Integrative constant.
     */
    float ki;

    /**
     * @brief Derivative constant.
     */
    float kd;

    /**
     * @brief Desired state.
     */
    float setpoint;

    /**
     * @brief Maximum response returned by the controller.
     */
    float saturation;

    /**
     * @brief Maximum integrative response.
     */
    float max_integral;

    /**
     * @brief Whether the terms of the controller are published for a variable monitor.
     */
    bool monitored;

    /**
     * @brief Flag indicating whether this is the first run of the controller.
     */
    bool first_run = true;

    /**
     * @brief Accumulated error for integrative term.
     */
    float error_acc = 0;

    /**
     * @brief Previous state for derivative term.
     */
    float prev_state = 0;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_PID_CONTROLLER_HPP
