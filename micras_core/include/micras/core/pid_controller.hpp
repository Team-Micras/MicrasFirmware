/**
 * @file
 */

#ifndef MICRAS_CORE_PID_CONTROLLER_HPP
#define MICRAS_CORE_PID_CONTROLLER_HPP

namespace micras::core {
/**
 * @brief Implementation of simple PID controller. Response = kp * (error + ki * integral(error) kd * d/dt(error))
 */
class PidController {
public:
    /**
     * @brief Configuration struct for the PID controller.
     */
    struct Config {
        float kp{};
        float ki{};
        float kd{};
        float setpoint{};
        float saturation{-1.0F};
        float max_integral{-1.0F};
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
     * @param save Whether to save the variables for calibration.
     * @return Response of the controller.
     */
    float compute_response(float state, float elapsed_time, bool save = false);

    /**
     * @brief Update PID with new state and return response.
     *
     * @param state Current value of the controlled variable.
     * @param state_change Derivative of the controlled variable.
     * @param elapsed_time Time since the last update.
     * @param save Whether to save the variables for calibration.
     * @return Response of the controller.
     */
    float compute_response(float state, float elapsed_time, float state_change, bool save = false);

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
     * @brief Flag indicating whether this is the first run of the controller.
     */
    bool first_run = true;

    /**
     * @brief Accumulated error for integrative term.
     */
    float error_acc = 0;

    /**
     * @brief Previous error for derivative term.
     */
    float prev_state = 0;

    /**
     * @brief Last response returned by the controller.
     */
    float last_response = 0;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_PID_CONTROLLER_HPP
