/**
 * @file
 */

#ifndef MICRAS_NAV_PID_CONTROLLER_HPP
#define MICRAS_NAV_PID_CONTROLLER_HPP

#include "micras/hal/timer.hpp"

namespace micras::nav {
/**
 * @brief Implementation of simple PID controller
 *        Response = Kp(error + Ki * integral(error) Kd * d/dt(error))
 */
class PidController {
public:
    struct Config {
        float kp{};                /**< Proportional constant */
        float ki{};                /**< Integrative constant */
        float kd{};                /**< Derivative constant */
        float setpoint{};          /**< Desired state */
        float saturation{-1.0F};   /**< Maximum response returned by the controller */
        float max_integral{-1.0F}; /**< Maximum integrative response */
    };

    /**
     * @brief Construct a new Pid Controller object
     *
     * @param config Controller parameters
     */
    explicit PidController(Config config);

    /**
     * @brief Set the setpoint object
     *
     * @param setpoint Desired state
     */
    void set_setpoint(float setpoint);

    /**
     * @brief Reset prev_error and error_acc objects
     */
    void reset();

    /**
     * @brief Update PID with new state and return response
     *
     * @param state Current value of the controlled variable
     * @param elapsed_time Time since the last update
     * @param save Whether to save the variables for calibration
     *
     * @return Response
     */
    float update(float state, float elapsed_time, bool save = false);

    /**
     * @brief Update PID with new state and return response
     *
     * @param state Current value of the controlled variable
     * @param state_change Derivative of the controlled variable
     * @param elapsed_time Time since the last update
     * @param save Whether to save the variables for calibration
     *
     * @return Response
     */
    float update(float state, float elapsed_time, float state_change, bool save = false);

private:
    float kp;                /**< Proportional constant */
    float ki;                /**< Integrative constant */
    float kd;                /**< Derivative constant */
    float setpoint;          /**< Desired state */
    float saturation;        /**< Maximum response returned by the controller */
    float max_integral;      /**< Maximum integrative accumulative response */
    float error_acc = 0;     /**< Accumulated error for i term */
    float prev_state = 0;    /**< Previous state for d term */
    float last_response = 0; /**< Last response returned by the controller */
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_PID_CONTROLLER_HPP
