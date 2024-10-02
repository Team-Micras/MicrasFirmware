/**
 * @file pid_controller.hpp
 *
 * @brief Nav Pid Controller class declaration
 *
 * @date 10/2024
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
     *
     * @return Response
     */
    float update(float state);

    /**
     * @brief Update PID with new state and return response
     *
     * @param state Current value of the controlled variable
     * @param state_change Derivative of the controlled variable
     *
     * @return Response
     */
    float update(float state, float state_change);

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

    hal::Timer timer; /**< Timer used to compute the loop time */
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_PID_CONTROLLER_HPP
