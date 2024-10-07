/**
 * @file pid_controller.cpp
 *
 * @brief Nav Pid Controller class implementation
 *
 * @date 10/2024
 */

#include <algorithm>
#include <cmath>

#include "micras/nav/pid_controller.hpp"

static volatile float test_error;
static volatile float test_proportional;
static volatile float test_integrative;
static volatile float test_derivative;
static volatile float test_response;

namespace micras::nav {
PidController::PidController(Config config) :
    kp{config.kp},
    ki{config.ki},
    kd{config.kd},
    setpoint{config.setpoint},
    saturation{config.saturation},
    max_integral{config.max_integral} { }

void PidController::set_setpoint(float setpoint) {
    this->setpoint = setpoint;
}

void PidController::reset() {
    this->error_acc = 0;
    this->prev_state = 0;
    this->last_response = 0;
}

float PidController::update(float state, float elapsed_time, bool save) {
    float state_change = (state - this->prev_state) / elapsed_time;
    return this->update(state, elapsed_time, state_change, save);
}

float PidController::update(float state, float elapsed_time, float state_change, bool save) {
    float error = this->setpoint - state;
    this->prev_state = state;

    float response = this->kp * (error + this->ki * this->error_acc - this->kd * state_change);

    if (this->saturation < 0 or std::abs(response) < this->saturation or
        (this->error_acc != 0 and std::signbit(this->error_acc) != std::signbit(error))) {
        this->error_acc += error * elapsed_time;
    }

    if (this->max_integral >= 0 and this->ki > 0) {
        this->error_acc = std::clamp(
            this->error_acc, -this->max_integral / (this->kp * this->ki), this->max_integral / (this->kp * this->ki)
        );
    }

    response = this->kp * (error + this->ki * this->error_acc - this->kd * state_change);

    if (this->saturation >= 0 and std::abs(response) >= this->saturation) {
        response = std::clamp(response, -this->saturation, this->saturation);
    }

    this->last_response = response;

    if (save) {
        test_error = error;
        test_proportional = this->kp * error;
        test_integrative = this->kp * this->ki * this->error_acc;
        test_derivative = -this->kp * this->kd * state_change;
        test_response = response;
    }

    return response;
}
}  // namespace micras::nav
