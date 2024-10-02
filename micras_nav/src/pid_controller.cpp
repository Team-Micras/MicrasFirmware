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

float PidController::update(float state) {
    float loop_time = this->timer.elapsed_time_us() / 1000000.0F;

    if (loop_time == 0) {
        return this->last_response;
    }

    float state_change = (state - this->prev_state) / loop_time;

    return this->update(state, state_change);
}

float PidController::update(float state, float state_change) {
    float loop_time = this->timer.elapsed_time_us() / 1000000.0F;

    if (loop_time == 0) {
        return this->last_response;
    }

    float error = this->setpoint - state;
    this->prev_state = state;

    float response = this->kp * (error + this->ki * this->error_acc + this->kd * state_change);

    if (this->saturation < 0 or std::abs(response) < this->saturation or
        (this->error_acc != 0 and std::signbit(this->error_acc) != std::signbit(error))) {
        this->error_acc += error * loop_time;
    }

    if (this->max_integral >= 0 and this->ki > 0) {
        this->error_acc = std::clamp(
            this->error_acc, -this->max_integral / (this->kp * this->ki), this->max_integral / (this->kp * this->ki)
        );
    }

    response = this->kp * (error + this->ki * this->error_acc + this->kd * state_change);

    if (this->saturation >= 0 and std::abs(response) >= this->saturation) {
        response = std::clamp(response, -this->saturation, this->saturation);
    }

    this->last_response = response;
    this->timer.reset_us();

    return response;
}
}  // namespace micras::nav
