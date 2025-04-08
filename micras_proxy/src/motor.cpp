/**
 * @file
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/proxy/motor.hpp"

namespace micras::proxy {
Motor::Motor(const Config& config) :
    backwards_pwm{config.backwards_pwm},
    forward_pwm{config.forward_pwm},
    max_stopped_command{config.max_stopped_command},
    deadzone{config.deadzone} {
    this->set_command(0.0F);
}

void Motor::set_command(float command) {
    const bool is_positive = (command >= 0.0F);
    command = std::abs(command);

    if (command <= max_stopped_command) {
        command = 0.0F;
    } else {
        command = core::remap(command, 0.0F, 100.0F, this->deadzone, 100.0F);
    }

    if (is_positive) {
        this->forward_pwm.set_duty_cycle(command);
        this->backwards_pwm.set_duty_cycle(0.0F);
    } else {
        this->forward_pwm.set_duty_cycle(0.0F);
        this->backwards_pwm.set_duty_cycle(command);
    }
}
}  // namespace micras::proxy
