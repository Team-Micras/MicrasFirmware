/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "micras/nav/controller.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Find the real roots of a quadratic, in the form that keeps its precision.
 *
 * @param quadratic The coefficient of the square.
 * @param linear The coefficient of the variable.
 * @param constant The constant term.
 * @return The roots, not a number where there is none, and only the first one of a linear equation.
 */
static std::array<float, 2> solve_quadratic(float quadratic, float linear, float constant) {
    constexpr float none = std::numeric_limits<float>::quiet_NaN();

    if (std::abs(quadratic) < 1.0e-9F) {
        return {std::abs(linear) > 0.0F ? -constant / linear : none, none};
    }

    const float discriminant = linear * linear - 4.0F * quadratic * constant;

    if (discriminant < 0.0F) {
        return {none, none};
    }

    const float half = -0.5F * (linear + std::copysign(std::sqrt(discriminant), linear));

    return {half / quadratic, std::abs(half) > 0.0F ? constant / half : none};
}

Controller::Controller(const Config& config) :
    config{config},
    linear_gains{compute_gains(config.linear, config.model.speed_constant(), config.model.acceleration_constant())},
    angular_gains{compute_gains(
        config.angular, config.model.angular_speed_constant(), config.model.angular_acceleration_constant()
    )} { }

void Controller::reset() {
    this->time_scale = 1.0F;
    this->saturated = false;
}

Controller::Command Controller::update(const Reference& unscaled, const State& estimate, float elapsed_time) {
    const RobotModel& model = this->config.model;

    const float scale = this->find_next_time_scale(unscaled, elapsed_time);
    const float scale_rate = (scale - this->time_scale) / elapsed_time;

    this->time_scale = scale;

    Reference reference = unscaled;
    reference.twist.linear = scale * unscaled.twist.linear;
    reference.twist.angular = scale * unscaled.twist.angular;
    reference.acceleration.linear = scale * scale * unscaled.acceleration.linear + scale_rate * unscaled.twist.linear;
    reference.acceleration.angular =
        scale * scale * unscaled.acceleration.angular + scale_rate * unscaled.twist.angular;

    const Command feed_forward = this->get_feed_forward(reference.twist, reference.acceleration);
    const float   forward_feed_forward = feed_forward.forward;
    const float   rotation_feed_forward = feed_forward.rotation;

    const Pose seen = reference.pose.relative(estimate.pose);

    const float along_error =
        std::clamp(-seen.position.x, -this->config.linear.max_error, this->config.linear.max_error);
    const float blend = std::clamp(reference.twist.linear / this->config.steering_blend_speed, -1.0F, 1.0F);
    const float steering =
        blend * std::clamp(
                    -this->config.steering_gain * seen.position.y, -this->config.max_steering, this->config.max_steering
                );
    const float slide_angle = model.traction.lateral_compliance * reference.twist.angular;
    const float slide_rate = model.traction.lateral_compliance * reference.acceleration.angular;
    const float orientation_error = std::clamp(
        -seen.orientation + slide_angle + steering, -this->config.angular.max_error, this->config.angular.max_error
    );

    const float forward_feedback = this->linear_gains.proportional * along_error +
                                   this->linear_gains.derivative * (reference.twist.linear - estimate.velocity.linear);

    const float rotation_feedback =
        this->angular_gains.proportional * orientation_error +
        this->angular_gains.derivative * (reference.twist.angular + slide_rate - estimate.velocity.angular);

    this->status = {
        .along_error = -seen.position.x,
        .across_error = -seen.position.y,
        .orientation_error = -seen.orientation,
        .forward_feed_forward = forward_feed_forward,
        .rotation_feed_forward = rotation_feed_forward,
        .forward_feedback = forward_feedback,
        .rotation_feedback = rotation_feedback,
    };

    return this->to_command(feed_forward, {.forward = forward_feedback, .rotation = rotation_feedback});
}

Controller::Command Controller::follow_speed(const Twist& twist, const Twist& acceleration, const State& estimate) {
    const Command feed_forward = this->get_feed_forward(twist, acceleration);
    const Command feedback{
        .forward = this->linear_gains.derivative * (twist.linear - estimate.velocity.linear),
        .rotation = this->angular_gains.derivative * (twist.angular - estimate.velocity.angular),
    };

    this->status = {
        .along_error = 0.0F,
        .across_error = 0.0F,
        .orientation_error = 0.0F,
        .forward_feed_forward = feed_forward.forward,
        .rotation_feed_forward = feed_forward.rotation,
        .forward_feedback = feedback.forward,
        .rotation_feedback = feedback.rotation,
    };

    return this->to_command(feed_forward, feedback);
}

Controller::Command Controller::get_feed_forward(const Twist& twist, const Twist& acceleration) const {
    const RobotModel& model = this->config.model;

    const float half_track = model.chassis.track_width / 2.0F;
    const float left_speed = twist.linear - twist.angular * half_track;
    const float right_speed = twist.linear + twist.angular * half_track;

    const float left_friction = std::clamp(left_speed / this->config.friction_speed, -1.0F, 1.0F);
    const float right_friction = std::clamp(right_speed / this->config.friction_speed, -1.0F, 1.0F);

    return {
        .forward = model.speed_constant() * twist.linear + model.acceleration_constant() * acceleration.linear +
                   model.drive.static_friction_voltage * (right_friction + left_friction) / 2.0F,
        .rotation = model.angular_speed_constant() * twist.angular +
                    model.angular_acceleration_constant() * acceleration.angular +
                    model.drive.static_friction_voltage * (right_friction - left_friction) / 2.0F,
    };
}

Controller::Command Controller::to_command(const Command& feed_forward, const Command& feedback) {
    const float to_percent = 100.0F / this->config.model.drive.supply_voltage;

    const Command command{
        .forward = to_percent * (feed_forward.forward + feedback.forward),
        .rotation = to_percent * (feed_forward.rotation + feedback.rotation),
    };

    this->saturated = std::abs(command.forward) + std::abs(command.rotation) > 100.0F;

    return command;
}

float Controller::find_next_time_scale(const Reference& reference, float elapsed_time) const {
    const float half_track = this->config.model.chassis.track_width / 2.0F;
    const float fastest_wheel = std::abs(reference.twist.linear) + std::abs(reference.twist.angular) * half_track;
    const float largest_change = this->config.max_time_scale_acceleration * elapsed_time;
    const float step = largest_change / std::max(fastest_wheel, largest_change);
    const float target = this->find_time_scale(reference);

    if (target <= this->time_scale) {
        return std::max(target, this->time_scale - step);
    }

    if (this->saturated) {
        return this->time_scale;
    }

    const float highest = std::min(target, this->time_scale + step);
    const float available = this->get_available_voltage();

    float rate = (highest - this->time_scale) / elapsed_time;

    for (const std::array<float, 4>& wheel : this->get_wheel_terms(reference)) {
        const float voltage = wheel.at(0) + highest * (wheel.at(1) + highest * wheel.at(2));
        const float rate_term = std::abs(wheel.at(3));

        if (rate_term > 0.0F) {
            rate = std::min(rate, std::max((available - std::copysign(1.0F, wheel.at(3)) * voltage) / rate_term, 0.0F));
        }
    }

    return this->time_scale + rate * elapsed_time;
}

float Controller::find_time_scale(const Reference& reference) const {
    const std::array<std::array<float, 4>, 2> wheels = this->get_wheel_terms(reference);
    const float                               available = this->get_available_voltage();

    const auto fits = [&wheels, available](float scale) {
        return std::ranges::all_of(wheels, [scale, available](const std::array<float, 4>& wheel) {
            return std::abs(wheel.at(0) + scale * (wheel.at(1) + scale * wheel.at(2))) <= available * 1.0001F;
        });
    };

    if (fits(1.0F)) {
        return 1.0F;
    }

    float best = min_time_scale;

    for (const std::array<float, 4>& wheel : wheels) {
        for (const float limit : {available, -available}) {
            for (const float root : solve_quadratic(wheel.at(2), wheel.at(1), wheel.at(0) - limit)) {
                if (root > best and root < 1.0F and fits(root)) {
                    best = root;
                }
            }
        }
    }

    return best;
}

std::array<std::array<float, 4>, 2> Controller::get_wheel_terms(const Reference& reference) const {
    const RobotModel& model = this->config.model;

    const float half_track = model.chassis.track_width / 2.0F;

    std::array<std::array<float, 4>, 2> wheels{};

    for (uint8_t i = 0; i < 2; i++) {
        const float side = i == 0 ? -1.0F : 1.0F;
        const float speed = reference.twist.linear + side * reference.twist.angular * half_track;

        wheels.at(i) = {
            model.drive.static_friction_voltage * std::clamp(speed / this->config.friction_speed, -1.0F, 1.0F),
            model.speed_constant() * reference.twist.linear +
                side * model.angular_speed_constant() * reference.twist.angular,
            model.acceleration_constant() * reference.acceleration.linear +
                side * model.angular_acceleration_constant() * reference.acceleration.angular,
            model.acceleration_constant() * reference.twist.linear +
                side * model.angular_acceleration_constant() * reference.twist.angular,
        };
    }

    return wheels;
}

float Controller::get_available_voltage() const {
    return this->config.model.drive.supply_voltage * (1.0F - this->config.voltage_reserve);
}

float Controller::get_time_scale() const {
    return this->time_scale;
}

const Controller::Status& Controller::get_status() const {
    return this->status;
}

Controller::Gains Controller::compute_gains(const Axis& axis, float speed_constant, float acceleration_constant) {
    return {
        .proportional = acceleration_constant * axis.natural_frequency * axis.natural_frequency,
        .derivative =
            std::max(2.0F * axis.damping * axis.natural_frequency * acceleration_constant - speed_constant, 0.0F),
    };
}
}  // namespace micras::nav
