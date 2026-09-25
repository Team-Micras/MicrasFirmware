/**
 * @file
 */

#include <algorithm>
#include <cmath>

#include "micras/nav/controller.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
Controller::Controller(const Config& config) :
    config{config},
    linear_gains{compute_gains(config.linear, config.model.speed_constant(), config.model.acceleration_constant())},
    angular_gains{compute_gains(
        config.angular, config.model.angular_speed_constant(), config.model.angular_acceleration_constant()
    )} { }

Controller::Command Controller::update(const Reference& reference, const State& estimate) {
    const RobotModel& model = this->config.model;

    const float half_track = model.chassis.track_width / 2.0F;
    const float left_speed = reference.twist.linear - reference.twist.angular * half_track;
    const float right_speed = reference.twist.linear + reference.twist.angular * half_track;

    const float left_friction = std::clamp(left_speed / this->config.friction_speed, -1.0F, 1.0F);
    const float right_friction = std::clamp(right_speed / this->config.friction_speed, -1.0F, 1.0F);

    const float forward_feed_forward = model.speed_constant() * reference.twist.linear +
                                       model.acceleration_constant() * reference.acceleration.linear +
                                       model.drive.static_friction_voltage * (right_friction + left_friction) / 2.0F;

    const float rotation_feed_forward = model.angular_speed_constant() * reference.twist.angular +
                                        model.angular_acceleration_constant() * reference.acceleration.angular +
                                        model.drive.static_friction_voltage * (right_friction - left_friction) / 2.0F;

    const float available = model.drive.supply_voltage * (1.0F - this->config.voltage_reserve);
    const float demand = std::abs(forward_feed_forward) + std::abs(rotation_feed_forward);

    this->time_scale = demand > 0.0F ? std::min(1.0F, this->time_scale * available / demand) : 1.0F;

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

    const float to_percent = 100.0F / model.drive.supply_voltage;

    return {
        .forward = to_percent * (forward_feed_forward + forward_feedback),
        .rotation = to_percent * (rotation_feed_forward + rotation_feedback),
    };
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
