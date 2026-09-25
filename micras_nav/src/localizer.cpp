/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "micras/core/utils.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
Localizer::Localizer(const Config& config) : config{config}, rolling_radius{config.model.rolling_radius(0.0F)} {
    this->config.speed_window = std::clamp<uint8_t>(config.speed_window, 1, max_speed_window);

    this->set_covariance({
        config.initial_position_deviation * config.initial_position_deviation,
        config.initial_position_deviation * config.initial_position_deviation,
        config.initial_orientation_deviation * config.initial_orientation_deviation,
        config.initial_bias_deviation * config.initial_bias_deviation,
    });
}

void Localizer::reset(const Pose& pose, const Measurements& measurements) {
    const float bias_variance = this->get_variance(bias_index);

    this->state = {.pose = pose, .velocity = {}};
    this->last_left_angle = measurements.left_wheel_angle;
    this->last_right_angle = measurements.right_wheel_angle;

    this->distances.fill(0.0F);
    this->durations.fill(0.0F);
    this->window_distance = 0.0F;
    this->window_duration = 0.0F;
    this->status = {};

    this->set_covariance({
        this->config.initial_position_deviation * this->config.initial_position_deviation,
        this->config.initial_position_deviation * this->config.initial_position_deviation,
        this->config.initial_orientation_deviation * this->config.initial_orientation_deviation,
        bias_variance,
    });
}

void Localizer::predict(const Measurements& measurements, float elapsed_time) {
    const float radius = this->rolling_radius;

    const float left_distance = radius * (measurements.left_wheel_angle - this->last_left_angle);
    const float right_distance = radius * (measurements.right_wheel_angle - this->last_right_angle);

    this->last_left_angle = measurements.left_wheel_angle;
    this->last_right_angle = measurements.right_wheel_angle;

    const float distance = (left_distance + right_distance) / 2.0F;
    const float rate = this->config.model.gyroscope_scale * measurements.angular_rate - this->bias;
    const float rotation = rate * elapsed_time;

    const float half_rotation = rotation / 2.0F;
    const float squared = half_rotation * half_rotation;
    const float chord = distance * (1.0F - squared / 6.0F + squared * squared / 120.0F);

    const float heading = this->state.pose.orientation + half_rotation;
    const float cosine = std::cos(heading);
    const float sine = std::sin(heading);

    const float slide = -this->config.model.traction.lateral_compliance * distance * rate;

    this->state.pose.position.x += chord * cosine - slide * sine;
    this->state.pose.position.y += chord * sine + slide * cosine;
    this->state.pose.orientation = core::assert_angle(this->state.pose.orientation + rotation);

    this->window_distance += distance - this->distances.at(this->window_index);
    this->window_duration += elapsed_time - this->durations.at(this->window_index);
    this->distances.at(this->window_index) = distance;
    this->durations.at(this->window_index) = elapsed_time;
    this->window_index = static_cast<uint8_t>((this->window_index + 1) % this->config.speed_window);

    this->state.velocity = {
        .linear = this->window_duration > 0.0F ? this->window_distance / this->window_duration : 0.0F,
        .angular = rate,
    };

    Matrix transition{};

    for (uint8_t i = 0; i < number_of_states; i++) {
        transition.at(i).at(i) = 1.0F;
    }

    transition.at(x_index).at(orientation_index) = -chord * sine;
    transition.at(y_index).at(orientation_index) = chord * cosine;
    transition.at(x_index).at(bias_index) = chord * sine * elapsed_time / 2.0F;
    transition.at(y_index).at(bias_index) = -chord * cosine * elapsed_time / 2.0F;
    transition.at(orientation_index).at(bias_index) = -elapsed_time;

    Matrix noise_input{};
    noise_input.at(x_index) = {cosine, -sine, 0.0F, 0.0F};
    noise_input.at(y_index) = {sine, cosine, 0.0F, 0.0F};
    noise_input.at(orientation_index) = {0.0F, 0.0F, 1.0F, 0.0F};
    noise_input.at(bias_index) = {0.0F, 0.0F, 0.0F, 1.0F};

    const RobotModel::Noise& noise = this->config.model.noise;

    const float traveled = std::abs(distance);
    const float longitudinal =
        noise.longitudinal_slip + noise.longitudinal_slip_per_acceleration * std::abs(measurements.acceleration.x);

    this->time_since_imu += elapsed_time;
    float orientation_variance = 0.0F;

    if (measurements.imu_is_new) {
        orientation_variance = noise.gyroscope * noise.gyroscope * this->time_since_imu;
        this->time_since_imu = 0.0F;
    }

    this->propagate(
        transition, noise_input,
        {
            longitudinal * longitudinal * traveled,
            noise.lateral_slip * noise.lateral_slip * traveled,
            orientation_variance,
            noise.gyroscope_bias_walk * noise.gyroscope_bias_walk * elapsed_time,
        }
    );
}

void Localizer::correct_at_rest(const Measurements& measurements, float elapsed_time) {
    const float wheel_difference = measurements.right_wheel_angle - measurements.left_wheel_angle;

    if (std::abs(this->state.velocity.linear) > this->config.stationary_linear_speed) {
        this->rest_time = 0.0F;
        return;
    }

    if (this->rest_time <= 0.0F) {
        this->rest_rotation = 0.0F;
        this->rest_wheel_difference = wheel_difference;
    }

    this->rest_time += elapsed_time;
    this->rest_rotation += this->config.model.gyroscope_scale * measurements.angular_rate * elapsed_time;

    if (this->rest_time < this->config.rest_window) {
        return;
    }

    const RobotModel& model = this->config.model;

    const float wheel_to_rotation = model.chassis.wheel_radius / model.chassis.track_width;
    const float wheel_rotation = wheel_to_rotation * (wheel_difference - this->rest_wheel_difference);
    const float measured_bias = (this->rest_rotation - wheel_rotation) / this->rest_time;

    const float gyroscope_deviation = model.noise.gyroscope / std::sqrt(this->rest_time);
    const float wheel_deviation = 2.0F * wheel_to_rotation * model.noise.wheel_angle / this->rest_time;

    this->update(
        measured_bias - this->bias, {0.0F, 0.0F, 0.0F, 1.0F},
        gyroscope_deviation * gyroscope_deviation + wheel_deviation * wheel_deviation, this->config.stationary_gate,
        this->config.max_position_correction
    );

    this->rest_time = 0.0F;
}

const State& Localizer::get_state() const {
    return this->state;
}

const Pose& Localizer::get_pose() const {
    return this->state.pose;
}

GridPose Localizer::get_cell() const {
    return this->state.pose.to_grid(this->config.model.maze.cell_size);
}

float Localizer::get_gyroscope_bias() const {
    return this->bias;
}

float Localizer::get_position_deviation() const {
    return std::sqrt(std::max(this->get_variance(x_index), this->get_variance(y_index)));
}

float Localizer::get_orientation_deviation() const {
    return std::sqrt(this->get_variance(orientation_index));
}

const Localizer::Status& Localizer::get_status() const {
    return this->status;
}

void Localizer::set_covariance(const Vector& variances) {
    this->upper = {};

    for (uint8_t i = 0; i < number_of_states; i++) {
        this->upper.at(i).at(i) = 1.0F;
    }

    this->diagonal = variances;
}

void Localizer::propagate(const Matrix& transition, const Matrix& noise_input, const Vector& noise_variances) {
    static constexpr uint8_t columns{2 * number_of_states};

    std::array<std::array<float, columns>, number_of_states> work{};
    std::array<float, columns>                               weights{};

    for (uint8_t i = 0; i < number_of_states; i++) {
        for (uint8_t j = 0; j < number_of_states; j++) {
            float sum = 0.0F;

            for (uint8_t k = 0; k <= j; k++) {
                sum += transition.at(i).at(k) * this->upper.at(k).at(j);
            }

            work.at(i).at(j) = sum;
            work.at(i).at(number_of_states + j) = noise_input.at(i).at(j);
        }

        weights.at(i) = this->diagonal.at(i);
        weights.at(number_of_states + i) = noise_variances.at(i);
    }

    for (uint8_t j = number_of_states; j-- > 0;) {
        float sum = 0.0F;

        for (uint8_t k = 0; k < columns; k++) {
            sum += work.at(j).at(k) * work.at(j).at(k) * weights.at(k);
        }

        this->diagonal.at(j) = sum;

        for (uint8_t i = 0; i < j; i++) {
            float cross = 0.0F;

            for (uint8_t k = 0; k < columns; k++) {
                cross += work.at(i).at(k) * work.at(j).at(k) * weights.at(k);
            }

            const float factor = sum > 0.0F ? cross / sum : 0.0F;
            this->upper.at(i).at(j) = factor;

            for (uint8_t k = 0; k < columns; k++) {
                work.at(i).at(k) -= factor * work.at(j).at(k);
            }
        }
    }
}

bool Localizer::update(
    float innovation, const Vector& jacobian, float variance, float gate, float max_position_correction
) {
    Vector projected{};
    Vector scaled{};

    float innovation_variance = variance;

    for (uint8_t j = 0; j < number_of_states; j++) {
        float sum = jacobian.at(j);

        for (uint8_t i = 0; i < j; i++) {
            sum += this->upper.at(i).at(j) * jacobian.at(i);
        }

        projected.at(j) = sum;
        scaled.at(j) = this->diagonal.at(j) * sum;
        innovation_variance += sum * scaled.at(j);
    }

    const float normalized = innovation * innovation / innovation_variance;

    if (normalized > gate) {
        this->status.rejected++;
        return false;
    }

    this->status.accepted++;
    this->status.innovation_level += 0.01F * (normalized - this->status.innovation_level);

    Vector cross{};

    for (uint8_t i = 0; i < number_of_states; i++) {
        float sum = scaled.at(i);

        for (uint8_t j = i + 1; j < number_of_states; j++) {
            sum += this->upper.at(i).at(j) * scaled.at(j);
        }

        cross.at(i) = sum;
    }

    const float position_step = std::hypot(cross.at(x_index), cross.at(y_index)) * std::abs(innovation);
    const float orientation_step = std::abs(cross.at(orientation_index) * innovation);

    const float required = std::max(
        {innovation_variance, position_step / max_position_correction,
         orientation_step / this->config.max_orientation_correction}
    );

    float  alpha = variance + (required - innovation_variance);
    Vector gain{};

    for (uint8_t j = 0; j < number_of_states; j++) {
        const float beta = alpha;
        alpha += projected.at(j) * scaled.at(j);

        const float lambda = -projected.at(j) / beta;
        this->diagonal.at(j) *= beta / alpha;
        gain.at(j) = scaled.at(j);

        for (uint8_t i = 0; i < j; i++) {
            const float previous = this->upper.at(i).at(j);
            this->upper.at(i).at(j) = previous + gain.at(i) * lambda;
            gain.at(i) += scaled.at(j) * previous;
        }
    }

    const float scale = innovation / alpha;

    this->state.pose.position.x += gain.at(x_index) * scale;
    this->state.pose.position.y += gain.at(y_index) * scale;
    this->state.pose.orientation =
        core::assert_angle(this->state.pose.orientation + gain.at(orientation_index) * scale);
    this->bias += gain.at(bias_index) * scale;

    return true;
}

void Localizer::set_downforce(float downforce) {
    this->rolling_radius = this->config.model.rolling_radius(downforce);
}

float Localizer::get_variance(uint8_t index) const {
    float variance = this->diagonal.at(index);

    for (uint8_t j = index + 1; j < number_of_states; j++) {
        variance += this->upper.at(index).at(j) * this->upper.at(index).at(j) * this->diagonal.at(j);
    }

    return variance;
}

bool Localizer::is_stationary() const {
    return std::abs(this->state.velocity.linear) < this->config.stationary_linear_speed and
           std::abs(this->state.velocity.angular) < this->config.stationary_angular_speed;
}
}  // namespace micras::nav
