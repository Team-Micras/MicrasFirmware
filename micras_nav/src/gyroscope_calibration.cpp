/**
 * @file
 */

#include <cmath>
#include <cstdint>
#include <numbers>

#include "micras/core/vector.hpp"
#include "micras/nav/gyroscope_calibration.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
GyroscopeCalibration::GyroscopeCalibration(const Config& config) : config{config} { }

void GyroscopeCalibration::start(const Pose& pose, const MotionLimits& limits) {
    this->phase = Phase::BEFORE;
    this->phase_time = 0.0F;
    this->pose = pose;
    this->spin = SpeedProfile{2.0F * std::numbers::pi_v<float> * this->config.turns, 0.0F, 0.0F, limits};
    this->angle_sum = 0.0F;
    this->angle_count = 0;
    this->raw_rotation = 0.0F;
    this->turning_time = 0.0F;
    this->valid = true;
}

Reference GyroscopeCalibration::update(const Measurements& measurements, float bias, float elapsed_time) {
    Reference reference{.pose = this->pose, .twist = {}, .acceleration = {}, .distance = 0.0F};

    if (this->phase == Phase::FINISHED) {
        return reference;
    }

    this->phase_time += elapsed_time;

    if (this->phase == Phase::TURNING) {
        const SpeedProfile::Sample sample = this->spin.sample(this->phase_time);

        this->raw_rotation += measurements.angular_rate * elapsed_time;
        this->turning_time += elapsed_time;

        reference.pose.orientation += sample.distance;
        reference.twist.angular = sample.speed;
        reference.acceleration.angular = sample.acceleration;
        reference.distance = sample.distance;

        if (this->phase_time >= this->spin.duration()) {
            this->phase = Phase::AFTER;
            this->phase_time = 0.0F;
        }

        return reference;
    }

    if (this->phase_time >= this->config.settle_time / 2.0F) {
        const WallReading& left = measurements.walls.at(this->config.left_sensor);
        const WallReading& right = measurements.walls.at(this->config.right_sensor);

        if (left.valid and right.valid) {
            this->angle_sum += this->get_wall_angle(measurements);
            this->angle_count++;
        }
    }

    if (this->phase_time < this->config.settle_time) {
        return reference;
    }

    const float angle = this->angle_count > 0 ? this->angle_sum / static_cast<float>(this->angle_count) : 0.0F;

    this->valid = this->valid and this->angle_count > 0;
    this->angle_sum = 0.0F;
    this->angle_count = 0;
    this->phase_time = 0.0F;

    if (this->phase == Phase::BEFORE) {
        this->angle_before = angle;
        this->bias = bias;
        this->phase = Phase::TURNING;
        return reference;
    }

    const float rotation = this->spin.distance() + angle - this->angle_before;

    if (std::abs(this->raw_rotation) > 0.0F) {
        this->scale = (rotation + this->bias * this->turning_time) / this->raw_rotation;
    } else {
        this->valid = false;
    }

    this->phase = Phase::FINISHED;

    return reference;
}

bool GyroscopeCalibration::is_finished() const {
    return this->phase == Phase::FINISHED;
}

bool GyroscopeCalibration::is_valid() const {
    return this->valid;
}

float GyroscopeCalibration::get_scale() const {
    return this->scale;
}

float GyroscopeCalibration::get_wall_angle(const Measurements& measurements) const {
    const auto get_point = [this, &measurements](uint8_t sensor) {
        const RobotModel::WallSensor& mounting = this->config.model.wall_sensors.at(sensor);
        const float                   range = measurements.walls.at(sensor).distance;

        return core::Vector{
            .x = mounting.position.x + range * std::cos(mounting.angle),
            .y = mounting.position.y + range * std::sin(mounting.angle),
        };
    };

    const core::Vector along = get_point(this->config.left_sensor) - get_point(this->config.right_sensor);

    return std::atan2(along.x, along.y);
}
}  // namespace micras::nav
