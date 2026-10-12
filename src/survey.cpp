/**
 * @file
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <vector>

#include "micras/core/utils.hpp"
#include "micras/survey.hpp"

namespace micras {
Survey::Survey(const Config& config) : config{config} { }

void Survey::start(const nav::Pose& pose, const nav::MotionLimits& linear, const nav::MotionLimits& angular) {
    this->phase = Phase::SETTLE;
    this->phase_time = 0.0F;
    this->origin = pose;
    this->anchor = pose;
    this->back = nav::SpeedProfile{this->config.back_distance, 0.0F, 0.0F, linear};
    this->turn = nav::SpeedProfile{std::numbers::pi_v<float>, 0.0F, 0.0F, angular};
    this->ret = nav::SpeedProfile{this->config.return_distance, 0.0F, 0.0F, linear};
    this->last_travel = 0.0F;
    this->count = 0;
    this->page = 0;
}

nav::Reference Survey::update(
    const nav::Pose& pose, const std::array<float, nav::number_of_wall_sensors>& intensities, float elapsed_time
) {
    nav::Reference reference{.pose = this->anchor, .twist = {}, .acceleration = {}, .distance = 0.0F};

    if (this->phase == Phase::FINISHED) {
        return reference;
    }

    this->phase_time += elapsed_time;

    const auto drive = [this, &reference, &pose, &intensities](const nav::SpeedProfile& profile, uint8_t pass) {
        const nav::SpeedProfile::Sample sample = profile.sample(this->phase_time);

        reference.pose = this->anchor.compose({.position = {.x = -sample.distance, .y = 0.0F}, .orientation = 0.0F});
        reference.twist.linear = -sample.speed;
        reference.acceleration.linear = -sample.acceleration;
        reference.distance = sample.distance;

        if (std::abs(this->get_travel(pose) - this->last_travel) >= this->config.step) {
            this->record(pose, intensities, pass);
        }

        return this->phase_time >= profile.duration();
    };

    switch (this->phase) {
        case Phase::SETTLE:
            if (this->phase_time >= this->config.settle_time) {
                this->record(pose, intensities, 0);
                this->phase = Phase::BACK;
                this->phase_time = 0.0F;
            }

            break;

        case Phase::BACK:
            if (drive(this->back, 1)) {
                this->anchor = reference.pose;
                this->phase = Phase::PAUSE;
                this->phase_time = 0.0F;
            }

            break;

        case Phase::PAUSE:
            if (this->phase_time >= this->config.settle_time) {
                this->phase = Phase::TURN;
                this->phase_time = 0.0F;
            }

            break;

        case Phase::TURN: {
            const nav::SpeedProfile::Sample sample = this->turn.sample(this->phase_time);

            reference.pose.orientation = core::assert_angle(this->anchor.orientation + sample.distance);
            reference.twist.angular = sample.speed;
            reference.acceleration.angular = sample.acceleration;
            reference.distance = sample.distance;

            if (this->phase_time >= this->turn.duration()) {
                this->anchor = reference.pose;
                this->phase = Phase::REST;
                this->phase_time = 0.0F;
            }

            break;
        }

        case Phase::REST:
            if (this->phase_time >= this->config.settle_time) {
                this->last_travel = this->get_travel(pose);
                this->record(pose, intensities, 2);
                this->phase = Phase::RETURN;
                this->phase_time = 0.0F;
            }

            break;

        case Phase::RETURN:
            if (drive(this->ret, 2)) {
                this->anchor = reference.pose;
                this->phase = Phase::FINISHED;
                this->phase_time = 0.0F;
            }

            break;

        case Phase::FINISHED:
            break;
    }

    return reference;
}

bool Survey::is_finished() const {
    return this->phase == Phase::FINISHED;
}

bool Survey::is_resting() const {
    return this->phase == Phase::SETTLE or this->phase == Phase::PAUSE or this->phase == Phase::REST;
}

uint16_t& Survey::get_page() {
    return this->page;
}

const uint16_t& Survey::get_count() const {
    return this->count;
}

std::vector<uint8_t> Survey::serialize() const {
    constexpr std::size_t sample_size = 1 + (3 + nav::number_of_wall_sensors) * sizeof(float);

    const uint32_t first = static_cast<uint32_t>(this->page) * samples_per_page;
    const uint32_t last = std::min<uint32_t>(first + samples_per_page, this->count);
    const uint32_t samples = last > first ? last - first : 0;

    std::vector<uint8_t> buffer(4 + samples * sample_size, 0);

    std::memcpy(buffer.data(), &this->page, sizeof(uint16_t));
    std::memcpy(buffer.data() + 2, &this->count, sizeof(uint16_t));

    for (uint32_t i = 0; i < samples; i++) {
        const Sample& sample = this->samples.at(first + i);
        uint8_t*      out = buffer.data() + 4 + i * sample_size;

        const std::array<float, 3 + nav::number_of_wall_sensors> values{
            sample.travel,
            sample.lateral,
            sample.heading,
            sample.intensities.at(0),
            sample.intensities.at(1),
            sample.intensities.at(2),
            sample.intensities.at(3),
        };

        *out = sample.pass;
        std::memcpy(out + 1, values.data(), sizeof(values));
    }

    return buffer;
}

void Survey::deserialize([[maybe_unused]] const uint8_t* serial_data, [[maybe_unused]] uint16_t size) { }

void Survey::record(
    const nav::Pose& pose, const std::array<float, nav::number_of_wall_sensors>& intensities, uint8_t pass
) {
    this->last_travel = this->get_travel(pose);

    if (this->count >= capacity) {
        return;
    }

    const float cosine = std::cos(this->origin.orientation);
    const float sine = std::sin(this->origin.orientation);
    const float dx = pose.position.x - this->origin.position.x;
    const float dy = pose.position.y - this->origin.position.y;

    this->samples.at(this->count) = {
        .travel = this->last_travel,
        .lateral = (cosine * dy) - (sine * dx),
        .heading = core::assert_angle(pose.orientation - this->origin.orientation),
        .intensities = intensities,
        .pass = pass,
    };
    this->count++;
}

float Survey::get_travel(const nav::Pose& pose) const {
    const float dx = pose.position.x - this->origin.position.x;
    const float dy = pose.position.y - this->origin.position.y;

    return -((std::cos(this->origin.orientation) * dx) + (std::sin(this->origin.orientation) * dy));
}
}  // namespace micras
