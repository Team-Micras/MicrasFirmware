/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <span>

#include "micras/nav/executor.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/turn_table.hpp"
#include "micras/nav/velocity_planner.hpp"

namespace micras::nav {
Executor::Executor(const Dynamics& dynamics, const Config& config) : dynamics{dynamics}, config{config} {
    this->segments.reserve(config.capacity);
    this->durations.reserve(config.capacity);
}

void Executor::reset(const Pose& pose, const RunProfile& profile) {
    this->run_profile = profile;
    this->segments.clear();
    this->durations.clear();
    this->index = 0;
    this->clock = 0.0F;
    this->duration = 0.0F;
    this->queued_time = 0.0F;
    this->settled_time = 0.0F;
    this->reference = {.pose = pose, .twist = {}, .acceleration = {}, .distance = 0.0F};
}

void Executor::push(std::span<const Segment> segments) {
    const bool was_finished = this->is_finished();

    if (was_finished) {
        this->segments.clear();
        this->durations.clear();
        this->index = 0;
        this->queued_time = 0.0F;
    }

    for (const Segment& segment : segments) {
        this->segments.push_back(segment);
        this->durations.push_back(VelocityPlanner::get_duration(segment, this->dynamics, this->run_profile));
        this->queued_time += this->durations.back();
    }

    if (was_finished and not this->segments.empty()) {
        this->start_next();
    }
}

Reference Executor::update(float elapsed_time, float time_scale, const State& estimate) {
    if (this->is_finished()) {
        return this->reference;
    }

    this->clock += elapsed_time * time_scale;

    while (not this->is_finished()) {
        const bool attaching = this->segments.at(this->index).kind == SegmentKind::ATTACH;

        if (this->clock < this->duration and not(attaching and this->has_settled(estimate, elapsed_time))) {
            break;
        }

        const float carried = attaching ? 0.0F : this->clock - this->duration;

        this->clock = this->duration;
        this->reference = this->evaluate();
        this->index++;

        if (this->is_finished()) {
            this->reference.twist = {};
            this->reference.acceleration = {};
            return this->reference;
        }

        this->start_next();
        this->clock = carried;
    }

    this->reference = this->evaluate();
    this->reference.twist.linear *= time_scale;
    this->reference.twist.angular *= time_scale;
    this->reference.acceleration.linear *= time_scale * time_scale;
    this->reference.acceleration.angular *= time_scale * time_scale;

    return this->reference;
}

bool Executor::is_finished() const {
    return this->index >= this->segments.size();
}

bool Executor::is_ending(float horizon) const {
    if (this->is_finished()) {
        return true;
    }

    return this->clock + horizon >= this->duration + this->queued_time;
}

void Executor::divert(std::span<const Segment> segments) {
    this->segments.resize(std::min(this->index, this->segments.size()));
    this->durations.resize(this->segments.size());
    this->index = this->segments.size();
    this->duration = 0.0F;
    this->clock = 0.0F;
    this->queued_time = 0.0F;
    this->push(segments);
}

const Segment* Executor::get_current() const {
    return this->is_finished() ? nullptr : &this->segments.at(this->index);
}

const Reference& Executor::get_reference() const {
    return this->reference;
}

void Executor::start_next() {
    const Segment& segment = this->segments.at(this->index);

    this->clock = 0.0F;
    this->settled_time = 0.0F;
    this->speed_profile = {};
    this->queued_time = std::max(this->queued_time - this->durations.at(this->index), 0.0F);

    switch (segment.kind) {
        case SegmentKind::STRAIGHT:
        case SegmentKind::DIAGONAL:
            this->speed_profile = SpeedProfile{
                std::abs(segment.length), segment.start_speed, segment.end_speed,
                this->dynamics.get_linear_limits(this->run_profile)
            };
            this->duration = this->speed_profile.duration();
            break;

        case SegmentKind::SPIN:
            this->speed_profile = SpeedProfile{
                std::abs(segment.length), 0.0F, 0.0F, this->dynamics.get_angular_limits(this->run_profile)
            };
            this->duration = this->speed_profile.duration();
            break;

        default:
            this->duration = this->durations.at(this->index);
            break;
    }
}

Reference Executor::evaluate() const {
    const Segment& segment = this->segments.at(this->index);
    const float    side = std::copysign(1.0F, segment.length);

    switch (segment.kind) {
        case SegmentKind::STRAIGHT:
        case SegmentKind::DIAGONAL: {
            const SpeedProfile::Sample sample = this->speed_profile.sample(this->clock);

            return {
                .pose =
                    segment.start.compose({.position = {.x = side * sample.distance, .y = 0.0F}, .orientation = 0.0F}),
                .twist = {.linear = side * sample.speed, .angular = 0.0F},
                .acceleration = {.linear = side * sample.acceleration, .angular = 0.0F},
                .distance = sample.distance,
            };
        }

        case SegmentKind::TURN: {
            const TurnShape& shape = this->dynamics.get_turn(this->run_profile, segment.turn);
            const float      speed = segment.start_speed;
            const float      distance = std::min(speed * this->clock, shape.length());
            const auto       sample = shape.sample<float>(distance);

            return {
                .pose = segment.start.compose(
                    {.position = {.x = sample.x, .y = side * sample.y}, .orientation = side * sample.heading}
                ),
                .twist = {.linear = speed, .angular = side * sample.curvature * speed},
                .acceleration = {.linear = 0.0F, .angular = side * sample.sharpness * speed * speed},
                .distance = distance,
            };
        }

        case SegmentKind::SPIN: {
            const SpeedProfile::Sample sample = this->speed_profile.sample(this->clock);

            return {
                .pose = segment.start.compose({.position = {}, .orientation = side * sample.distance}),
                .twist = {.linear = 0.0F, .angular = side * sample.speed},
                .acceleration = {.linear = 0.0F, .angular = side * sample.acceleration},
                .distance = sample.distance,
            };
        }

        case SegmentKind::STOP:
        case SegmentKind::ATTACH:
            break;
    }

    return {.pose = segment.start, .twist = {}, .acceleration = {}, .distance = 0.0F};
}

bool Executor::has_settled(const State& estimate, float elapsed_time) {
    const Pose error = this->segments.at(this->index).start.relative(estimate.pose);

    const bool within_tolerance = error.position.magnitude() < this->config.settle_distance and
                                  std::abs(error.orientation) < this->config.settle_angle and
                                  std::abs(estimate.velocity.linear) < this->config.settle_linear_speed and
                                  std::abs(estimate.velocity.angular) < this->config.settle_angular_speed;

    this->settled_time = within_tolerance ? this->settled_time + elapsed_time : 0.0F;

    return this->settled_time >= this->config.settle_time;
}
}  // namespace micras::nav
