/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <span>

#include "micras/nav/curve_speed.hpp"
#include "micras/nav/executor.hpp"
#include "micras/nav/lattice.hpp"
#include "micras/nav/line.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/turn_table.hpp"
#include "micras/nav/velocity_planner.hpp"

namespace micras::nav {
Executor::Executor(const Dynamics& dynamics, const Line& line, const Config& config) :
    dynamics{dynamics}, line{line}, config{config} {
    this->segments.reserve(config.capacity);
    this->durations.reserve(config.capacity);
}

void Executor::reset(const Pose& pose, const RunProfile& profile) {
    this->run_profile = profile;
    this->braking = false;
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
    } else {
        const auto done = static_cast<std::ptrdiff_t>(this->index);

        this->segments.erase(this->segments.begin(), this->segments.begin() + done);
        this->durations.erase(this->durations.begin(), this->durations.begin() + done);
        this->index = 0;
    }

    for (const Segment& segment : segments) {
        this->append(segment);
    }

    if (was_finished and not this->segments.empty()) {
        this->start_next();
    }
}

void Executor::append(const Segment& segment) {
    this->segments.push_back(segment);
    this->durations.push_back(
        segment.kind == SegmentKind::LINE ? this->line.duration() :
                                            VelocityPlanner::get_duration(segment, this->dynamics, this->run_profile)
    );
    this->queued_time += this->durations.back();
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
        this->reference = this->evaluate(this->clock);
        this->index++;

        if (this->is_finished()) {
            this->reference.twist = {};
            this->reference.acceleration = {};
            return this->reference;
        }

        this->start_next();
        this->clock = carried;
    }

    this->reference = this->evaluate(this->clock);

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

void Executor::brake(float rest_time) {
    this->braking = true;
    this->rest_time = rest_time;

    if (this->is_finished()) {
        const std::array rest{this->make_rest(this->reference.pose)};
        this->push(rest);
        return;
    }

    this->queued_time = std::accumulate(
        this->durations.begin() + static_cast<std::ptrdiff_t>(this->index) + 1, this->durations.end(), 0.0F
    );

    Segment&    segment = this->segments.at(this->index);
    const float travelled = this->reference.distance;
    const float left = std::max(std::abs(segment.length) - travelled, 0.0F);

    this->clock = 0.0F;

    switch (segment.kind) {
        case SegmentKind::STRAIGHT:
            this->braking_speed = std::abs(this->reference.twist.linear);
            segment.start = this->reference.pose;
            segment.length = std::copysign(left, segment.length);
            this->brake_straight();
            return;

        case SegmentKind::TURN:
        case SegmentKind::LINE:
            this->braking_speed = std::abs(this->reference.twist.linear);
            this->brake_curve(travelled);
            return;

        case SegmentKind::SPIN:
            this->braking_speed = std::abs(this->reference.twist.angular);
            segment.start = this->reference.pose;
            segment.length = std::copysign(left, segment.length);
            this->brake_spin();
            return;

        case SegmentKind::STOP:
        case SegmentKind::ATTACH:
            this->come_to_rest(this->reference.pose);
            return;
    }
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

    if (this->braking) {
        this->start_braked();
        return;
    }

    switch (segment.kind) {
        case SegmentKind::STRAIGHT:
            this->speed_profile = SpeedProfile{
                std::abs(segment.length), segment.start_speed, segment.end_speed,
                this->dynamics.get_linear_limits(this->run_profile).capped(segment.max_speed)
            };
            this->duration = this->speed_profile.duration();
            break;

        case SegmentKind::SPIN:
            this->speed_profile = SpeedProfile{
                std::abs(segment.length), segment.start_speed, segment.end_speed,
                this->dynamics.get_angular_limits(this->run_profile)
            };
            this->duration = this->speed_profile.duration();
            break;

        case SegmentKind::TURN:
            this->curve_speed = CurveSpeed{
                this->dynamics.get_turn(this->run_profile, segment.turn), segment.start_speed, segment.end_speed,
                this->dynamics.get_curve_limits(this->run_profile)
            };
            this->duration = this->curve_speed.duration();
            break;

        default:
            this->duration = this->durations.at(this->index);
            break;
    }
}

void Executor::start_braked() {
    const Segment& segment = this->segments.at(this->index);

    switch (segment.kind) {
        case SegmentKind::STRAIGHT:
            this->brake_straight();
            return;

        case SegmentKind::TURN:
        case SegmentKind::LINE:
            this->brake_curve(0.0F);
            return;

        case SegmentKind::SPIN:
        case SegmentKind::STOP:
        case SegmentKind::ATTACH:
            this->come_to_rest(segment.start);
            return;
    }
}

void Executor::brake_straight() {
    const Segment&     segment = this->segments.at(this->index);
    const MotionLimits limits = this->dynamics.get_linear_limits(this->run_profile);
    const float        available = std::abs(segment.length);
    const float        stopping = SpeedProfile::get_braking_distance(this->braking_speed, 0.0F, limits);

    if (stopping <= available + rest_tolerance) {
        this->speed_profile = SpeedProfile{stopping, this->braking_speed, 0.0F, limits};
        this->duration = this->speed_profile.duration();
        this->braking_speed = 0.0F;
        this->rest_after_current();
        return;
    }

    const float end_speed = SpeedProfile::get_braked_speed(available, this->braking_speed, limits);

    this->speed_profile = SpeedProfile{available, this->braking_speed, end_speed, limits};
    this->duration = this->speed_profile.duration();
    this->braking_speed = end_speed;
    this->continue_braking(segment.length);
}

void Executor::brake_curve(float start_distance) {
    const Segment&    segment = this->segments.at(this->index);
    const CurveLimits limits = this->dynamics.get_curve_limits(this->run_profile);

    if (segment.kind == SegmentKind::TURN) {
        const TurnShape& shape = this->dynamics.get_turn(this->run_profile, segment.turn);

        this->curve_speed = CurveSpeed::braking(
            [&shape](float distance) { return shape.bending_at(distance); }, start_distance, shape.length(),
            this->braking_speed, limits
        );
    } else {
        this->curve_speed = CurveSpeed::braking(
            [this](float distance) {
                const Line::Point point = this->line.sample_point(distance);
                return Bending{.curvature = point.curvature, .sharpness = point.sharpness};
            },
            start_distance, this->line.length(), this->braking_speed, limits
        );
    }

    this->duration = this->curve_speed.duration();
    this->braking_speed = this->curve_speed.end_speed();

    if (this->braking_speed <= rest_speed) {
        this->rest_after_current();
    } else {
        this->continue_braking(1.0F);
    }
}

void Executor::brake_spin() {
    const Segment&     segment = this->segments.at(this->index);
    const MotionLimits limits = this->dynamics.get_angular_limits(this->run_profile);
    const float        stopping = SpeedProfile::get_braking_distance(this->braking_speed, 0.0F, limits);

    this->speed_profile = SpeedProfile{std::min(stopping, std::abs(segment.length)), this->braking_speed, 0.0F, limits};
    this->duration = this->speed_profile.duration();
    this->braking_speed = 0.0F;
    this->rest_after_current();
}

void Executor::come_to_rest(const Pose& pose) {
    this->segments.resize(this->index + 1);
    this->durations.resize(this->index + 1);
    this->segments.at(this->index) = this->make_rest(pose);
    this->durations.at(this->index) = this->rest_time;
    this->duration = this->rest_time;
    this->queued_time = 0.0F;
}

void Executor::rest_after_current() {
    const Segment rest = this->make_rest(this->evaluate(this->duration).pose);

    this->segments.resize(this->index + 1);
    this->durations.resize(this->index + 1);
    this->queued_time = 0.0F;
    this->append(rest);
}

void Executor::continue_braking(float direction) {
    if (this->index + 1 < this->segments.size()) {
        return;
    }

    const MotionLimits limits = this->dynamics.get_linear_limits(this->run_profile);

    this->append({
        .kind = SegmentKind::STRAIGHT,
        .turn = TurnId::SS90S,
        .length = std::copysign(SpeedProfile::get_braking_distance(this->braking_speed, 0.0F, limits), direction),
        .start_speed = this->braking_speed,
        .end_speed = 0.0F,
        .max_speed = std::numeric_limits<float>::infinity(),
        .start = this->evaluate(this->duration).pose,
    });
}

Segment Executor::make_rest(const Pose& pose) const {
    return {
        .kind = SegmentKind::STOP,
        .turn = TurnId::SS90S,
        .length = this->rest_time,
        .start_speed = 0.0F,
        .end_speed = 0.0F,
        .max_speed = std::numeric_limits<float>::infinity(),
        .start = pose,
    };
}

Reference Executor::evaluate(float time) const {
    const Segment& segment = this->segments.at(this->index);
    const float    side = std::copysign(1.0F, segment.length);

    switch (segment.kind) {
        case SegmentKind::STRAIGHT: {
            const SpeedProfile::Sample sample = this->speed_profile.sample(time);

            return {
                .pose =
                    segment.start.compose({.position = {.x = side * sample.distance, .y = 0.0F}, .orientation = 0.0F}),
                .twist = {.linear = side * sample.speed, .angular = 0.0F},
                .acceleration = {.linear = side * sample.acceleration, .angular = 0.0F},
                .distance = sample.distance,
            };
        }

        case SegmentKind::TURN: {
            const TurnShape&           shape = this->dynamics.get_turn(this->run_profile, segment.turn);
            const SpeedProfile::Sample motion = this->curve_speed.sample(time);
            const auto                 point = shape.sample<float>(motion.distance);

            return {
                .pose = segment.start.compose(
                    {.position = {.x = point.x, .y = side * point.y}, .orientation = side * point.heading}
                ),
                .twist = {.linear = motion.speed, .angular = side * point.curvature * motion.speed},
                .acceleration =
                    {
                        .linear = motion.acceleration,
                        .angular = side * (point.sharpness * motion.speed * motion.speed +
                                           point.curvature * motion.acceleration),
                    },
                .distance = motion.distance,
            };
        }

        case SegmentKind::LINE: {
            const SpeedProfile::Sample motion =
                this->braking ? this->curve_speed.sample(time) : this->line.sample_motion(time);
            const Line::Point point = this->line.sample_point(motion.distance);

            return {
                .pose = point.pose,
                .twist = {.linear = motion.speed, .angular = point.curvature * motion.speed},
                .acceleration =
                    {
                        .linear = motion.acceleration,
                        .angular =
                            point.sharpness * motion.speed * motion.speed + point.curvature * motion.acceleration,
                    },
                .distance = motion.distance,
            };
        }

        case SegmentKind::SPIN: {
            const SpeedProfile::Sample sample = this->speed_profile.sample(time);

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
