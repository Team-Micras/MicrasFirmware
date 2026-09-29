/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <span>

#include "constants.hpp"
#include "micras/nav/curve_speed.hpp"
#include "micras/nav/executor.hpp"
#include "micras/nav/gyroscope_calibration.hpp"
#include "micras/nav/lattice.hpp"
#include "micras/nav/line.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/turn_table.hpp"
#include "test_host.hpp"

using namespace micras;
using namespace micras::nav;

namespace {
constexpr float step{loop_time};
constexpr float infinity{std::numeric_limits<float>::infinity()};
constexpr int   max_iterations{static_cast<int>(20.0F / loop_time)};

const RunProfile fast_profile{make_run_profile(false, false, false, true)};

bool is_near(float value, float expected, float tolerance) {
    return std::abs(value - expected) <= tolerance;
}

Segment straight(float length, float speed, const Pose& start) {
    return {
        .kind = SegmentKind::STRAIGHT,
        .turn = TurnId::SS90S,
        .length = length,
        .start_speed = speed,
        .end_speed = speed,
        .max_speed = infinity,
        .start = start,
    };
}

Segment turn(TurnId id, float side, float speed, const Pose& start) {
    return {
        .kind = SegmentKind::TURN,
        .turn = id,
        .length = side,
        .start_speed = speed,
        .end_speed = speed,
        .max_speed = infinity,
        .start = start,
    };
}

Pose turn_point(const TurnShape& shape, const Segment& segment, float distance) {
    const auto  point = shape.sample<float>(distance);
    const float side = std::copysign(1.0F, segment.length);

    return segment.start.compose(
        {.position = {.x = point.x, .y = side * point.y}, .orientation = side * point.heading}
    );
}

/**
 * @brief How a braking went, from the brake to the end of what the executor was given.
 */
struct Braking {
    Reference at_brake;
    Reference at_rest;
    float     largest_jump;
    float     largest_speed;
    bool      speed_never_rose;
};

Braking brake_and_follow(Executor& executor, float time_before_brake) {
    const State estimate{};

    for (float time = 0.0F; time < time_before_brake; time += step) {
        executor.update(step, 1.0F, estimate);
    }

    Braking braking{
        .at_brake = executor.get_reference(),
        .at_rest = {},
        .largest_jump = 0.0F,
        .largest_speed = 0.0F,
        .speed_never_rose = true,
    };

    executor.brake(mission_config.stop_time);

    Reference last = braking.at_brake;

    for (int i = 0; i < max_iterations and not executor.is_finished(); i++) {
        const Reference& reference = executor.update(step, 1.0F, estimate);
        const float      speed = std::abs(reference.twist.linear);

        braking.largest_jump =
            std::max(braking.largest_jump, (reference.pose.position - last.pose.position).magnitude());
        braking.largest_speed = std::max(braking.largest_speed, speed);
        braking.speed_never_rose = braking.speed_never_rose and speed <= std::abs(last.twist.linear) + 1.0e-4F;
        last = reference;
    }

    CHECK(executor.is_finished());
    braking.at_rest = last;

    return braking;
}

void test_braked_speed() {
    const MotionLimits constant{
        .max_speed = 5.0F,
        .acceleration = 10.0F,
        .deceleration = 10.0F,
        .motor_acceleration = 40.0F,
        .motor_speed = 5.0F
    };

    CHECK(constant.braking_crossover_speed() <= 0.0F);

    for (const float distance : {0.0F, 0.1F, 0.3F, 0.44F}) {
        const float expected = std::sqrt(9.0F - 2.0F * constant.deceleration * distance);
        CHECK(is_near(SpeedProfile::get_braked_speed(distance, 3.0F, constant), expected, 1.0e-4F));
    }

    CHECK(SpeedProfile::get_braked_speed(0.45F, 3.0F, constant) == 0.0F);
    CHECK(SpeedProfile::get_braked_speed(1.0F, 3.0F, constant) == 0.0F);

    const Dynamics     dynamics{dynamics_config};
    const MotionLimits limits = dynamics.get_linear_limits(fast_profile);
    const float        stopping = SpeedProfile::get_braking_distance(3.0F, 0.0F, limits);

    for (const float share : {0.1F, 0.5F, 0.9F}) {
        const float speed = SpeedProfile::get_braked_speed(share * stopping, 3.0F, limits);
        CHECK(speed > 0.0F and speed < 3.0F);
        CHECK(is_near(SpeedProfile::get_braking_distance(3.0F, speed, limits), share * stopping, 1.0e-4F));
    }
}

void test_braked_straight() {
    const Dynamics     dynamics{dynamics_config};
    const Line         line{};
    const MotionLimits limits = dynamics.get_linear_limits(search_profile);
    Executor           executor{dynamics, line, mission_config.executor};
    const std::array   segments{straight(1.0F, 1.0F, {})};

    executor.reset({}, search_profile);
    executor.push(segments);

    const Braking braking = brake_and_follow(executor, 0.2F);
    const float   speed = braking.at_brake.twist.linear;
    const float   travelled = braking.at_rest.pose.position.x - braking.at_brake.pose.position.x;

    CHECK(is_near(speed, 1.0F, 1.0e-3F));
    CHECK(limits.braking_crossover_speed() <= speed);
    CHECK(is_near(travelled, SpeedProfile::get_braking_distance(speed, 0.0F, limits), 1.0e-4F));

    if (limits.braking_crossover_speed() <= 0.0F) {
        CHECK(is_near(travelled, speed * speed / (2.0F * limits.deceleration), 1.0e-4F));
    }

    CHECK(braking.speed_never_rose);
    CHECK(braking.at_rest.pose.position.y == 0.0F and braking.at_rest.pose.orientation == 0.0F);
    CHECK(braking.at_rest.twist.linear == 0.0F);
}

void test_braked_curve_deceleration() {
    const Dynamics    dynamics{dynamics_config};
    const CurveLimits limits = dynamics.get_curve_limits(fast_profile);
    float             worst = 0.0F;

    for (uint8_t id = 0; id < number_of_turns; id++) {
        const auto       turn_id = static_cast<TurnId>(id);
        const TurnShape& shape = dynamics.get_turn(fast_profile, turn_id);
        const auto       bending = [&shape](float distance) { return shape.bending_at(distance); };

        for (const float share : {0.0F, 0.3F, 0.7F}) {
            for (const float speed : {dynamics.get_turn_speed(fast_profile, turn_id), 1.0F}) {
                const float      start = share * shape.length();
                const CurveSpeed motion = CurveSpeed::braking(bending, start, shape.length(), speed, limits);

                for (float time = 0.0F; time < motion.duration(); time += step) {
                    const SpeedProfile::Sample sample = motion.sample(time);
                    const float allowed = limits.get_deceleration(sample.speed, bending(sample.distance));

                    CHECK(sample.acceleration <= 0.0F);
                    worst = std::max(worst, -sample.acceleration - allowed);
                }

                const SpeedProfile::Sample end = motion.sample(motion.duration());

                CHECK(is_near(end.speed, motion.end_speed(), 1.0e-4F));
                CHECK(end.speed <= speed);
                CHECK(motion.end_speed() == 0.0F or is_near(end.distance, shape.length(), 1.0e-5F));
            }
        }
    }

    CHECK(worst <= 1.0e-3F);
}

void test_braked_turn_rests_on_it() {
    const Dynamics   dynamics{dynamics_config};
    const Line       line{};
    const TurnShape& shape = dynamics.get_turn(search_profile, TurnId::SS90S);
    const float      speed = 0.5F * dynamics.get_turn_speed(search_profile, TurnId::SS90S);
    Executor         executor{dynamics, line, mission_config.executor};
    const Segment    curve = turn(TurnId::SS90S, 1.0F, speed, {});
    const std::array segments{curve};

    executor.reset({}, search_profile);
    executor.push(segments);

    const CurveSpeed planned{shape, speed, speed, dynamics.get_curve_limits(search_profile)};
    const Braking    braking = brake_and_follow(executor, 0.3F * planned.duration());
    const Pose       rest = braking.at_rest.pose;

    float closest = infinity;
    float rest_distance = 0.0F;

    for (float distance = braking.at_brake.distance; distance <= shape.length(); distance += 1.0e-5F) {
        const float gap = (turn_point(shape, curve, distance).position - rest.position).magnitude();

        if (gap < closest) {
            closest = gap;
            rest_distance = distance;
        }
    }

    CHECK(closest < 1.0e-4F);
    CHECK(rest_distance < shape.length());
    CHECK(is_near(rest.orientation, turn_point(shape, curve, rest_distance).orientation, 1.0e-3F));
    CHECK(braking.speed_never_rose);
    CHECK(braking.largest_jump <= braking.largest_speed * step + 1.0e-5F);
}

void test_braked_turn_goes_on_straight() {
    const Dynamics   dynamics{dynamics_config};
    const Line       line{};
    const TurnShape& shape = dynamics.get_turn(fast_profile, TurnId::SS90S);
    const float      speed = dynamics.get_turn_speed(fast_profile, TurnId::SS90S);
    Executor         executor{dynamics, line, mission_config.executor};
    const Segment    curve = turn(TurnId::SS90S, -1.0F, speed, {});
    const Pose       exit = turn_point(shape, curve, shape.length());
    const std::array segments{curve, straight(0.5F, speed, exit)};

    executor.reset({}, fast_profile);
    executor.push(segments);

    const CurveSpeed planned{shape, speed, speed, dynamics.get_curve_limits(fast_profile)};
    const State      estimate{};

    for (float time = 0.0F; time < 0.95F * planned.duration(); time += step) {
        executor.update(step, 1.0F, estimate);
    }

    executor.brake(mission_config.stop_time);

    CHECK(executor.get_current()->kind == SegmentKind::TURN);
    CHECK(not executor.is_ending(planned.duration()));

    const Braking braking = brake_and_follow(executor, 0.0F);
    const Pose    along = exit.relative(braking.at_rest.pose);

    CHECK(braking.speed_never_rose);
    CHECK(braking.largest_jump <= braking.largest_speed * step + 1.0e-5F);
    CHECK(along.position.x > 0.0F);
    CHECK(std::abs(along.position.y) < 1.0e-4F and std::abs(along.orientation) < 1.0e-4F);
}

void test_gyroscope_brake() {
    const Dynamics       dynamics{dynamics_config};
    const MotionLimits   limits = dynamics.get_angular_limits(search_profile);
    GyroscopeCalibration calibration{gyroscope_calibration_config};
    const Measurements   measurements{};
    Reference            reference{};

    calibration.start({}, limits);

    for (float time = 0.0F; time < gyroscope_calibration_config.settle_time + 0.5F; time += step) {
        reference = calibration.update(measurements, 0.0F, step);
    }

    const float speed = reference.twist.angular;
    const float angle = reference.pose.orientation;

    CHECK(speed > 1.0F);

    calibration.brake(limits);

    float last_speed = speed;
    bool  within_limits = true;

    for (int i = 0; i < max_iterations and not calibration.is_finished(); i++) {
        reference = calibration.update(measurements, 0.0F, step);
        within_limits = within_limits and reference.twist.angular <= last_speed + 1.0e-4F and
                        -reference.acceleration.angular <= limits.deceleration_at(last_speed) + 1.0e-3F;
        last_speed = reference.twist.angular;
    }

    const float braked = SpeedProfile::get_braking_distance(speed, 0.0F, limits);

    CHECK(calibration.is_finished());
    CHECK(not calibration.is_valid());
    CHECK(within_limits);
    CHECK(reference.twist.angular == 0.0F);
    CHECK(is_near(reference.pose.orientation, angle + braked, 1.0e-3F));

    for (int i = 0; i < 100; i++) {
        const Reference held = calibration.update(measurements, 0.0F, step);
        CHECK(is_near(held.pose.orientation, angle + braked, 1.0e-3F));
        CHECK(held.twist.angular == 0.0F and held.acceleration.angular == 0.0F);
    }
}
}  // namespace

int main() {
    test_braked_speed();
    test_braked_straight();
    test_braked_curve_deceleration();
    test_braked_turn_rests_on_it();
    test_braked_turn_goes_on_straight();
    test_gyroscope_brake();

    std::puts("braking ok");
}
