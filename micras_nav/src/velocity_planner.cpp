/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <ranges>
#include <span>

#include "micras/nav/curve_speed.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/turn_table.hpp"
#include "micras/nav/velocity_planner.hpp"

namespace micras::nav {
float VelocityPlanner::plan(
    std::span<Segment> route, const Dynamics& dynamics, const RunProfile& profile, float start_speed, float end_speed
) {
    if (route.empty()) {
        return 0.0F;
    }

    const MotionLimits linear_limits = dynamics.get_linear_limits(profile);
    const CurveLimits  curve_limits = dynamics.get_curve_limits(profile);

    const auto is_straight = [](const Segment& segment) {
        return segment.kind == SegmentKind::STRAIGHT or segment.kind == SegmentKind::DIAGONAL;
    };

    for (Segment& segment : route) {
        segment.start_speed = 0.0F;
        segment.end_speed = 0.0F;

        if (is_straight(segment)) {
            segment.start_speed = linear_limits.max_speed;
            segment.end_speed = linear_limits.max_speed;
        } else if (segment.kind == SegmentKind::TURN) {
            const TurnShape& shape = dynamics.get_turn(profile, segment.turn);

            segment.start_speed = curve_limits.get_speed_limit(shape.bending_at(0.0F));
            segment.end_speed = curve_limits.get_speed_limit(shape.bending_at(shape.length()));
        }
    }

    float next_speed = end_speed;

    for (Segment& segment : route | std::views::reverse) {
        segment.end_speed = std::min(segment.end_speed, next_speed);

        if (is_straight(segment)) {
            segment.start_speed = std::min(
                segment.start_speed,
                SpeedProfile::get_brakeable_speed(std::abs(segment.length), segment.end_speed, linear_limits)
            );
        } else if (segment.kind == SegmentKind::TURN) {
            segment.start_speed = std::min(
                segment.start_speed,
                CurveSpeed::get_entry_speed(dynamics.get_turn(profile, segment.turn), segment.end_speed, curve_limits)
            );
        } else {
            segment.start_speed = std::min(segment.start_speed, segment.end_speed);
        }

        next_speed = segment.start_speed;
    }

    float previous_speed = start_speed;

    for (Segment& segment : route) {
        segment.start_speed = std::min(segment.start_speed, previous_speed);

        if (is_straight(segment)) {
            segment.end_speed = std::min(
                segment.end_speed,
                SpeedProfile::get_reachable_speed(std::abs(segment.length), segment.start_speed, linear_limits)
            );
        } else if (segment.kind == SegmentKind::TURN) {
            segment.end_speed = std::min(
                segment.end_speed,
                CurveSpeed::get_exit_speed(dynamics.get_turn(profile, segment.turn), segment.start_speed, curve_limits)
            );
        } else {
            segment.end_speed = std::min(segment.end_speed, segment.start_speed);
        }

        previous_speed = segment.end_speed;
    }

    float total_time = 0.0F;

    for (const Segment& segment : route) {
        total_time += get_duration(segment, dynamics, profile);
    }

    return total_time;
}

float VelocityPlanner::get_duration(const Segment& segment, const Dynamics& dynamics, const RunProfile& profile) {
    switch (segment.kind) {
        case SegmentKind::STRAIGHT:
        case SegmentKind::DIAGONAL:
            return SpeedProfile{
                std::abs(segment.length), segment.start_speed, segment.end_speed, dynamics.get_linear_limits(profile)
            }
                .duration();

        case SegmentKind::TURN:
            return CurveSpeed{
                dynamics.get_turn(profile, segment.turn), segment.start_speed, segment.end_speed,
                dynamics.get_curve_limits(profile)
            }
                .duration();

        case SegmentKind::LINE:
            return 0.0F;

        case SegmentKind::SPIN:
            return SpeedProfile{std::abs(segment.length), 0.0F, 0.0F, dynamics.get_angular_limits(profile)}.duration();

        case SegmentKind::STOP:
        case SegmentKind::ATTACH:
            return segment.length;
    }

    return 0.0F;
}
}  // namespace micras::nav
