/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <ranges>
#include <span>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/velocity_planner.hpp"

namespace micras::nav {
float VelocityPlanner::plan(
    std::span<Segment> route, const Dynamics& dynamics, const RunProfile& profile, float start_speed, float end_speed
) {
    if (route.empty()) {
        return 0.0F;
    }

    const MotionLimits linear_limits = dynamics.get_linear_limits(profile);

    const auto is_straight = [](const Segment& segment) {
        return segment.kind == SegmentKind::STRAIGHT or segment.kind == SegmentKind::DIAGONAL;
    };

    for (Segment& segment : route) {
        float limit = 0.0F;

        if (is_straight(segment)) {
            limit = linear_limits.max_speed;
        } else if (segment.kind == SegmentKind::TURN) {
            limit = dynamics.get_turn_speed(profile, segment.turn);
        }

        segment.max_speed = limit;
        segment.start_speed = limit;
        segment.end_speed = limit;
    }

    float next_speed = end_speed;

    for (Segment& segment : route | std::views::reverse) {
        segment.end_speed = std::min(segment.end_speed, next_speed);

        if (is_straight(segment)) {
            segment.start_speed = std::min(
                segment.start_speed,
                SpeedProfile::get_brakeable_speed(std::abs(segment.length), segment.end_speed, linear_limits)
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
            return dynamics.get_turn(profile, segment.turn).length() / segment.start_speed;

        case SegmentKind::SPIN:
            return SpeedProfile{std::abs(segment.length), 0.0F, 0.0F, dynamics.get_angular_limits(profile)}.duration();

        case SegmentKind::STOP:
        case SegmentKind::ATTACH:
            return segment.length;
    }

    return 0.0F;
}
}  // namespace micras::nav
