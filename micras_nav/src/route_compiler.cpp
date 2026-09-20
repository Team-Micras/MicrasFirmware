/**
 * @file
 */

#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "micras/nav/lattice.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/planner.hpp"
#include "micras/nav/route_compiler.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
void RouteCompiler::compile(
    const Route& route, const Dynamics& dynamics, const RunProfile& profile, float start_distance,
    std::vector<Segment>& segments
) {
    const float cell_size = dynamics.get_model().maze.cell_size;

    const auto retreat = [cell_size](const LatticePose& node, float distance) {
        return node.to_pose(cell_size).compose({.position = {.x = -distance, .y = 0.0F}, .orientation = 0.0F});
    };

    const auto emit_straight = [&segments](const Pose& start, float length, bool diagonal) {
        if (length > min_straight) {
            segments.push_back({
                .kind = diagonal ? SegmentKind::DIAGONAL : SegmentKind::STRAIGHT,
                .turn = TurnId::SS90S,
                .length = length,
                .start_speed = 0.0F,
                .end_speed = 0.0F,
                .max_speed = 0.0F,
                .start = start,
            });
        }
    };

    segments.clear();

    LatticePose node = route.start;
    Pose        straight_start = retreat(node, start_distance);
    float       straight_length = start_distance;

    for (const RouteStep& step : route.steps) {
        const float step_length = node.is_diagonal() ? cell_size / std::numbers::sqrt2_v<float> : cell_size;

        for (uint8_t i = 0; i < step.run; i++) {
            node = node.advanced();
        }

        straight_length += static_cast<float>(step.run) * step_length;

        if (not step.has_turn) {
            continue;
        }

        const TurnShape& shape = dynamics.get_turn(profile, step.turn);

        emit_straight(straight_start, straight_length + shape.pre, node.is_diagonal());

        segments.push_back({
            .kind = SegmentKind::TURN,
            .turn = step.turn,
            .length = step.side == TurnSide::LEFT ? shape.angle : -shape.angle,
            .start_speed = 0.0F,
            .end_speed = 0.0F,
            .max_speed = 0.0F,
            .start = retreat(node, -shape.pre),
        });

        node = get_turn_exit(node, step.turn, step.side);
        straight_start = retreat(node, shape.post);
        straight_length = shape.post;
    }

    emit_straight(straight_start, straight_length + route.stop_distance, node.is_diagonal());
}
}  // namespace micras::nav
