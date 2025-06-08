/**
 * @file
 */

#include <cmath>
#include <iterator>
#include <numbers>

#include "micras/nav/action_queuer.hpp"

namespace micras::nav {
ActionQueuer::ActionQueuer(Config config) :
    cell_size{config.cell_size},
    start_offset{config.start_offset},
    curve_safety_margin{config.curve_safety_margin},
    solving_params{config.solving} {
    const float exploration_curve_radius = config.exploring.max_linear_speed * config.exploring.max_linear_speed /
                                           config.exploring.max_centrifugal_acceleration;

    const float exploration_max_angular_speed = TurnAction::calculate_max_angular_speed(
        std::numbers::pi_v<float> / 2.0F, exploration_curve_radius, config.exploring.max_angular_acceleration,
        config.exploring.max_centrifugal_acceleration
    );

    const float exploration_linear_speed =
        config.exploring.max_centrifugal_acceleration / exploration_max_angular_speed;

    MoveAction::Config move_config = {
        .start_speed = 0.0F,
        .end_speed = exploration_linear_speed,
        .max_speed = exploration_linear_speed,
        .max_acceleration = config.exploring.max_linear_acceleration,
        .max_deceleration = config.exploring.max_linear_deceleration
    };

    this->start = std::make_shared<MoveAction>(ActionType::START, this->cell_size - config.start_offset, move_config);
    this->move_half = std::make_shared<MoveAction>(ActionType::MOVE_FORWARD, cell_size / 2.0F, move_config);

    move_config.start_speed = exploration_linear_speed;

    this->move_forward = std::make_shared<MoveAction>(ActionType::MOVE_FORWARD, cell_size, move_config);
    this->move_to_turn = std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, this->cell_size / 2.0F - exploration_curve_radius, move_config, false
    );
    this->move_from_turn = std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, this->cell_size / 2.0F - exploration_curve_radius, move_config
    );

    move_config.end_speed = 0.0F;

    this->stop = std::make_shared<MoveAction>(ActionType::STOP, cell_size / 2.0F, move_config, false);

    const TurnAction::Config turn_config = {
        .max_angular_speed = exploration_max_angular_speed,
        .linear_speed = exploration_linear_speed,
        .max_angular_acceleration = config.exploring.max_angular_acceleration
    };

    this->turn_left = std::make_shared<TurnAction>(ActionType::TURN, std::numbers::pi_v<float> / 2.0F, turn_config);
    this->turn_right = std::make_shared<TurnAction>(ActionType::TURN, -std::numbers::pi_v<float> / 2.0F, turn_config);

    const TurnAction::Config turn_back_config = {
        .max_angular_speed = config.exploring.max_angular_acceleration * 0.01F,
        .linear_speed = 0.0F,
        .max_angular_acceleration = config.exploring.max_angular_acceleration
    };

    this->turn_back = std::make_shared<TurnAction>(ActionType::SPIN, std::numbers::pi_v<float>, turn_back_config);

    this->compute_curve_parameters(std::numbers::pi_v<float> / 4.0F, false);
    this->compute_curve_parameters(std::numbers::pi_v<float> / 2.0F, false);
    this->compute_curve_parameters(std::numbers::pi_v<float> / 2.0F, true);
    this->compute_curve_parameters(3.0F * std::numbers::pi_v<float> / 4.0F, false);
    this->compute_curve_parameters(std::numbers::pi_v<float>, false);
}

void ActionQueuer::push_exploring(const GridPose& origin_pose, const GridPoint& target_position) {
    if (origin_pose.front().position == target_position) {
        this->action_queue.emplace_back(this->move_forward);
        return;
    }

    if (origin_pose.turned_left().front().position == target_position) {
        this->action_queue.emplace_back(this->move_to_turn);
        this->action_queue.emplace_back(this->turn_left);
        this->action_queue.emplace_back(this->move_to_turn);
        return;
    }

    if (origin_pose.turned_right().front().position == target_position) {
        this->action_queue.emplace_back(this->move_to_turn);
        this->action_queue.emplace_back(this->turn_right);
        this->action_queue.emplace_back(this->move_from_turn);
        return;
    }

    if (origin_pose.turned_back().front().position == target_position) {
        this->action_queue.emplace_back(this->stop);
        this->action_queue.emplace_back(this->turn_back);
        this->action_queue.emplace_back(this->move_half);
        return;
    }
}

std::shared_ptr<Action> ActionQueuer::pop() {
    auto action = this->action_queue.front();
    this->action_queue.pop_front();
    return action;
}

bool ActionQueuer::empty() const {
    return this->action_queue.empty();
}

void ActionQueuer::recompute(const std::list<GridPoint>& best_route, bool add_start) {
    this->action_queue = {};

    if (add_start) {
        this->action_queue.emplace_back(start);
    }

    if (best_route.empty()) {
        return;
    }

    std::list<Action::Id> actions;

    Side direction = best_route.front().direction(*std::next(best_route.begin()));

    for (auto route_it = best_route.begin(); std::next(route_it) != best_route.end(); route_it++) {
        actions.push_back(get_action({*route_it, direction}, *std::next(route_it), this->cell_size));
        direction = (*route_it).direction(*std::next(route_it));
    }

    this->add_diagonals(actions);
    this->join_actions(actions);

    actions.front().value -= this->start_offset;
    float start_speed = 0.0F;

    for (auto action_it = std::next(actions.begin()); action_it != actions.end(); action_it++) {
        if (action_it->type != ActionType::TURN) {
            continue;
        }

        if (std::prev(action_it)->value < 0.0F) {
            std::terminate();
        }

        const CurveParameters& curve_parameters =
            this->get_curve_parameters(action_it->value, std::prev(action_it)->type == ActionType::DIAGONAL);

        const MoveAction::Config move_config = {
            .start_speed = start_speed,
            .end_speed = curve_parameters.linear_speed,
            .max_speed = this->solving_params.max_linear_speed,
            .max_acceleration = this->solving_params.max_linear_acceleration,
            .max_deceleration = this->solving_params.max_linear_deceleration
        };
        this->action_queue.emplace_back(std::make_shared<MoveAction>(
            std::prev(action_it)->type, std::prev(action_it)->value, move_config,
            std::prev(action_it)->type != ActionType::DIAGONAL
        ));

        const TurnAction::Config turn_config = {
            .max_angular_speed = curve_parameters.max_angular_speed,
            .linear_speed = curve_parameters.linear_speed,
            .max_angular_acceleration = this->solving_params.max_angular_acceleration
        };
        this->action_queue.emplace_back(std::make_shared<TurnAction>(action_it->type, action_it->value, turn_config));

        start_speed = curve_parameters.linear_speed;
    }

    const MoveAction::Config stop_config = {
        .start_speed = start_speed,
        .end_speed = 0.0F,
        .max_speed = this->solving_params.max_linear_speed,
        .max_acceleration = this->solving_params.max_linear_acceleration,
        .max_deceleration = this->solving_params.max_linear_deceleration
    };

    if (actions.back().type == ActionType::MOVE_FORWARD) {
        this->action_queue.emplace_back(
            std::make_shared<MoveAction>(ActionType::STOP, actions.back().value + this->cell_size / 2.0F, stop_config)
        );
    } else {
        this->action_queue.emplace_back(
            std::make_shared<MoveAction>(ActionType::STOP, this->cell_size / 2.0F, stop_config)
        );
    }
}

float ActionQueuer::get_total_time() const {
    float total_time = 0.0F;

    for (const auto& action : this->action_queue) {
        total_time += action->get_total_time();
    }

    return total_time;
}

Action::Id ActionQueuer::get_action(const GridPose& origin_pose, const GridPoint& target_point, float cell_size) {
    if (origin_pose.front().position == target_point) {
        const float distance = origin_pose.position.to_vector(cell_size).distance(target_point.to_vector(cell_size));
        return {ActionType::MOVE_FORWARD, distance};
    }

    if (origin_pose.turned_left().front().position == target_point) {
        return {ActionType::TURN, std::numbers::pi_v<float> / 2.0F};
    }

    if (origin_pose.turned_right().front().position == target_point) {
        return {ActionType::TURN, -std::numbers::pi_v<float> / 2.0F};
    }

    return {ActionType::STOP, cell_size / 2.0F};
}

void ActionQueuer::add_diagonals(std::list<Action::Id>& actions) const {
    for (auto action_it = actions.begin(); std::next(action_it) != actions.end(); action_it++) {
        uint8_t diagonal_count = 1;

        while (action_it->type == ActionType::TURN and std::next(action_it)->type == ActionType::TURN and
               std::abs(action_it->value) == std::numbers::pi_v<float> / 2.0F and
               action_it->value == -std::next(action_it)->value) {
            diagonal_count++;
            action_it = actions.erase(action_it);
        }

        if (diagonal_count == 1) {
            continue;
        }

        action_it->value /= 2.0F;
        const float first_angle = diagonal_count % 2 == 0 ? -action_it->value : action_it->value;
        const float diagonal_distance = diagonal_count * this->cell_size * std::numbers::sqrt2_v<float> / 2.0F;
        actions.insert(action_it, {{ActionType::TURN, first_angle}, {ActionType::DIAGONAL, diagonal_distance}});
    }
}

void ActionQueuer::join_actions(std::list<Action::Id>& actions) {
    for (auto action_it = actions.begin(); std::next(action_it) != actions.end();) {
        if (action_it->type == std::next(action_it)->type) {
            const float value = action_it->value;
            action_it = actions.erase(action_it);
            action_it->value += value;
            continue;
        }

        if (action_it->type != ActionType::TURN) {
            action_it++;
            continue;
        }

        auto [trim_before_distance, trim_after_distance] = this->get_trim_distances(*std::prev(action_it), *action_it);

        std::prev(action_it)->value -= trim_before_distance;
        std::next(action_it)->value -= trim_after_distance;
        action_it++;
    }
}

std::pair<float, float>
    ActionQueuer::get_trim_distances(const Action::Id& action_before, const Action::Id& turn_action) {
    const float            turn_angle = std::abs(turn_action.value);
    const CurveParameters& curve_parameters =
        this->get_curve_parameters(turn_angle, action_before.type == ActionType::DIAGONAL);

    if (turn_angle == std::numbers::pi_v<float> / 4.0F) {
        const float trim_before_distance = curve_parameters.forward_displacement - curve_parameters.side_displacement;
        const float trim_after_distance = curve_parameters.side_displacement * std::numbers::sqrt2_v<float>;

        return {trim_before_distance, trim_after_distance};
    }

    if (turn_angle == std::numbers::pi_v<float> / 2.0F) {
        float trim_before_distance = curve_parameters.forward_displacement;
        float trim_after_distance = curve_parameters.side_displacement;

        if (action_before.type != ActionType::DIAGONAL) {
            trim_before_distance -= this->cell_size / 2.0F;
            trim_after_distance -= this->cell_size / 2.0F;
        }

        return {trim_before_distance, trim_after_distance};
    }

    if (turn_angle == 3.0F * std::numbers::pi_v<float> / 4.0F) {
        float trim_before_distance = curve_parameters.forward_displacement + curve_parameters.side_displacement;
        float trim_after_distance = std::numbers::sqrt2_v<float> * curve_parameters.side_displacement;

        if (action_before.type == ActionType::DIAGONAL) {
            trim_before_distance -= this->cell_size * std::numbers::sqrt2_v<float> / 2.0F;
            trim_after_distance -= this->cell_size;
        } else {
            trim_before_distance -= this->cell_size;
            trim_after_distance -= this->cell_size * std::numbers::sqrt2_v<float> / 2.0F;
        }

        return {trim_before_distance, trim_after_distance};
    }

    if (turn_angle == std::numbers::pi_v<float>) {
        const float trim_distance = this->cell_size / 2.0F - this->curve_safety_margin;

        return {trim_distance, trim_distance};
    }

    return {0.0F, 0.0F};
}

ActionQueuer::CurveParameters& ActionQueuer::get_curve_parameters(float angle, bool is_diagonal) {
    const float turn_angle = std::abs(angle);

    if (turn_angle == std::numbers::pi_v<float> / 4.0F) {
        return this->curves_parameters[CurveType::REGULAR_45];
    }

    if (turn_angle == std::numbers::pi_v<float> / 2.0F) {
        return is_diagonal ? this->curves_parameters[CurveType::DIAGONAL_90] :
                             this->curves_parameters[CurveType::REGULAR_90];
    }

    if (turn_angle == 3.0F * std::numbers::pi_v<float> / 4.0F) {
        return this->curves_parameters[CurveType::REGULAR_135];
    }

    return this->curves_parameters[CurveType::REGULAR_180];
}

void ActionQueuer::compute_curve_parameters(float angle, bool is_diagonal) {
    const float curve_radius = calculate_curve_radius(angle, this->cell_size, this->curve_safety_margin, is_diagonal);
    const float max_angular_speed = TurnAction::calculate_max_angular_speed(
        angle, curve_radius, this->solving_params.max_angular_acceleration,
        this->solving_params.max_centrifugal_acceleration
    );
    const float curve_linear_speed = this->solving_params.max_centrifugal_acceleration / max_angular_speed;

    this->get_curve_parameters(angle, is_diagonal) = {
        .curve_radius = curve_radius,
        .max_angular_speed = max_angular_speed,
        .linear_speed = curve_linear_speed,
        .side_displacement = TurnAction::calculate_side_displacement(
            angle, max_angular_speed, curve_linear_speed, this->solving_params.max_angular_acceleration
        ),
        .forward_displacement = TurnAction::calculate_forward_displacement(
            angle, max_angular_speed, curve_linear_speed, this->solving_params.max_angular_acceleration
        ),
    };
}

constexpr float
    ActionQueuer::calculate_curve_radius(float angle, float cell_size, float safety_margin, bool is_diagonal) {
    const float turn_angle = std::abs(angle);

    if (turn_angle == std::numbers::pi_v<float> / 4.0F) {
        return cell_size * (1.0F + std::numbers::sqrt2_v<float>) / 2.0F;
    }

    if (turn_angle == std::numbers::pi_v<float> / 2.0F) {
        const float curve_radius = (cell_size - 2.0F * safety_margin) / (2.0F * std::numbers::sqrt2_v<float> - 2.0F);

        if (is_diagonal) {
            return curve_radius;
        }

        return curve_radius + cell_size / 2.0F;
    }

    if (turn_angle == 3.0F * std::numbers::pi_v<float> / 4.0F) {
        const float constant = 10.0F * std::numbers::sqrt2_v<float> - 14.0F;
        const float root_term =
            (std::sqrt(constant * (cell_size * cell_size - 2.0F * cell_size * safety_margin)) + cell_size) / 2.0F;

        return root_term - safety_margin * (std::numbers::sqrt2_v<float> - 1.0F);
    }

    return cell_size / 2.0F;
}
}  // namespace micras::nav
