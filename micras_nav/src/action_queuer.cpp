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
    exploring_params{config.exploring},
    solving_params{config.solving},
    exploration_curve_radius{
        exploring_params.max_linear_speed * exploring_params.max_linear_speed /
        exploring_params.max_centrifugal_acceleration
    },
    stop{std::make_shared<MoveAction>(
        ActionType::STOP, cell_size / 2.0F, exploring_params.max_linear_speed, 0.0F, exploring_params.max_linear_speed,
        exploring_params.max_linear_acceleration, exploring_params.max_linear_deceleration, false
    )},
    start{std::make_shared<MoveAction>(
        ActionType::START, cell_size - config.start_offset, 0.0F, exploring_params.max_linear_speed,
        exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration
    )},
    move_forward{std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, cell_size, exploring_params.max_linear_speed, exploring_params.max_linear_speed,
        exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration
    )},
    move_half{std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, cell_size / 2.0F, 0.0F, exploring_params.max_linear_speed,
        exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration, false
    )},
    move_to_turn{std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, this->cell_size / 2.0F - exploration_curve_radius, exploring_params.max_linear_speed,
        exploring_params.max_linear_speed, exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration
    )},
    turn_left{std::make_shared<TurnAction>(
        ActionType::TURN, std::numbers::pi_v<float> / 2.0F,
        TurnAction::calculate_max_angular_speed(
            std::numbers::pi_v<float> / 2.0F, exploration_curve_radius, exploring_params.max_angular_acceleration,
            exploring_params.max_centrifugal_acceleration
        ),
        exploring_params.max_linear_speed, exploring_params.max_angular_acceleration
    )},
    turn_right{std::make_shared<TurnAction>(
        ActionType::TURN, -std::numbers::pi_v<float> / 2.0F,
        TurnAction::calculate_max_angular_speed(
            -std::numbers::pi_v<float> / 2.0F, exploration_curve_radius, exploring_params.max_angular_acceleration,
            exploring_params.max_centrifugal_acceleration
        ),
        exploring_params.max_linear_speed, exploring_params.max_angular_acceleration
    )},
    turn_back{std::make_shared<TurnAction>(
        ActionType::SPIN, std::numbers::pi_v<float>,
        TurnAction::calculate_max_angular_speed(
            std::numbers::pi_v<float>, 0.0F, exploring_params.max_angular_acceleration,
            exploring_params.max_centrifugal_acceleration
        ),
        0.0F, exploring_params.max_angular_acceleration
    )} {
    for (const auto& [angle, curve_radius] : {
             std::pair{std::numbers::pi_v<float> / 4.0F, config.radius_45},
             std::pair{std::numbers::pi_v<float> / 2.0F, config.radius_90},
             std::pair{3.0F * std::numbers::pi_v<float> / 4.0F, config.radius_135},
             std::pair{std::numbers::pi_v<float>, this->cell_size / 2.0F},
         }) {
        const float max_angular_speed = TurnAction::calculate_max_angular_speed(
            angle, curve_radius, config.solving.max_angular_acceleration, config.solving.max_centrifugal_acceleration
        );
        const float curve_linear_speed = this->solving_params.max_centrifugal_acceleration / max_angular_speed;

        this->curves_parameters[angle] = {
            .curve_radius = curve_radius,
            .max_angular_speed = max_angular_speed,
            .linear_speed = curve_linear_speed,
            .side_displacement = TurnAction::calculate_side_displacement(
                angle, max_angular_speed, curve_linear_speed, config.solving.max_angular_acceleration
            ),
            .forward_displacement = TurnAction::calculate_forward_displacement(
                angle, max_angular_speed, curve_linear_speed, config.solving.max_angular_acceleration
            ),
        };
    }
}

void ActionQueuer::push_exploring(const GridPose& origin_pose, const GridPoint& target_position) {
    if (origin_pose.front().position == target_position) {
        this->action_queue.emplace_back(move_forward);
        return;
    }

    if (origin_pose.turned_left().front().position == target_position) {
        this->action_queue.emplace_back(move_to_turn);
        this->action_queue.emplace_back(turn_left);
        this->action_queue.emplace_back(move_to_turn);
        return;
    }

    if (origin_pose.turned_right().front().position == target_position) {
        this->action_queue.emplace_back(move_to_turn);
        this->action_queue.emplace_back(turn_right);
        this->action_queue.emplace_back(move_to_turn);
        return;
    }

    if (origin_pose.turned_back().front().position == target_position) {
        this->action_queue.emplace_back(stop);
        this->action_queue.emplace_back(turn_back);
        this->action_queue.emplace_back(move_half);
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

void ActionQueuer::recompute(const std::list<GridPose>& best_route, bool add_start) {
    this->action_queue = {};

    if (add_start) {
        this->action_queue.emplace_back(start);
    }

    if (best_route.empty()) {
        return;
    }

    std::list<Action::Id> actions;

    for (auto route_it = best_route.begin(); std::next(route_it) != best_route.end(); route_it++) {
        actions.splice(actions.end(), get_actions(*route_it, *std::next(route_it), this->cell_size));
    }

    for (auto action_it = actions.begin(); action_it != actions.end(); action_it++) {
        if (action_it->type != ActionType::TURN) {
            continue;
        }

        if (std::next(action_it)->type == ActionType::TURN) {
            const float turn_angle = action_it->value;
            action_it = actions.erase(action_it);
            action_it->value += turn_angle;
        }

        auto [trim_before_distance, trim_after_distance] = this->get_trim_distances(*std::prev(action_it), *action_it);

        std::prev(action_it)->value -= trim_before_distance;
        std::next(action_it)->value -= trim_after_distance;
    }

    actions.front().value -= this->start_offset;
    float start_speed = 0.0F;

    for (auto action_it = std::next(actions.begin()); action_it != actions.end(); action_it++) {
        if (action_it->type != ActionType::TURN) {
            continue;
        }

        const auto& curve_parameters = this->curves_parameters.at(std::abs(action_it->value));

        this->action_queue.emplace_back(std::make_shared<MoveAction>(
            std::prev(action_it)->type, std::prev(action_it)->value, start_speed, curve_parameters.linear_speed,
            this->solving_params.max_linear_speed, this->solving_params.max_linear_acceleration,
            this->solving_params.max_linear_deceleration
        ));

        this->action_queue.emplace_back(std::make_shared<TurnAction>(
            action_it->type, action_it->value, curve_parameters.max_angular_speed, curve_parameters.linear_speed,
            this->solving_params.max_angular_acceleration
        ));

        start_speed = curve_parameters.linear_speed;
    }

    if (actions.back().type == ActionType::MOVE_FORWARD) {
        this->action_queue.emplace_back(std::make_shared<MoveAction>(
            ActionType::STOP, actions.back().value + this->cell_size / 2.0F, start_speed, 0.0F,
            this->solving_params.max_linear_speed, this->solving_params.max_linear_acceleration,
            this->solving_params.max_linear_deceleration
        ));
    } else {
        this->action_queue.emplace_back(std::make_shared<MoveAction>(
            ActionType::STOP, this->cell_size / 2.0F, start_speed, 0.0F, this->solving_params.max_linear_speed,
            this->solving_params.max_linear_acceleration, this->solving_params.max_linear_deceleration
        ));
    }
}

float ActionQueuer::get_total_time() const {
    float total_time = 0.0F;

    for (const auto& action : this->action_queue) {
        total_time += action->get_total_time();
    }

    return total_time;
}

std::list<Action::Id>
    ActionQueuer::get_actions(const GridPose& origin_pose, const GridPose& target_pose, float cell_size) {
    const Side relative_side = origin_pose.get_relative_side(target_pose.position);

    if (origin_pose.position == target_pose.position or relative_side == Side::DOWN) {
        return {};
    }

    if (relative_side == Side::UP and origin_pose.orientation == target_pose.orientation) {
        const float distance =
            origin_pose.position.to_vector(cell_size).distance(target_pose.position.to_vector(cell_size));
        return {{ActionType::MOVE_FORWARD, distance}};
    }

    if (origin_pose.turned_left().front() == target_pose) {
        return {{ActionType::TURN, std::numbers::pi_v<float> / 2.0F}};
    }

    if (origin_pose.turned_right().front() == target_pose) {
        return {{ActionType::TURN, -std::numbers::pi_v<float> / 2.0F}};
    }

    if (origin_pose.turned_left().front().turned_left().front() == target_pose) {
        return {{ActionType::TURN, std::numbers::pi_v<float>}};
    }

    if (origin_pose.turned_right().front().turned_right().front() == target_pose) {
        return {{ActionType::TURN, -std::numbers::pi_v<float>}};
    }

    const bool      keep_direction = (origin_pose.orientation == target_pose.orientation);
    const GridPoint target_position = (keep_direction ? target_pose : target_pose.turned_back().front()).position;

    if (std::abs(target_position.x - origin_pose.position.x) != std::abs(target_position.y - origin_pose.position.y) or
        origin_pose.turned_right().get_relative_side(target_position) != Side::LEFT) {
        return {};
    }

    if (not keep_direction and
        target_pose.orientation != (relative_side == Side::LEFT ? origin_pose.turned_left().orientation :
                                                                  origin_pose.turned_right().orientation)) {
        return {};
    }

    std::list<Action::Id> actions;

    float turn_angle =
        relative_side == Side::LEFT ? std::numbers::pi_v<float> / 4.0F : -std::numbers::pi_v<float> / 4.0F;

    actions.emplace_back(Action::Id{ActionType::TURN, turn_angle});

    float distance = origin_pose.position.to_vector(cell_size).distance(target_position.to_vector(cell_size));

    if (keep_direction) {
        turn_angle *= -1.0F;
    } else {
        distance += cell_size * std::numbers::sqrt2_v<float> / 2;
    }

    actions.emplace_back(Action::Id{ActionType::DIAGONAL, distance});
    actions.emplace_back(Action::Id{ActionType::TURN, turn_angle});

    return actions;
}

std::pair<float, float>
    ActionQueuer::get_trim_distances(const Action::Id& action_before, const Action::Id& turn_action) const {
    const float turn_angle = std::abs(turn_action.value);
    const auto& curve_parameters = this->curves_parameters.at(turn_angle);

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

    return {0.0F, 0.0F};
}
}  // namespace micras::nav
