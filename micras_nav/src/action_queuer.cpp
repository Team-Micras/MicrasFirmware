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
    curve_linear_speed{std::sqrt(config.solving.max_centrifugal_acceleration * config.solving.curve_radius)},
    exploring_params{config.exploring},
    solving_params{config.solving},
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
    turn_left{std::make_shared<TurnAction>(
        ActionType::TURN, std::numbers::pi_v<float> / 2.0F, cell_size / 2.0F, exploring_params.max_linear_speed,
        exploring_params.max_angular_acceleration
    )},
    turn_right{std::make_shared<TurnAction>(
        ActionType::TURN, -std::numbers::pi_v<float> / 2.0F, cell_size / 2.0F, exploring_params.max_linear_speed,
        exploring_params.max_angular_acceleration
    )},
    turn_back{std::make_shared<TurnAction>(
        ActionType::SPIN, std::numbers::pi_v<float>, 0.0F, 0.0F, exploring_params.max_angular_acceleration
    )} { }

void ActionQueuer::push_exploring(const GridPose& origin_pose, const GridPoint& target_position) {
    if (origin_pose.front().position == target_position) {
        this->action_queue.emplace_back(move_forward);
        return;
    }

    if (origin_pose.turned_left().front().position == target_position) {
        this->action_queue.emplace_back(turn_left);
        return;
    }

    if (origin_pose.turned_right().front().position == target_position) {
        this->action_queue.emplace_back(turn_right);
        return;
    }

    if (origin_pose.turned_back().front().position == target_position) {
        this->action_queue.emplace_back(stop);
        this->action_queue.emplace_back(turn_back);
        this->action_queue.emplace_back(move_half);
        return;
    }
}

void ActionQueuer::push_solving(const GridPose& origin_pose, const GridPose& target_pose) {
    const Side relative_side = origin_pose.get_relative_side(target_pose.position);

    if (origin_pose.position == target_pose.position or relative_side == Side::DOWN) {
        return;
    }

    if (relative_side == Side::UP and origin_pose.orientation == target_pose.orientation) {
        const float distance =
            origin_pose.position.to_vector(this->cell_size).distance(target_pose.position.to_vector(this->cell_size));
        this->action_queue.emplace_back(std::make_shared<MoveAction>(
            ActionType::MOVE_FORWARD, distance, this->curve_linear_speed, this->curve_linear_speed,
            this->solving_params.max_linear_speed, this->solving_params.max_linear_acceleration,
            this->solving_params.max_linear_deceleration
        ));
    }

    if (origin_pose.turned_left().front() == target_pose) {
        this->action_queue.emplace_back(std::make_shared<TurnAction>(
            ActionType::TURN, std::numbers::pi_v<float> / 2.0F, this->cell_size / 2.0F, this->curve_linear_speed,
            this->solving_params.max_angular_acceleration
        ));
        return;
    }

    if (origin_pose.turned_right().front() == target_pose) {
        this->action_queue.emplace_back(std::make_shared<TurnAction>(
            ActionType::TURN, -std::numbers::pi_v<float> / 2.0F, this->cell_size / 2.0F, this->curve_linear_speed,
            this->solving_params.max_angular_acceleration
        ));
        return;
    }

    if (origin_pose.turned_left().front().turned_left().front() == target_pose) {
        this->action_queue.emplace_back(std::make_shared<TurnAction>(
            ActionType::TURN, std::numbers::pi_v<float>, this->cell_size / 2.0F, this->curve_linear_speed,
            this->solving_params.max_angular_acceleration
        ));
        return;
    }

    if (origin_pose.turned_right().front().turned_right().front() == target_pose) {
        this->action_queue.emplace_back(std::make_shared<TurnAction>(
            ActionType::TURN, -std::numbers::pi_v<float>, this->cell_size / 2.0F, this->curve_linear_speed,
            this->solving_params.max_angular_acceleration
        ));
        return;
    }

    const bool      keep_direction = (origin_pose.orientation == target_pose.orientation);
    const GridPoint target_position = (keep_direction ? target_pose : target_pose.turned_back().front()).position;

    if (std::abs(target_position.x - origin_pose.position.x) != std::abs(target_position.y - origin_pose.position.y) or
        origin_pose.turned_right().get_relative_side(target_position) != Side::LEFT) {
        return;
    }

    if (not keep_direction and
        target_pose.orientation != (relative_side == Side::LEFT ? origin_pose.turned_left().orientation :
                                                                  origin_pose.turned_right().orientation)) {
        return;
    }

    float turn_angle =
        relative_side == Side::LEFT ? std::numbers::pi_v<float> / 4.0F : -std::numbers::pi_v<float> / 4.0F;

    this->action_queue.emplace_back(std::make_shared<TurnAction>(
        ActionType::TURN, turn_angle, this->cell_size / 2.0F, this->curve_linear_speed,
        this->solving_params.max_angular_acceleration
    ));

    float distance =
        origin_pose.position.to_vector(this->cell_size).distance(target_position.to_vector(this->cell_size));

    if (keep_direction) {
        turn_angle *= -1.0F;
    } else {
        distance += this->cell_size * std::numbers::sqrt2_v<float> / 2;
    }

    this->action_queue.emplace_back(std::make_shared<MoveAction>(
        ActionType::DIAGONAL, distance, this->curve_linear_speed, this->curve_linear_speed,
        this->solving_params.max_linear_speed, this->solving_params.max_linear_acceleration,
        this->solving_params.max_linear_deceleration, false
    ));

    this->action_queue.emplace_back(std::make_shared<TurnAction>(
        ActionType::TURN, turn_angle, this->cell_size / 2.0F, this->curve_linear_speed,
        this->solving_params.max_angular_acceleration
    ));
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

    for (auto route_it = best_route.begin(); std::next(route_it) != best_route.end(); route_it++) {
        this->push_solving(*route_it, *std::next(route_it));
    }

    float start_distance = this->cell_size - this->start_offset;

    if (this->action_queue.front()->get_id().type == ActionType::MOVE_FORWARD) {
        start_distance += this->action_queue.front()->get_id().value - this->cell_size;
        this->action_queue.pop_front();
    }

    this->action_queue.emplace_front(std::make_shared<MoveAction>(
        ActionType::START, start_distance, 0.0F, this->curve_linear_speed, this->solving_params.max_linear_speed,
        this->solving_params.max_linear_acceleration, this->solving_params.max_linear_deceleration
    ));
    this->action_queue.emplace_back(stop);

    for (auto action_it = this->action_queue.begin(); action_it != this->action_queue.end(); action_it++) {
        if ((*action_it)->get_id().type != ActionType::TURN or
            std::abs((*action_it)->get_id().value) != std::numbers::pi_v<float> / 4.0F) {
            continue;
        }

        if ((*std::prev(action_it))->get_id().type == ActionType::TURN) {
            action_it = this->join_curves(std::prev(action_it));
        } else if ((*std::next(action_it))->get_id().type == ActionType::TURN) {
            action_it = this->join_curves(action_it);
        }

        auto [trim_before_distance, trim_after_distance] = this->get_trim_distances(**action_it);

        **std::prev(action_it) -= trim_before_distance;
        **std::next(action_it) -= trim_after_distance;
    }
}

float ActionQueuer::get_total_time() const {
    float total_time = 0.0F;

    for (const auto& action : this->action_queue) {
        total_time += action->get_total_time();
    }

    return total_time;
}

std::pair<float, float> ActionQueuer::get_trim_distances(const Action& turn_action) const {
    const float turn_angle = std::abs(turn_action.get_id().value);
    const float side_displacement = (1.0F - std::cos(turn_angle)) * this->cell_size / 2.0F;

    const float correction_factor = 14.12F - 13.3F * std::cos(turn_angle - 1.145F);
    const float max_angular_speed = TurnAction::calculate_max_angular_speed(
        turn_angle, this->cell_size / 2.0F, this->curve_linear_speed, this->solving_params.max_angular_acceleration
    );
    const float forward_displacement =
        std::sin(turn_angle) / max_angular_speed +
        max_angular_speed / (std::sin(turn_angle) * correction_factor * this->solving_params.max_angular_acceleration);

    if (turn_angle == std::numbers::pi_v<float> / 4.0F) {
        const float trim_before_distance = forward_displacement - side_displacement;
        const float trim_after_distance = side_displacement * std::numbers::sqrt2_v<float>;

        return {trim_before_distance, trim_after_distance};
    }

    if (turn_angle == std::numbers::pi_v<float> / 2.0F) {
        const float trim_distance = side_displacement;

        return {trim_distance, trim_distance};
    }

    if (turn_angle == 3.0F * std::numbers::pi_v<float> / 4.0F) {
        const float trim_before_distance =
            forward_displacement + side_displacement - (this->cell_size * std::numbers::sqrt2_v<float> / 2.0F);
        const float trim_after_distance = std::numbers::sqrt2_v<float> * side_displacement - this->cell_size;

        return {trim_before_distance, trim_after_distance};
    }

    return {0.0F, 0.0F};
}

std::deque<std::shared_ptr<Action>>::iterator
    ActionQueuer::join_curves(std::deque<std::shared_ptr<Action>>::iterator turn_it) {
    const float turn_angle = (*turn_it)->get_id().value;

    turn_it = this->action_queue.erase(turn_it);
    (**turn_it) += turn_angle;

    return turn_it;
}
}  // namespace micras::nav
