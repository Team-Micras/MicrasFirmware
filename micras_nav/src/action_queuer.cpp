/**
 * @file
 */

#include <cmath>
#include <iterator>
#include <numbers>

#include "micras/nav/action_queuer.hpp"
#include "micras/nav/actions/move.hpp"

namespace micras::nav {
ActionQueuer::ActionQueuer(Config config) :
    cell_size{config.cell_size},
    start_offset{config.start_offset},
    curve_linear_speed{std::sqrt(config.solving.max_centrifugal_acceleration * config.solving.curve_radius)},
    straight_trim_distance{std::tan(std::numbers::pi_v<float> / 8.0F) * cell_size / 2.0F},
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

    if (std::abs(target_position.x - origin_pose.position.x) != std::abs(target_position.y - origin_pose.position.y)) {
        return;
    }

    float turn_angle =
        relative_side == Side::LEFT ? std::numbers::pi_v<float> / 4.0F : -std::numbers::pi_v<float> / 4.0F;

    this->action_queue.emplace_back(std::make_shared<TurnAction>(
        ActionType::TURN, turn_angle, this->cell_size / 2.0F, this->curve_linear_speed,
        this->solving_params.max_angular_acceleration
    ));

    float distance =
        origin_pose.position.to_vector(this->cell_size).distance(target_position.to_vector(this->cell_size)) -
        2.0F * this->straight_trim_distance;

    if (not keep_direction) {
        turn_angle *= -1.0F;
    } else {
        distance += std::numbers::sqrt2_v<float> / 2;
    }

    this->action_queue.emplace_back(std::make_shared<MoveAction>(
        ActionType::DIAGONAL, distance, this->curve_linear_speed, this->curve_linear_speed,
        this->solving_params.max_linear_speed, this->solving_params.max_linear_acceleration,
        this->solving_params.max_linear_deceleration
    ));

    this->action_queue.emplace_back(std::make_shared<TurnAction>(
        ActionType::TURN, turn_angle, this->cell_size / 2.0F, this->curve_linear_speed,
        this->solving_params.max_angular_acceleration
    ));
}

std::shared_ptr<Action> ActionQueuer::pop() {
    const auto& action = this->action_queue.front();
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

    for (auto route_it = std::next(best_route.begin()); std::next(route_it) != best_route.end(); route_it++) {
        this->push_solving(*route_it, *std::next(route_it));
    }

    float start_distance = this->start_offset;

    if (this->action_queue.front()->get_id().type == ActionType::MOVE_FORWARD) {
        start_distance += this->action_queue.front()->get_id().value;
        this->action_queue.pop_front();
    }

    this->action_queue.emplace_front(std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, start_distance, 0.0F, this->curve_linear_speed, this->solving_params.max_linear_speed,
        this->solving_params.max_linear_acceleration, this->solving_params.max_linear_deceleration
    ));
    this->action_queue.emplace_back(stop);

    for (auto action_it = this->action_queue.begin(); action_it != this->action_queue.end();) {
        if ((*action_it)->get_id().type != ActionType::DIAGONAL) {
            auto before_diagonal_it = std::prev(action_it, 2);

            if ((*before_diagonal_it)->get_id().type == ActionType::TURN) {
                action_it = std::next(this->join_curves(before_diagonal_it));
            }

            auto last_straight = *std::prev(action_it, 2);
            this->trim_straight(*last_straight);

            auto after_diagonal_it = std::next(action_it, 2);

            if ((*after_diagonal_it)->get_id().type == ActionType::TURN) {
                action_it = std::prev(this->join_curves(std::prev(after_diagonal_it)));
            }

            auto next_straight = *std::next(action_it, 2);
            this->trim_straight(*next_straight);
        }

        action_it++;
    }
}

float ActionQueuer::get_total_time() const {
    float total_time = 0.0F;

    for (const auto& action : this->action_queue) {
        total_time += action->get_total_time();
    }

    return total_time;
}

void ActionQueuer::trim_straight(Action& straight_action) const {
    if (straight_action.get_id().type == ActionType::DIAGONAL) {
        straight_action -= this->cell_size / 2.0F - this->straight_trim_distance;
    } else {
        straight_action -= this->straight_trim_distance;
    }
}

std::deque<std::shared_ptr<Action>>::iterator
    ActionQueuer::join_curves(std::deque<std::shared_ptr<Action>>::iterator turn_it) {
    const float turn_angle = (*turn_it)->get_id().value;

    turn_it = this->action_queue.erase(turn_it);
    (**turn_it) += turn_angle;

    return turn_it;
}
}  // namespace micras::nav
