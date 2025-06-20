/**
 * @file
 */

#include <numbers>

#include "micras/nav/action_queuer.hpp"

namespace micras::nav {
ActionQueuer::ActionQueuer(Config config) :
    cell_size{config.cell_size},
    exploring_params{config.exploring},
    // solving_params{config.solving},
    stop{std::make_shared<MoveAction>(
        ActionType::STOP, cell_size / 2.0F, exploring_params.max_linear_speed, 0.0F, exploring_params.max_linear_speed,
        exploring_params.max_linear_acceleration, exploring_params.max_linear_deceleration, false
    )},
    start{std::make_shared<MoveAction>(
        ActionType::START, cell_size - config.start_offset, 0.001F * exploring_params.max_linear_acceleration,
        exploring_params.max_linear_speed, exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration
    )},
    move_forward{std::make_shared<MoveAction>(
        ActionType::MOVE_FORWARD, cell_size, exploring_params.max_linear_speed, exploring_params.max_linear_speed,
        exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration
    )},
    move_half{std::make_shared<MoveAction>(
        ActionType::MOVE_HALF, cell_size / 2.0F, 0.001F * exploring_params.max_linear_acceleration,
        exploring_params.max_linear_speed, exploring_params.max_linear_speed, exploring_params.max_linear_acceleration,
        exploring_params.max_linear_deceleration, false
    )},
    turn_left{std::make_shared<TurnAction>(
        ActionType::TURN_LEFT, std::numbers::pi_v<float> / 2.0F, cell_size / 2.0F, exploring_params.max_linear_speed,
        exploring_params.max_angular_acceleration
    )},
    turn_right{std::make_shared<TurnAction>(
        ActionType::TURN_RIGHT, -std::numbers::pi_v<float> / 2.0F, cell_size / 2.0F, exploring_params.max_linear_speed,
        exploring_params.max_angular_acceleration
    )},
    turn_back{std::make_shared<TurnAction>(
        ActionType::TURN_BACK, std::numbers::pi_v<float>, 0.0F, 0.0F, exploring_params.max_angular_acceleration
    )} { }

void ActionQueuer::push(const GridPose& current_pose, const GridPoint& target_position) {
    if (current_pose.front().position == target_position) {
        this->action_queue.emplace(move_forward);
        return;
    }

    if (current_pose.turned_left().front().position == target_position) {
        this->action_queue.emplace(turn_left);
        return;
    }

    if (current_pose.turned_right().front().position == target_position) {
        this->action_queue.emplace(turn_right);
        return;
    }

    if (current_pose.turned_back().front().position == target_position) {
        this->action_queue.emplace(stop);
        this->action_queue.emplace(turn_back);
        this->action_queue.emplace(move_half);
        return;
    }
}

std::shared_ptr<Action> ActionQueuer::pop() {
    const auto& action = this->action_queue.front();
    this->action_queue.pop();
    return action;
}

bool ActionQueuer::empty() const {
    return this->action_queue.empty();
}

void ActionQueuer::recompute(const std::list<GridPose>& best_route) {
    this->action_queue = {};
    this->action_queue.emplace(start);

    if (best_route.empty()) {
        return;
    }

    for (auto it = std::next(best_route.begin()); std::next(it) != best_route.end(); it++) {
        this->push(*it, std::next(it)->position);
    }
}
}  // namespace micras::nav
