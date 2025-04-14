/**
 * @file
 */

#include <numbers>

#include "micras/nav/action_queuer.hpp"

namespace micras::nav {
ActionQueuer::ActionQueuer(Config config) :
    cell_size{config.cell_size},
    start_offset{config.start_offset},
    exploring_params{config.exploring},
    solving_params{config.solving},
    start{std::make_shared<MoveAction>(
        cell_size - start_offset, 0.0F, exploring_params.max_speed, exploring_params.max_speed,
        exploring_params.max_acceleration, exploring_params.max_deceleration
    )},
    move_forward{std::make_shared<MoveAction>(
        cell_size, exploring_params.max_speed, exploring_params.max_speed, exploring_params.max_speed,
        exploring_params.max_acceleration, exploring_params.max_deceleration
    )},
    turn_left{std::make_shared<TurnAction>(
        std::numbers::pi_v<float> / 2.0F, cell_size / 2.0F, exploring_params.max_centrifugal_acceleration
    )},
    turn_right{std::make_shared<TurnAction>(
        -std::numbers::pi_v<float> / 2.0F, cell_size / 2.0F, exploring_params.max_centrifugal_acceleration
    )},
    turn_back{std::make_shared<TurnAction>(
        std::numbers::pi_v<float>, cell_size / 2.0F, exploring_params.max_centrifugal_acceleration
    )},
    action_queue{{start}} { }

void ActionQueuer::push(const GridPose& current_pose, const GridPose& target_pose) {
    if (current_pose.front() == target_pose) {
        this->action_queue.emplace(move_forward);
        return;
    }

    if (current_pose.turned_left().front() == target_pose) {
        this->action_queue.emplace(turn_left);
        return;
    }

    if (current_pose.turned_right().front() == target_pose) {
        this->action_queue.emplace(turn_right);
        return;
    }

    if (current_pose.turned_back().front() == target_pose) {
        this->action_queue.emplace(turn_back);
        this->action_queue.emplace(move_forward);
        return;
    }
}

const std::shared_ptr<Action>& ActionQueuer::pop() {
    auto action = this->action_queue.front();
    this->action_queue.pop();
    return action;
}

bool ActionQueuer::empty() const {
    return this->action_queue.empty();
}

void ActionQueuer::recalculate(const std::map<uint16_t, GridPose, std::greater<>>& best_route) {
    return;
}
}  // namespace micras::nav
