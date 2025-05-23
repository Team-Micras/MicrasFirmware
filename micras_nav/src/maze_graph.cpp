/**
 * @file
 */

#include <cmath>
#include <limits>
#include <queue>
#include "micras/nav/action_queuer.hpp"

#include "micras/nav/maze_graph.hpp"

namespace micras::nav {
MazeGraph::MazeGraph(Config config) :
    cell_size{config.cell_size},
    cost_params{config.cost_params},
    curve_radius{this->cell_size / 2.0F},
    curve_linear_speed{std::sqrt(cost_params.max_centrifugal_acceleration * curve_radius)} { }

bool MazeGraph::has_node(const GridPose& pose) const {
    return this->graph.contains(pose);
}

void MazeGraph::add_node(const GridPose& pose) {
    this->graph.emplace(pose, Node{pose});
}

void MazeGraph::remove_node(const GridPose& pose) {
    for (const GridPose& next_pose : this->graph.at(pose).next) {
        if (this->graph.at(next_pose).prev.size() == 1) {
            this->remove_node(next_pose);
        } else {
            this->graph.at(next_pose).prev.erase(pose);
        }
    }

    this->graph.erase(pose);
}

void MazeGraph::add_edge(const GridPose& from, const GridPose& to) {
    if (this->graph.at(from).next.contains(to)) {
        return;
    }

    this->graph.at(from).next.emplace(to);
    this->graph.at(to).prev.emplace(from);
}

void MazeGraph::remove_edge(const GridPose& from, const GridPose& to) {
    this->graph.at(from).next.erase(to);
    this->graph.at(to).prev.erase(from);

    if (this->graph.at(to).prev.empty()) {
        this->remove_node(to);
    }
}

void MazeGraph::process_nodes(const GridPose& start) {
    this->add_extra_edges(start);
}

void MazeGraph::add_extra_edges(const GridPose& start) {
    std::unordered_set<GridPose> visited;
    std::queue<GridPose>         queue;
    queue.push(start);

    while (not queue.empty()) {
        const GridPose current_pose = queue.front();
        Node&          current_node = this->graph.at(current_pose);
        queue.pop();

        visited.insert(current_pose);
        bool skipped = false;

        for (auto next_it = current_node.next.begin(); next_it != current_node.next.end();) {
            GridPose next_pose = *next_it;

            for (const GridPose& next_next_pose : this->graph.at(next_pose).next) {
                if (current_node.next.contains(next_next_pose)) {
                    continue;
                }

                if (this->can_skip(current_pose, next_pose, next_next_pose)) {
                    skipped = true;
                    this->add_edge(current_pose, next_next_pose);
                    next_it = current_node.next.begin();

                    if (this->graph.at(next_pose).next.size() == 1) {
                        this->remove_edge(current_pose, next_pose);
                        next_it = current_node.next.begin();
                        break;
                    }
                }
            }

            if (skipped) {
                skipped = false;
                continue;
            }

            if (not visited.contains(next_pose)) {
                queue.push(next_pose);
            }

            next_it++;
        }
    }
}

std::list<GridPose> MazeGraph::get_best_route(const GridPose& start, const std::unordered_set<GridPoint>& end) {
    std::unordered_map<GridPose, float>    distance;
    std::unordered_map<GridPose, GridPose> previous;
    std::unordered_set<GridPose>           visited;

    auto comp_function = [&distance](const GridPose& first, const GridPose& second) {
        return distance.at(first) > distance.at(second);
    };

    std::priority_queue<GridPose, std::deque<GridPose>, decltype(comp_function)> queue(comp_function);

    for (const auto& [pose, node] : this->graph) {
        distance[pose] = std::numeric_limits<float>::max();
    }

    distance[start] = 0;
    queue.push(start);
    GridPose current_pose = start;

    while (!queue.empty()) {
        current_pose = queue.top();
        queue.pop();

        if (visited.contains(current_pose)) {
            continue;
        }

        if (end.contains(current_pose.position)) {
            break;
        }

        visited.insert(current_pose);

        for (const GridPose& next_pose : this->graph.at(current_pose).next) {
            float edge_cost = this->get_edge_cost(current_pose, next_pose);

            if (distance.at(next_pose) > distance.at(current_pose) + edge_cost) {
                previous[next_pose] = current_pose;
                distance[next_pose] = distance.at(current_pose) + edge_cost;
                queue.push(next_pose);
            }
        }
    }

    std::list<GridPose> route;

    while (current_pose != start) {
        route.push_front(current_pose);
        current_pose = previous.at(current_pose);
    }

    route.push_front(start);

    return route;
}

void MazeGraph::reset() {
    this->graph.clear();
}

bool MazeGraph::can_skip(const GridPose& first, const GridPose& second, const GridPose& third) const {
    auto skip_action = this->get_main_action_type(first, third);

    if (skip_action == ActionQueuer::ActionType::STOP) {
        return false;
    }

    if (skip_action != ActionQueuer::ActionType::MOVE_FORWARD) {
        return true;
    }

    return this->get_main_action_type(first, second) == ActionQueuer::ActionType::MOVE_FORWARD;
}

float MazeGraph::get_edge_cost(const GridPose& from, const GridPose& to) const {
    auto  actions = ActionQueuer::get_actions(from, to, this->cell_size);
    float total_time = 0.0F;

    for (const auto& action : actions) {
        if (action.type == ActionQueuer::ActionType::TURN) {
            const float max_angular_speed = TurnAction::calculate_max_angular_speed(
                action.value, this->curve_radius, this->cost_params.max_angular_acceleration,
                this->cost_params.max_centrifugal_acceleration
            );

            total_time += TurnAction::calculate_total_time(
                action.value, max_angular_speed, this->cost_params.max_angular_acceleration
            );
        } else {
            total_time += MoveAction::calculate_total_time(
                action.value, this->curve_linear_speed, this->curve_linear_speed, this->cost_params.max_linear_speed,
                this->cost_params.max_linear_acceleration, this->cost_params.max_linear_deceleration
            );
        }
    }

    return total_time;
}

ActionQueuer::ActionType MazeGraph::get_main_action_type(const GridPose& from, const GridPose& to) const {
    auto actions = ActionQueuer::get_actions(from, to, this->cell_size);

    if (actions.empty()) {
        return ActionQueuer::ActionType::STOP;
    }

    if (actions.size() == 3) {
        return ActionQueuer::ActionType::DIAGONAL;
    }

    return static_cast<ActionQueuer::ActionType>(actions.front().type);
}
}  // namespace micras::nav
