/**
 * @file
 */

#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

#include "micras/nav/maze_graph.hpp"
#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
MazeGraph::MazeGraph(EdgeCostFunction edge_cost_function) : get_edge_cost(std::move(edge_cost_function)) { }

bool MazeGraph::has_node(const GridPose& pose) const {
    return this->graph.contains(pose);
}

void MazeGraph::add_node(const GridPose& pose) {
    this->graph.emplace(pose, Node{pose});
}

void MazeGraph::add_edge(const GridPose& from, const GridPose& to, float cost) {
    if (this->graph.at(from).next_costs.contains(to)) {
        return;
    }

    this->graph.at(from).next_costs[to] = cost;
    this->graph.at(to).prev.emplace(from);
}

void MazeGraph::process_nodes(const GridPose& start) {
    this->remove_extra_nodes(start);
    this->add_extra_edges(start);
}

void MazeGraph::remove_extra_nodes(const GridPose& start) {
    std::unordered_set<GridPose> visited;
    std::queue<GridPose>         queue;
    queue.push(start);

    while (not queue.empty()) {
        const GridPose current_pose = queue.front();
        Node&          current_node = this->graph.at(current_pose);
        queue.pop();

        visited.insert(current_pose);

        for (auto next_it = current_node.next_costs.begin(); next_it != current_node.next_costs.end();) {
            Node& next_node = this->graph.at(next_it->first);

            if (next_node.next_costs.size() == 1) {
                const GridPose& next_next = next_node.next_costs.begin()->first;

                if (next_next.orientation == current_pose.orientation or
                    next_next.orientation == current_pose.turned_back().orientation) {
                    current_node.next_costs.erase(next_it);
                    next_node.prev.erase(current_pose);

                    this->add_edge(current_pose, next_next);

                    if (next_node.prev.empty()) {
                        this->graph.erase(next_node.pose);
                        this->graph.at(next_next).prev.erase(next_node.pose);
                    }

                    next_it = current_node.next_costs.begin();
                    continue;
                }
            }

            if (not visited.contains(next_it->first)) {
                queue.push(next_it->first);
            }

            next_it++;
        }
    }
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

        for (auto next_it = current_node.next_costs.begin(); next_it != current_node.next_costs.end();) {
            if (next_it->second == 0) {
                next_it->second = this->get_edge_cost(current_pose, next_it->first);
            }

            GridPose next_pose = next_it->first;

            for (const auto& [next_next_pose, next_next_cost] : this->graph.at(next_pose).next_costs) {
                if (current_node.next_costs.contains(next_next_pose)) {
                    continue;
                }

                const float edge_cost = this->get_edge_cost(current_pose, next_next_pose);

                if (edge_cost > 0) {
                    this->add_edge(current_pose, next_next_pose, edge_cost);
                    next_it = current_node.next_costs.begin();
                }
            }

            if (not visited.contains(next_it->first)) {
                queue.push(next_it->first);
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
        queue.pop();
        current_pose = queue.top();

        if (visited.contains(current_pose)) {
            continue;
        }

        if (end.contains(current_pose.position)) {
            break;
        }

        visited.insert(current_pose);

        for (const auto& [next_pose, edge_cost] : this->graph.at(current_pose).next_costs) {
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

    return route;
}
}  // namespace micras::nav
