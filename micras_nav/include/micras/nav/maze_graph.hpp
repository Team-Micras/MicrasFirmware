/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_GRAPH_HPP
#define MICRAS_NAV_MAZE_GRAPH_HPP

#include <unordered_map>
#include <unordered_set>

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
using EdgeCostFunction = std::function<float(const GridPose&, const GridPose&)>;

/**
 * @brief
 */
class MazeGraph {
public:
    struct Node {
        explicit Node(GridPose pose) : pose(pose){};

        GridPose                            pose;
        std::unordered_map<GridPose, float> next_costs;
        std::unordered_set<GridPose>        prev;
    };

    explicit MazeGraph(EdgeCostFunction edge_cost_function = [](const GridPose& from, const GridPose& to) -> float {
        return std::abs(from.position.x - to.position.x) + std::abs(from.position.y - to.position.y);
    });

    bool has_node(const GridPose& pose) const;

    void add_node(const GridPose& pose);

    void add_edge(const GridPose& from, const GridPose& to, float cost = 0.0F);

    void process_nodes(const GridPose& start);

private:
    void remove_extra_nodes(const GridPose& start);

    void add_extra_edges(const GridPose& start);

    void dijkstra(const GridPose& start, const GridPose& end);

    EdgeCostFunction get_edge_cost;

    /**
     * @brief
     */
    std::unordered_map<GridPose, Node> graph;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_GRAPH_HPP
