/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_GRAPH_HPP
#define MICRAS_NAV_MAZE_GRAPH_HPP

#include <list>
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

    /**
     * @brief Construct a new MazeGraph object.
     *
     * @param edge_cost_function Function to compute the cost of an edge.
     */
    explicit MazeGraph(EdgeCostFunction edge_cost_function = [](const GridPose& from, const GridPose& to) -> float {
        float edge_cost = std::abs(from.position.x - to.position.x) + std::abs(from.position.y - to.position.y);
        return edge_cost > 1.0F ? 0.0F : edge_cost;
    });

    /**
     * @brief Check if the graph has a node for a given pose.
     *
     * @param pose The pose to check.
     * @return true if the graph has a node for the given pose, false otherwise.
     */
    bool has_node(const GridPose& pose) const;

    /**
     * @brief Add a new node to the graph.
     *
     * @param pose The grid pose that represents the node.
     */
    void add_node(const GridPose& pose);

    /**
     * @brief Add a new edge connecting two poses.
     *
     * @param from The starting node of the edge.
     * @param to The ending node of the edge.
     * @param cost The cost of the edge.
     */
    void add_edge(const GridPose& from, const GridPose& to, float cost = 0.0F);

    /**
     * @brief Process the nodes in the graph.
     *
     * @param start The starting node.
     */
    void process_nodes(const GridPose& start);

    /**
     * @brief Compute the best route on the graph using the dijkstra algorithm.
     *
     * @param start The starting node.
     * @param end A set of ending nodes.
     * @return A list of poses representing the best route from start to end.
     */
    std::list<GridPose> get_best_route(const GridPose& start, const std::unordered_set<GridPoint>& end);

private:
    /**
     * @brief Remove unnecessary nodes from the graph.
     *
     * @param start The starting node.
     */
    void remove_extra_nodes(const GridPose& start);

    /**
     * @brief Add extra edges representing diagonals, long curves and long straights to the graph.
     *
     * @param start The starting node.
     */
    void add_extra_edges(const GridPose& start);

    /**
     * @brief Function to compute the cost of an edge.
     *
     * @param from The starting node of the edge.
     * @param to The ending node of the edge.
     * @return The cost of the edge.
     */
    EdgeCostFunction get_edge_cost;

    /**
     * @brief
     */
    std::unordered_map<GridPose, Node> graph;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_GRAPH_HPP
