/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_GRAPH_HPP
#define MICRAS_NAV_MAZE_GRAPH_HPP

#include <list>
#include <unordered_map>
#include <unordered_set>

#include "micras/nav/action_queuer.hpp"
#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
/**
 * @brief
 */
class MazeGraph {
public:
    /**
     * @brief Node structure representing a node in the maze graph.
     */
    struct Node {
        explicit Node(GridPose pose) : pose(pose){};

        GridPose                     pose;
        std::unordered_set<GridPose> next;
        std::unordered_set<GridPose> prev;
    };

    /**
     * @brief Configuration structure for the maze graph.
     */
    struct Config {
        float                         cell_size;
        ActionQueuer::Config::Dynamic cost_params;
    };

    /**
     * @brief Construct a new MazeGraph object.
     *
     * @param config The configuration for the maze graph.
     */
    explicit MazeGraph(Config config);

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
     * @brief Remove a node from the graph.
     *
     * @param pose The grid pose that represents the node.
     */
    void remove_node(const GridPose& pose);

    /**
     * @brief Add a new edge connecting two poses.
     *
     * @param from The starting node of the edge.
     * @param to The ending node of the edge.
     * @param cost The cost of the edge.
     */
    void add_edge(const GridPose& from, const GridPose& to);

    /**
     * @brief Remove an edge connecting two poses.
     *
     * @param from The starting node of the edge.
     * @param to The ending node of the edge.
     */
    void remove_edge(const GridPose& from, const GridPose& to);

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

    /**
     * @brief Clear the contents of the graph.
     */
    void reset();

private:
    /**
     * @brief Add extra edges representing diagonals, long curves and long straights to the graph.
     *
     * @param start The starting node.
     */
    void add_extra_edges(const GridPose& start);

    /**
     * @brief Check if a skip is possible between three poses.
     *
     * @param first The first pose.
     * @param second The second pose.
     * @param third The third pose.
     * @return true if a skip is possible, false otherwise.
     */
    bool can_skip(const GridPose& first, const GridPose& second, const GridPose& third) const;

    /**
     * @brief Get the cost of an edge between two poses.
     *
     * @param from The starting pose.
     * @param to The ending pose.
     * @return The cost of the edge.
     */
    float get_edge_cost(const GridPose& from, const GridPose& to) const;

    /**
     * @brief Get the main action type that connects two grid poses.
     *
     * @param from The starting pose.
     * @param to The ending pose.
     * @return The main action type.
     */
    ActionQueuer::ActionType get_main_action_type(const GridPose& from, const GridPose& to) const;

    /**
     * @brief The size of the cells in the maze.
     */
    float cell_size;

    /**
     * @brief The configuration for calculating the edge costs.
     */
    ActionQueuer::Config::Dynamic cost_params;

    /**
     * @brief The radius of the curve when performing a turn.
     */
    float curve_radius;

    /**
     * @brief The linear speed of the robot while turning.
     */
    float curve_linear_speed;

    /**
     * @brief
     */
    std::unordered_map<GridPose, Node> graph;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_GRAPH_HPP
