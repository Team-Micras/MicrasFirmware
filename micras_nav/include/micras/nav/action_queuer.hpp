/**
 * @file
 */

#ifndef MICRAS_NAV_ACTION_QUEUER_HPP
#define MICRAS_NAV_ACTION_QUEUER_HPP

#include <deque>
#include <list>
#include <memory>
#include <unordered_map>

#include "micras/nav/actions/move.hpp"
#include "micras/nav/actions/turn.hpp"

namespace micras::nav {
/**
 * @brief Class to queue actions for the robot.
 */
class ActionQueuer {
public:
    /**
     * @brief Enum for the exploration action types.
     */
    enum ActionType : uint8_t {
        STOP = 0,
        START = 1,
        MOVE_FORWARD = 2,
        TURN = 3,
        SPIN = 4,
        DIAGONAL = 5,
    };

    /**
     * @brief Configuration struct for the ActionQueuer class.
     */
    struct Config {
        struct Dynamic {
            float max_linear_speed;
            float max_linear_acceleration;
            float max_linear_deceleration;
            float max_centrifugal_acceleration;
            float max_angular_acceleration;
        };

        float   cell_size;
        float   start_offset;
        Dynamic exploring;
        Dynamic solving;

        float radius_45;
        float radius_90;
        float radius_135;
    };

    /**
     * @brief Construct a new ActionQueuer object.
     *
     * @param config Configuration for the ActionQueuer.
     */
    explicit ActionQueuer(Config config);

    /**
     * @brief Push an exploring action to the queue.
     *
     * @param current_pose Current pose of the robot.
     * @param target_position Target position to move to.
     */
    void push_exploring(const GridPose& origin_pose, const GridPoint& target_position);

    /**
     * @brief Get the actions to perform from the origin pose to the target pose.
     *
     * @param origin_pose Origin pose of the robot.
     * @param target_pose Target pose to move to.
     * @return List of actions to perform.
     */
    std::list<Action::Id> get_actions(const GridPose& origin_pose, const GridPose& target_pose) const;

    /**
     * @brief Pop an action from the queue.
     *
     * @return Shared pointer to the action.
     */
    std::shared_ptr<Action> pop();

    /**
     * @brief Check if the action queue is empty.
     *
     * @return True if the action queue is empty, false otherwise.
     */
    bool empty() const;

    /**
     * @brief Fill the action queue with a sequence of actions to the end.
     */
    void recompute(const std::list<GridPose>& best_route, bool add_start = true);

    /**
     * @brief Get the total time to complete the actions in the queue.
     *
     * @return Total time to complete the actions in the queue.
     */
    float get_total_time() const;

private:
    /**
     * @brief Struct to hold the parameters for a curve.
     */
    struct CurveParameters {
        float curve_radius;
        float max_angular_speed;
        float linear_speed;
        float side_displacement;
        float forward_displacement;
    };

    /**
     * @brief Get the trim distances for a turn action.
     *
     * @param action_before Id of the action before the turn action.
     * @param turn_action Turn action to get the trim distances for.
     * @return Pair of trim distances before and after the turn.
     */
    std::pair<float, float> get_trim_distances(const Action::Id& action_before, const Action::Id& turn_action) const;

    /**
     * @brief Size of the cells in the grid.
     */
    float cell_size;

    /**
     * @brief Distance of the robot to the on on the start cell.
     */
    float start_offset;

    /**
     * @brief Dynamic exploring parameters.
     */
    Config::Dynamic exploring_params;

    /**
     * @brief Dynamic solving parameters.
     */
    Config::Dynamic solving_params;

    /**
     * @brief Radius of the exploration curve.
     */
    float exploration_curve_radius;

    /**
     * @brief Pre-computed curve parameters for different angles.
     */
    std::unordered_map<float, CurveParameters> curves_parameters;

    /**
     * @brief Pre-built actions to use in the exploration.
     */
    ///@{
    std::shared_ptr<MoveAction> stop;
    std::shared_ptr<MoveAction> start;
    std::shared_ptr<MoveAction> move_forward;
    std::shared_ptr<MoveAction> move_half;
    std::shared_ptr<MoveAction> move_to_turn;
    std::shared_ptr<TurnAction> turn_left;
    std::shared_ptr<TurnAction> turn_right;
    std::shared_ptr<TurnAction> turn_back;
    ///@}

    /**
     * @brief Queue of actions to be performed.
     */
    std::deque<std::shared_ptr<Action>> action_queue;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ACTION_QUEUER_HPP
