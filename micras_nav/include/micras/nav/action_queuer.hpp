/**
 * @file
 */

#ifndef MICRAS_NAV_ACTION_QUEUER_HPP
#define MICRAS_NAV_ACTION_QUEUER_HPP

#include <map>
#include <memory>
#include <queue>

#include "micras/nav/actions/move.hpp"
#include "micras/nav/actions/turn.hpp"

namespace micras::nav {
/**
 * @brief Class to queue actions for the robot.
 */
class ActionQueuer {
public:
    /**
     * @brief Configuration struct for the ActionQueuer class.
     */
    struct Config {
        struct Dynamic {
            float max_linear_speed;
            float max_linear_acceleration;
            float max_linear_deceleration;
            float curve_radius;
            float max_centrifugal_acceleration;
            float max_angular_acceleration;
        };

        float   cell_size;
        float   start_offset;
        Dynamic exploring;
        Dynamic solving;
    };

    /**
     * @brief Construct a new ActionQueuer object.
     *
     * @param config Configuration for the ActionQueuer.
     */
    explicit ActionQueuer(Config config);

    /**
     * @brief Push an action to the queue.
     *
     * @param current_pose Current pose of the robot.
     * @param target_position Target position to move to.
     */
    void push(const GridPose& current_pose, const GridPoint& target_position);

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
    void recompute(const std::map<uint16_t, GridPose, std::greater<>>& best_route);

private:
    /**
     * @brief Size of the cells in the grid.
     */
    float cell_size;

    /**
     * @brief Dynamic exploring parameters.
     */
    Config::Dynamic exploring_params;

    /**
     * @brief Dynamic solving parameters.
     */
    // Config::Dynamic solving_params;

    /**
     * @brief Pre-built actions to use in the exploration.
     */
    ///@{
    std::shared_ptr<MoveAction> start;
    std::shared_ptr<MoveAction> move_forward;
    std::shared_ptr<MoveAction> stop;
    std::shared_ptr<MoveAction> move_half;
    std::shared_ptr<TurnAction> turn_left;
    std::shared_ptr<TurnAction> turn_right;
    std::shared_ptr<TurnAction> turn_back;
    ///@}

    /**
     * @brief Queue of actions to be performed.
     */
    std::queue<std::shared_ptr<Action>> action_queue;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ACTION_QUEUER_HPP
