/**
 * @file
 */

#ifndef MICRAS_NAV_ACTION_QUEUER_HPP
#define MICRAS_NAV_ACTION_QUEUER_HPP

#include <list>
#include <memory>
#include <queue>

#include "micras/nav/actions/move.hpp"
#include "micras/nav/actions/turn.hpp"

namespace micras::nav {
/**
 * @brief Class to follow the side walls using a PID controller.
 */
class ActionQueuer {
public:
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

    explicit ActionQueuer(Config config);

    void push(const GridPose& current_pose, const GridPoint& target_position);

    std::shared_ptr<Action> pop();

    bool empty() const;

    void recalculate(const std::list<GridPose>& best_route);

private:
    float           cell_size;
    float           start_offset;
    Config::Dynamic exploring_params;
    // Config::Dynamic solving_params;

    std::shared_ptr<MoveAction> start;
    std::shared_ptr<MoveAction> move_forward;
    std::shared_ptr<TurnAction> turn_left;
    std::shared_ptr<TurnAction> turn_right;
    std::shared_ptr<TurnAction> turn_back;

    std::queue<std::shared_ptr<Action>> action_queue;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_ACTION_QUEUER_HPP
