/**
 * @file
 *
 * @brief The shape of every turn, computed when the firmware is compiled, and the dynamics built on
 * them.
 *
 * @note Designing the turns takes the compiler about a gigabyte of memory and several seconds, so it
 * happens in this one source and not in every source that includes the constants.
 */

#include "constants.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/turn_table.hpp"
#include "robot.hpp"
#include "turn_margins.hpp"
#include "two_bend_turns.hpp"

namespace micras {
namespace {
/**
 * @brief Turns of two bends placed on the lattice, with the normal and with the risky margin.
 */
///@{
constexpr nav::TurnTable::TwoBendShapes two_bend_shapes{
    nav::TurnTable::place(robot_model, turn_margin, two_bend_designs)
};
constexpr nav::TurnTable::TwoBendShapes risky_two_bend_shapes{
    nav::TurnTable::place(robot_model, risky_turn_margin, risky_two_bend_designs)
};
///@}

/**
 * @brief Shape of every turn, with the normal and with the risky margin.
 *
 * @note A turn that does not fit in the maze with the margin asked for stops the build here.
 */
///@{
constexpr nav::TurnTable turn_table{robot_model, turn_margin, two_bend_shapes};
constexpr nav::TurnTable risky_turn_table{robot_model, risky_turn_margin, risky_two_bend_shapes};
///@}

static_assert(turn_table.is_valid(), "a turn does not fit in the maze with the normal margin");
static_assert(risky_turn_table.is_valid(), "a turn does not fit in the maze with the risky margin");
static_assert(
    nav::TurnTable::clears(robot_model, turn_margin, two_bend_designs),
    "a turn of two bends does not clear the walls with the normal margin: the turn designer found none"
);
static_assert(
    nav::TurnTable::clears(robot_model, risky_turn_margin, risky_two_bend_designs),
    "a turn of two bends does not clear the walls with the risky margin: the turn designer found none"
);
}  // namespace

constexpr nav::Dynamics::Config dynamics_config{
    .model = robot_model,
    .turns = turn_table,
    .risky_turns = risky_turn_table,
    .max_linear_speed = 4.0F,
    .max_angular_speed = 12.0F,
    .voltage_reserve = voltage_reserve,
};
}  // namespace micras
