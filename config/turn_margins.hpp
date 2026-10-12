/**
 * @file
 */

#ifndef MICRAS_TURN_MARGINS_HPP
#define MICRAS_TURN_MARGINS_HPP

namespace micras {
/**
 * @brief Distance kept between the outline of the robot and any obstacle when a turn is designed,
 * without and with the risky switch.
 *
 * @note Apart from the constants, so that the turn designer can read them without compiling the
 * designs it is there to replace.
 *
 * @note What the robot really keeps from the walls is the margin less the drift of its pose estimate
 * in the turns. Simulated in the home maze with its glossy walls, under noise, a start
 * 3 mm off, walls of Minnaert exponent 1.6 and 2 and softer tires, 12 mm left 7 mm and 20 mm left
 * 16 mm, for 0.09 s more of the fast run. A margin of 22 mm leaves some turn with no design.
 */
///@{
constexpr float turn_margin{0.020F};
constexpr float risky_turn_margin{0.016F};
///@}
}  // namespace micras

#endif  // MICRAS_TURN_MARGINS_HPP
