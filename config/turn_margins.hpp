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
 */
///@{
constexpr float turn_margin{0.015F};
constexpr float risky_turn_margin{0.010F};
///@}
}  // namespace micras

#endif  // MICRAS_TURN_MARGINS_HPP
