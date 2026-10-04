/**
 * @file
 *
 * @brief Designs of the turns of two bends, for each margin.
 *
 * @note Written by the turn designer of the simulation (the `sim_turn_designs` target), which searches
 * for the fastest two bends that clear the walls. Do not edit by hand: the build checks every design
 * against the robot and the maze, and a change to either that makes one no longer fit stops it, which
 * is when the designer has to run again. A turn nothing fits is written with no curvature, which also
 * stops the build.
 */

#ifndef MICRAS_TWO_BEND_TURNS_HPP
#define MICRAS_TWO_BEND_TURNS_HPP

#include <array>

#include "micras/nav/lattice.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras {
/**
 * @brief Designs of the turns of two bends, from nav::first_two_bend_turn on, with a margin of 12 mm.
 */
constexpr std::array<nav::TwoBendDesign, nav::number_of_two_bend_turns> two_bend_designs{{
    {.first_angle = -0.392699093F,
     .first_curvature = 13.1556044F,
     .second_angle = 1.17809725F,
     .second_curvature = 13.1556044F,
     .pre = 0.00499999989F},  // SD45E
    {.first_angle = 1.22173047F,
     .first_curvature = 19.8867226F,
     .second_angle = -0.436332315F,
     .second_curvature = 16.1747284F,
     .pre = 0.0F},  // SD45T
    {.first_angle = -0.0872664601F,
     .first_curvature = 10.7000208F,
     .second_angle = 2.44346094F,
     .second_curvature = 19.8867226F,
     .pre = 0.0F},  // SD135T
    {.first_angle = -0.436332315F,
     .first_curvature = 16.1747284F,
     .second_angle = 1.22173047F,
     .second_curvature = 19.8867226F,
     .pre = 0.0F},  // DS45T
    {.first_angle = -1.43989658F,
     .first_curvature = 19.8867226F,
     .second_angle = 1.43989658F,
     .second_curvature = 19.8867226F,
     .pre = 0.0F},  // DD0E
    {.first_angle = -1.17809725F,
     .first_curvature = 13.1556044F,
     .second_angle = 0.392699093F,
     .second_curvature = 10.7000208F,
     .pre = 0.0F},  // DS45E
    {.first_angle = -0.741764903F,
     .first_curvature = 11.8644524F,
     .second_angle = -0.829031408F,
     .second_curvature = 11.8644524F,
     .pre = 0.0F},  // DD90E
    {.first_angle = -2.44346094F,
     .first_curvature = 19.8867226F,
     .second_angle = 0.0872664601F,
     .second_curvature = 10.7000208F,
     .pre = 0.0F},  // DS135B
}};

/**
 * @brief Designs of the turns of two bends, from nav::first_two_bend_turn on, with a margin of 8 mm.
 */
constexpr std::array<nav::TwoBendDesign, nav::number_of_two_bend_turns> risky_two_bend_designs{{
    {.first_angle = -0.392699093F,
     .first_curvature = 13.1556044F,
     .second_angle = 1.17809725F,
     .second_curvature = 13.1556044F,
     .pre = 0.00499999989F},  // SD45E
    {.first_angle = 1.22173047F,
     .first_curvature = 19.8867226F,
     .second_angle = -0.436332315F,
     .second_curvature = 16.1747284F,
     .pre = 0.0F},  // SD45T
    {.first_angle = -0.0872664601F,
     .first_curvature = 10.7000208F,
     .second_angle = 2.44346094F,
     .second_curvature = 19.8867226F,
     .pre = 0.0F},  // SD135T
    {.first_angle = -0.436332315F,
     .first_curvature = 16.1747284F,
     .second_angle = 1.22173047F,
     .second_curvature = 19.8867226F,
     .pre = 0.0F},  // DS45T
    {.first_angle = -1.43989658F,
     .first_curvature = 19.8867226F,
     .second_angle = 1.43989658F,
     .second_curvature = 19.8867226F,
     .pre = 0.0F},  // DD0E
    {.first_angle = -1.17809725F,
     .first_curvature = 13.1556044F,
     .second_angle = 0.392699093F,
     .second_curvature = 10.7000208F,
     .pre = 0.0F},  // DS45E
    {.first_angle = -0.741764903F,
     .first_curvature = 10.7000208F,
     .second_angle = -0.829031408F,
     .second_curvature = 10.7000208F,
     .pre = 0.0F},  // DD90E
    {.first_angle = -2.44346094F,
     .first_curvature = 19.8867226F,
     .second_angle = 0.0872664601F,
     .second_curvature = 10.7000208F,
     .pre = 0.0F},  // DS135B
}};
}  // namespace micras

#endif  // MICRAS_TWO_BEND_TURNS_HPP
