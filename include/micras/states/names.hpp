/**
 * @file
 */

#ifndef STATE_NAMES_HPP
#define STATE_NAMES_HPP

#include <algorithm>
#include <array>
#include <string_view>
#include <utility>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief Names of the states of the robot, indexed as micras::State.
 *
 * @note The simulation records these names in its runs and baselines, so renaming a state is a change of
 * the simulation's baselines.
 */
inline constexpr std::array<std::string_view, std::to_underlying(State::NUMBER_OF_STATES)> state_names{
    "INIT",
    "IDLE",
    "WAIT_FOR_RUN",
    "RUN",
    "PLAN",
    "SAVE",
    "WAIT_FOR_CALIBRATE",
    "CALIBRATE",
    "WAIT_FOR_IDENTIFY",
    "IDENTIFY",
    "WAIT_FOR_GYROSCOPE",
    "CALIBRATE_GYROSCOPE",
    "ERROR",
};

static_assert(
    std::ranges::none_of(state_names, [](std::string_view name) { return name.empty(); }),
    "every state needs a name in micras::state_names"
);

/**
 * @brief Get the name of a state.
 *
 * @param state The state.
 * @return The name of the state.
 */
constexpr std::string_view state_name(State state) {
    return state_names.at(std::to_underlying(state));
}

static_assert(state_name(State::INIT) == "INIT");
static_assert(state_name(State::IDLE) == "IDLE");
static_assert(state_name(State::WAIT_FOR_RUN) == "WAIT_FOR_RUN");
static_assert(state_name(State::RUN) == "RUN");
static_assert(state_name(State::PLAN) == "PLAN");
static_assert(state_name(State::SAVE) == "SAVE");
static_assert(state_name(State::WAIT_FOR_CALIBRATE) == "WAIT_FOR_CALIBRATE");
static_assert(state_name(State::CALIBRATE) == "CALIBRATE");
static_assert(state_name(State::WAIT_FOR_IDENTIFY) == "WAIT_FOR_IDENTIFY");
static_assert(state_name(State::IDENTIFY) == "IDENTIFY");
static_assert(state_name(State::WAIT_FOR_GYROSCOPE) == "WAIT_FOR_GYROSCOPE");
static_assert(state_name(State::CALIBRATE_GYROSCOPE) == "CALIBRATE_GYROSCOPE");
static_assert(state_name(State::ERROR) == "ERROR");
}  // namespace micras

#endif  // STATE_NAMES_HPP
