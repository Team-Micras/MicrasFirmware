/**
 * @file
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <utility>

#include "micras/command.hpp"
#include "micras/states/base.hpp"

namespace micras {
namespace {
/**
 * @brief Set of commands, one bit for each value of Command.
 */
using CommandSet = uint8_t;

/**
 * @brief Commands a state accepts from the link.
 */
struct CommandRule {
    State      state;
    CommandSet accepted;
};
}  // namespace

/**
 * @brief Make a set of commands.
 *
 * @param commands The commands in the set.
 * @return The set.
 */
static constexpr CommandSet command_set(std::initializer_list<Command> commands) {
    CommandSet set{};

    for (const Command command : commands) {
        set |= static_cast<CommandSet>(1U << std::to_underlying(command));
    }

    return set;
}

/**
 * @brief Commands a state that is busy with something accepts: only a stop.
 */
static constexpr CommandSet busy_commands{command_set({Command::STOP})};

/**
 * @brief Commands each state accepts from the link, indexed by the id of the state.
 */
static constexpr std::array<CommandRule, std::to_underlying(State::NUMBER_OF_STATES)> command_table{{
    {.state = State::INIT, .accepted = busy_commands},
    {.state = State::IDLE,
     .accepted = command_set(
         {Command::EXPLORE, Command::SOLVE, Command::CALIBRATE, Command::SAVE, Command::RESET, Command::STOP}
     )},
    {.state = State::WAIT_FOR_RUN, .accepted = busy_commands},
    {.state = State::RUN, .accepted = busy_commands},
    {.state = State::PLAN, .accepted = busy_commands},
    {.state = State::SAVE, .accepted = busy_commands},
    {.state = State::WAIT_FOR_CALIBRATE, .accepted = busy_commands},
    {.state = State::CALIBRATE, .accepted = busy_commands},
    {.state = State::WAIT_FOR_IDENTIFY, .accepted = busy_commands},
    {.state = State::IDENTIFY, .accepted = busy_commands},
    {.state = State::WAIT_FOR_GYROSCOPE, .accepted = busy_commands},
    {.state = State::CALIBRATE_GYROSCOPE, .accepted = busy_commands},
    {.state = State::ERROR, .accepted = command_set({Command::STOP, Command::LEAVE_ERROR})},
    {.state = State::BRAKE, .accepted = busy_commands},
}};

/**
 * @brief Check that the command table lists the states in the order of their ids, which it is
 * indexed by.
 *
 * @return True if every state is where its id says.
 */
static consteval bool is_indexed_by_state() {
    for (std::size_t id = 0; id < command_table.size(); id++) {
        if (std::to_underlying(command_table.at(id).state) != id) {
            return false;
        }
    }

    return true;
}

static_assert(is_indexed_by_state(), "the command table lists the states in the order of their ids");

static_assert(
    std::to_underlying(Command::NUMBER_OF_COMMANDS) <= std::numeric_limits<CommandSet>::digits,
    "every command needs a bit of a command set"
);

std::optional<Command> to_command(uint8_t code) {
    if (code >= std::to_underlying(Command::NUMBER_OF_COMMANDS)) {
        return std::nullopt;
    }

    return static_cast<Command>(code);
}

std::optional<Reason> refusal(State state, bool entered, Command command) {
    const CommandSet accepted = entered ? command_table.at(std::to_underlying(state)).accepted : busy_commands;

    if ((accepted & command_set({command})) != 0) {
        return std::nullopt;
    }

    if (command == Command::LEAVE_ERROR) {
        return Reason::NOT_IN_ERROR;
    }

    if (state == State::SAVE) {
        return Reason::BUSY_SAVING;
    }

    return Reason::NOT_IDLE;
}

StopAction stop_action(State state, bool entered) {
    if (state == State::SAVE) {
        return StopAction::DEFER;
    }

    if (state == State::BRAKE) {
        return entered ? StopAction::WHEEL_BRAKE : StopAction::CONFIRM;
    }

    if (entered and (state == State::RUN or state == State::IDENTIFY or state == State::CALIBRATE_GYROSCOPE)) {
        return StopAction::BRAKE;
    }

    if (state == State::INIT or state == State::ERROR) {
        return StopAction::STAY;
    }

    return StopAction::IDLE;
}
}  // namespace micras
