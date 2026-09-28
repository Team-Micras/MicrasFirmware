/**
 * @file
 */

#include <array>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <utility>

#include "micras/command.hpp"
#include "micras/states/base.hpp"
#include "test_host.hpp"

using namespace micras;

namespace {
constexpr std::array busy_states{
    State::INIT,
    State::WAIT_FOR_RUN,
    State::RUN,
    State::PLAN,
    State::WAIT_FOR_CALIBRATE,
    State::CALIBRATE,
    State::WAIT_FOR_IDENTIFY,
    State::IDENTIFY,
    State::WAIT_FOR_GYROSCOPE,
    State::CALIBRATE_GYROSCOPE,
    State::BRAKE,
};

constexpr std::array idle_commands{
    Command::EXPLORE, Command::SOLVE, Command::CALIBRATE, Command::SAVE, Command::RESET,
};

std::optional<Reason> entered(State state, Command command) {
    return refusal(state, true, command);
}
}  // namespace

int main() {
    // --- codes ---
    for (uint8_t code = 0; code < std::to_underlying(Command::NUMBER_OF_COMMANDS); code++) {
        CHECK(to_command(code) == static_cast<Command>(code));
    }

    CHECK(not to_command(std::to_underlying(Command::NUMBER_OF_COMMANDS)).has_value());
    CHECK(not to_command(255).has_value());

    // --- a stop is accepted everywhere, entered or not ---
    for (uint8_t id = 0; id < std::to_underlying(State::NUMBER_OF_STATES); id++) {
        CHECK(not refusal(static_cast<State>(id), true, Command::STOP).has_value());
        CHECK(not refusal(static_cast<State>(id), false, Command::STOP).has_value());
    }

    // --- idle accepts every command but leaving an error ---
    for (const Command command : idle_commands) {
        CHECK(not entered(State::IDLE, command).has_value());
    }

    CHECK(entered(State::IDLE, Command::LEAVE_ERROR) == Reason::NOT_IN_ERROR);

    // --- a busy robot refuses everything but a stop ---
    for (const State state : busy_states) {
        for (const Command command : idle_commands) {
            CHECK(entered(state, command) == Reason::NOT_IDLE);
        }

        CHECK(entered(state, Command::LEAVE_ERROR) == Reason::NOT_IN_ERROR);
    }

    // --- a save in progress says so, and leaving an error is refused as in any other state ---
    for (const Command command : idle_commands) {
        CHECK(entered(State::SAVE, command) == Reason::BUSY_SAVING);
    }

    CHECK(entered(State::SAVE, Command::LEAVE_ERROR) == Reason::NOT_IN_ERROR);

    // --- the error state only lets the robot stop or leave it ---
    for (const Command command : idle_commands) {
        CHECK(entered(State::ERROR, command) == Reason::NOT_IDLE);
    }

    CHECK(not entered(State::ERROR, Command::LEAVE_ERROR).has_value());

    // --- a state that was chosen but not entered accepts only a stop ---
    for (const Command command : idle_commands) {
        CHECK(refusal(State::IDLE, false, command) == Reason::NOT_IDLE);
        CHECK(refusal(State::SAVE, false, command) == Reason::BUSY_SAVING);
    }

    CHECK(refusal(State::ERROR, false, Command::LEAVE_ERROR) == Reason::NOT_IN_ERROR);

    std::puts("command ok");
}
