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
};

constexpr std::array idle_commands{
    Command::EXPLORE, Command::SOLVE, Command::CALIBRATE, Command::SAVE, Command::RESET,
};
}  // namespace

int main() {
    // --- codes ---
    for (uint8_t code = 0; code <= std::to_underlying(Command::LEAVE_ERROR); code++) {
        CHECK(to_command(code) == static_cast<Command>(code));
    }

    CHECK(not to_command(7).has_value());
    CHECK(not to_command(255).has_value());

    // --- a stop is accepted everywhere ---
    for (uint8_t id = 0; id < std::to_underlying(State::NUMBER_OF_STATES); id++) {
        CHECK(not refusal(static_cast<State>(id), Command::STOP).has_value());
    }

    // --- idle accepts every command but leaving an error ---
    for (const Command command : idle_commands) {
        CHECK(not refusal(State::IDLE, command).has_value());
    }

    CHECK(refusal(State::IDLE, Command::LEAVE_ERROR) == Reason::NOT_IN_ERROR);

    // --- a busy robot refuses everything but a stop ---
    for (const State state : busy_states) {
        for (const Command command : idle_commands) {
            CHECK(refusal(state, command) == Reason::NOT_IDLE);
        }

        CHECK(refusal(state, Command::LEAVE_ERROR) == Reason::NOT_IN_ERROR);
    }

    // --- a save in progress says so ---
    for (const Command command : idle_commands) {
        CHECK(refusal(State::SAVE, command) == Reason::BUSY_SAVING);
    }

    CHECK(refusal(State::SAVE, Command::LEAVE_ERROR) == Reason::BUSY_SAVING);

    // --- the error state only lets the robot stop or leave it ---
    for (const Command command : idle_commands) {
        CHECK(refusal(State::ERROR, command) == Reason::NOT_IDLE);
    }

    CHECK(not refusal(State::ERROR, Command::LEAVE_ERROR).has_value());

    std::puts("command ok");
}
