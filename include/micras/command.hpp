/**
 * @file
 */

#ifndef MICRAS_COMMAND_HPP
#define MICRAS_COMMAND_HPP

#include <cstdint>
#include <optional>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief Commands the link can ask the robot to run.
 *
 * @note These are edges, not levels: each one happens once, when it arrives, or is refused then.
 * Everything that is a level, like the run profile, is a writable variable instead.
 *
 * @note Unlike a press of the button, a command is never kept for later: the states that accept each
 * one are fixed, and a command that arrives in any other state is refused with a reason. Starting a
 * run, a maintenance procedure, a save or a reset needs the robot idle. A stop is accepted in every
 * state: it ends whatever the robot was doing and leaves it idle, or in the error state if it was
 * there, and while the maze is being saved it waits for the save to end. Leaving the error state
 * needs an error that came after the start, since one the start found means hardware that cannot be
 * trusted to move.
 */
enum class Command : uint8_t {
    EXPLORE = 0,      ///< Search the maze, as a short press of the button does.
    SOLVE = 1,        ///< Run the fastest route found, as a long press does.
    CALIBRATE = 2,    ///< Start the maintenance procedure the switches select, as an extra long press does.
    SAVE = 3,         ///< Save the maze to the flash, answering only once it is written.
    RESET = 4,        ///< Put the estimate of the pose back at the start.
    STOP = 5,         ///< Stop the motors and end whatever the robot is doing.
    LEAVE_ERROR = 6,  ///< Go from the error state back to idle.

    NUMBER_OF_COMMANDS = 7,  ///< Number of commands, which is not a command itself.
};

/**
 * @brief Why a command was refused or deferred, as the link reports it.
 */
enum class Reason : uint8_t {
    NONE = 0,             ///< No reason: the command ran.
    NOT_IDLE = 1,         ///< The command needs the robot idle, and it is busy or in the error state.
    BUSY_SAVING = 2,      ///< The maze is being saved: refused, or for a stop, deferred until the save ends.
    FAULT_FROM_INIT = 3,  ///< The error was found at start, and the robot is not to move again until reset.
    NOT_IN_ERROR = 4,     ///< Leaving the error state, with the robot not in it.
    SAVE_FAILED = 5,      ///< The flash did not take the maze.
};

/**
 * @brief How a stop is carried out, from the state it arrives in.
 */
enum class StopAction : uint8_t {
    DEFER = 0,        ///< Wait for the save to end, and stop then.
    CONFIRM = 1,      ///< Nothing more to do: the brake was chosen and starts on its own.
    BRAKE = 2,        ///< Brake to a standstill the way the state moves, then be idle.
    WHEEL_BRAKE = 3,  ///< Drop the brake under way and bring the wheels to rest, with no path and no pose.
    IDLE = 4,         ///< Turn everything off and be idle.
    STAY = 5,         ///< Turn everything off and stay in the state.
};

/**
 * @brief Get the command a code of the link stands for.
 *
 * @param code Code of the command on the wire.
 * @return The command, or no value if no command has that code.
 */
std::optional<Command> to_command(uint8_t code);

/**
 * @brief Check whether a state accepts a command, from the table of the commands of each state.
 *
 * @note Only the table: a command a state accepts can still be refused for what the robot holds,
 * such as leaving an error that the start found.
 *
 * @note A state that was chosen but not entered yet accepts only a stop. Its entry function has not
 * run, so a command that left it for another state would skip it: the stop of the idle state, or
 * the LED and the log of the error state.
 *
 * @param state The state the robot is in.
 * @param entered Whether the state has been entered, rather than only chosen.
 * @param command The command.
 * @return Why the state refuses the command, or no value if it accepts it.
 */
std::optional<Reason> refusal(State state, bool entered, Command command);

/**
 * @brief Choose how to carry out a stop, from the state it arrives in.
 *
 * @note A run and the procedures that move the robot brake to a standstill once they have been
 * entered, the way each moves. A second stop once that brake is under way asks for a stop that
 * trusts neither the path nor the pose, so it brakes the wheels instead, while one that arrives
 * before the brake has started only confirms it. During a save the stop waits for the save to end.
 * The initialization and the error state keep the robot where it is, and every other state makes
 * it idle.
 *
 * @param state The state the robot is in.
 * @param entered Whether the state has been entered, rather than only chosen.
 * @return How to stop.
 */
StopAction stop_action(State state, bool entered);
}  // namespace micras

#endif  // MICRAS_COMMAND_HPP
