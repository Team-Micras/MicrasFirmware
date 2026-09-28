/**
 * @file
 */

#ifndef MICRAS_CORE_FSM_HPP
#define MICRAS_CORE_FSM_HPP

#include <array>
#include <cstdint>

namespace micras::core {
/**
 * @brief A single state of a finite state machine.
 */
class FsmState {
public:
    /**
     * @brief Destroy the FsmState object.
     */
    virtual ~FsmState() = default;

    /**
     * @brief Execute the entry function of this state.
     */
    virtual void on_entry() = 0;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    virtual uint8_t execute() = 0;

    /**
     * @brief Get the id object of the state.
     *
     * @return The id of the state.
     */
    uint8_t get_id() const { return this->id; }

    /**
     * @brief The id of the state that is not valid.
     */
    static constexpr uint8_t invalid_id{0xFF};

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    explicit FsmState(uint8_t id) : id{id} { }

    FsmState(const FsmState&) = default;
    FsmState(FsmState&&) = default;
    FsmState& operator=(const FsmState&) = default;
    FsmState& operator=(FsmState&&) = default;
    ///@}

private:
    /**
     * @brief Fixed id of the state.
     */
    uint8_t id;
};

/**
 * @brief Finite state machine over a dense set of state ids.
 *
 * @note The states are stored in an array indexed by their own id, so running the machine costs an
 * array access rather than a hash lookup. That only works because the ids are a dense enumeration
 * from zero.
 *
 * @tparam num_of_states Number of states, and therefore one past the largest valid id.
 */
template <uint8_t num_of_states>
class TFsm {
public:
    /**
     * @brief Construct a new TFsm object.
     *
     * @param initial_state_id The id of the initial state.
     */
    explicit TFsm(uint8_t initial_state_id);

    /**
     * @brief Add a state to the FSM.
     *
     * @note The state is borrowed, not owned: whoever owns the machine holds its states by value,
     * next to it, so that nothing is allocated and they live exactly as long as the machine does.
     *
     * @param state The state to be added.
     */
    void add_state(FsmState& state);

    /**
     * @brief Run the FSM current state to compute the next state.
     *
     * @note Aborts when the current state id has no state behind it, which can only be a
     * programming error, and running the abort handler beats dispatching through a null pointer
     * with the motors turning.
     */
    void update();

    /**
     * @brief Make the machine go to a state instead of the one the current state chose.
     *
     * @note For a decision taken outside the states, such as a stop that arrived over the link.
     * The next update enters the state, running its entry function unless it is the state that ran
     * last.
     *
     * @param state_id The id of the state to go to.
     */
    void transition_to(uint8_t state_id);

    /**
     * @brief Get the id of the state currently running.
     *
     * @return The id of the current state.
     */
    uint8_t get_current_state_id() const;

    /**
     * @brief Check if the current state has been entered, rather than only chosen.
     *
     * @note A state chosen by the one before it, or by transition_to, is entered by the next update,
     * which runs its entry function first. Until then the machine is between the two, and whatever
     * the entry function sets up, such as stopping the motors, has not happened yet.
     *
     * @return True if the current state has run at least once since the machine went to it.
     */
    bool has_entered_current_state() const;

private:
    /**
     * @brief States of the machine, indexed by their id.
     */
    std::array<FsmState*, num_of_states> states{};

    /**
     * @brief Id of the state currently running.
     */
    uint8_t current_state_id{0};

    /**
     * @brief Id of the last executed state.
     */
    uint8_t previous_state_id{FsmState::invalid_id};
};
}  // namespace micras::core

#include "micras/core/impl/fsm.tpp"  // IWYU pragma: export

#endif  // MICRAS_CORE_FSM_HPP
