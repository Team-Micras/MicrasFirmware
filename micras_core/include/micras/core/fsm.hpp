/**
 * @file
 */

#ifndef MICRAS_CORE_FSM_HPP
#define MICRAS_CORE_FSM_HPP

#include <array>
#include <cstdint>
#include <memory>

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
 * from zero, which the number of states asserts.
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
     * @brief Add a state to the FSM, taking ownership of it.
     *
     * @param state The state to be added.
     */
    void add_state(std::unique_ptr<FsmState> state);

    /**
     * @brief Run the FSM current state to compute the next state.
     *
     * @note Aborts when the current state id has no state behind it, which can only be a
     * programming error, and running the abort handler beats dispatching through a null pointer
     * with the motors turning.
     */
    void update();

private:
    /**
     * @brief States of the machine, indexed by their id.
     */
    std::array<std::unique_ptr<FsmState>, num_of_states> states{};

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
