/**
 * @file
 */

#ifndef MICRAS_CORE_FSM_HPP
#define MICRAS_CORE_FSM_HPP

#include <cstdint>
#include <unordered_map>

namespace micras::core {
class FSM {
public:
    class State {
    public:
        /**
         * @brief Construct a new State object.
         *
         * @param id The id of the state.
         */
        explicit State(uint8_t id);

        /**
         * @brief Destroy the State object.
         */
        virtual ~State() = default;

        /**
         * @brief Special member functions declared as default.
         */
        ///@{
        State(const State&) = default;
        State(State&&) = default;
        State& operator=(const State&) = default;
        State& operator=(State&&) = default;
        ///@}

        /**
         * @brief Execute this state.
         *
         * @return The id of the next state.
         */
        virtual uint8_t run();

        /**
         * @brief Get the id object of the state.
         *
         * @return The id of the state.
         */
        uint8_t get_id() const;

    private:
        /**
         * @brief Fixed id of the state.
         */
        uint8_t id;
    };

    /**
     * @brief Construct a new FSM object.
     *
     * @param initial_state_id The id of the initial state.
     */
    explicit FSM(uint8_t initial_state_id);

    /**
     * @brief Add a state to the FSM.
     *
     * @param state The state to be added.
     */
    void add_state(const State& state);

    /**
     * @brief Run the FSM current state to calculate the next state.
     */
    void run();

private:
    /**
     * @brief Map of ids to states.
     */
    std::unordered_map<int, State> states;

    /**
     * @brief Id of the state currently running.
     */
    uint8_t current_state_id{0};
};
}  // namespace micras::core

#endif  // MICRAS_CORE_FSM_HPP
