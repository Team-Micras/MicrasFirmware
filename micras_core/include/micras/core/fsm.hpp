/**
 * @file
 */

#ifndef MICRAS_CORE_FSM_HPP
#define MICRAS_CORE_FSM_HPP

#include <cstdint>
#include <memory>
#include <unordered_map>

namespace micras::core {
class Fsm {
public:
    class State {
    public:
        /**
         * @brief Destroy the State object.
         */
        virtual ~State() = default;

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
        uint8_t get_id() const;

        /**
         * @brief The id of the state that is not valid.
         */
        static constexpr uint8_t invalid_id{0xFF};

    protected:
        /**
         * @brief Special member functions declared as default.
         */
        ///@{
        explicit State(uint8_t id);
        State(const State&) = default;
        State(State&&) = default;
        State& operator=(const State&) = default;
        State& operator=(State&&) = default;
        ///@}

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
    explicit Fsm(uint8_t initial_state_id);

    /**
     * @brief Add a state to the FSM.
     *
     * @param state The state to be added.
     */
    void add_state(std::unique_ptr<State> state);

    /**
     * @brief Run the FSM current state to compute the next state.
     */
    void update();

private:
    /**
     * @brief Map of ids to states.
     */
    std::unordered_map<uint8_t, std::unique_ptr<State>> states;

    /**
     * @brief Id of the state currently running.
     */
    uint8_t current_state_id{0};

    /**
     * @brief Id of the last executed state.
     */
    uint8_t previous_state_id{State::invalid_id};
};
}  // namespace micras::core

#endif  // MICRAS_CORE_FSM_HPP
