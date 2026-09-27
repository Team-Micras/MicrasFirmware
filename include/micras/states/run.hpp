/**
 * @file
 */

#ifndef RUN_STATE_HPP
#define RUN_STATE_HPP

#include <utility>

#include "micras/states/base.hpp"

namespace micras {
class RunState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override {
        this->micras.init();
        this->micras.prepare();
    }

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override {
        if (this->micras.check_crash()) {
            return std::to_underlying(Micras::State::ERROR);
        }

        if (not this->micras.run()) {
            return this->get_id();
        }

        switch (this->micras.get_objective()) {
            case core::Objective::EXPLORE:
                this->micras.set_objective(core::Objective::RETURN);
                this->micras.save_best_route();
                return std::to_underlying(Micras::State::WAIT_FOR_RUN);

            case core::Objective::RETURN:
                this->micras.set_objective(core::Objective::SOLVE);
                this->micras.save_best_route();
                return std::to_underlying(Micras::State::IDLE);

            case core::Objective::SOLVE:
                return std::to_underlying(Micras::State::IDLE);
        }

        return std::to_underlying(Micras::State::ERROR);
    }
};
}  // namespace micras

#endif  // RUN_STATE_HPP
