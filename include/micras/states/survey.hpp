/**
 * @file
 */

#ifndef SURVEY_STATE_HPP
#define SURVEY_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that drives the survey of the wall sensors and keeps what they read.
 *
 * @note The robot starts centered in a straight corridor, its front caps touching the wall at its end.
 */
class SurveyState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override;
};
}  // namespace micras

#endif  // SURVEY_STATE_HPP
