/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/survey.hpp"

namespace micras {
void SurveyState::on_entry() {
    this->micras.start_survey();
}

uint8_t SurveyState::execute() {
    if (this->micras.check_fault()) {
        return std::to_underlying(State::ERROR);
    }

    return this->micras.survey() ? std::to_underlying(State::IDLE) : this->get_id();
}
}  // namespace micras
