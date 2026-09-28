/**
 * @file
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "micras/comm/credit_window.hpp"
#include "micras/comm/protocol.hpp"

namespace micras::comm {
void CreditWindow::reset() {
    this->sent = 0;
    this->consumed = 0;
}

bool CreditWindow::allows(std::size_t size) const {
    return this->outstanding() + size <= credit_window;
}

void CreditWindow::charge(std::size_t size) {
    this->sent += static_cast<uint32_t>(size);
}

void CreditWindow::acknowledge(uint32_t consumed_total) {
    constexpr uint32_t half_range{UINT32_C(1) << 31U};

    const uint32_t advance = consumed_total - this->consumed;

    if (advance >= half_range) {
        return;
    }

    this->consumed += std::min(advance, this->outstanding());
}

uint32_t CreditWindow::outstanding() const {
    return this->sent - this->consumed;
}

uint16_t CreditWindow::available() const {
    return static_cast<uint16_t>(credit_window - std::min<uint32_t>(this->outstanding(), credit_window));
}
}  // namespace micras::comm
