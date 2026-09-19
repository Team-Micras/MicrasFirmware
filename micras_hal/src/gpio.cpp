/**
 * @file
 */

#include <cstdint>

#include "micras/hal/gpio.hpp"

namespace micras::hal {
/**
 * @brief Offset of the reset half of the bit set reset register.
 */
static constexpr uint32_t reset_offset{16};

Gpio::Gpio(const Config& config) : port{config.port}, pin{config.pin} { }

bool Gpio::read() const {
    return (this->port->IDR & this->pin) != 0;
}

void Gpio::write(bool state) {
    this->port->BSRR = state ? this->pin : static_cast<uint32_t>(this->pin) << reset_offset;
}

void Gpio::toggle() {
    this->write((this->port->ODR & this->pin) == 0);
}
}  // namespace micras::hal
