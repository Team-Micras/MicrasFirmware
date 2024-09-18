/**
 * @file battery.cpp
 *
 * @brief Proxy Battery class implementation
 *
 * @date 03/2024
 */

#include "micras/proxy/battery.hpp"

namespace micras::proxy {
Battery::Battery(const Config& config) : adc{config.adc}, voltage_divider{config.voltage_divider} {
    this->adc.start_dma(&(this->raw_reading), 1);
}

float Battery::get_voltage() const {
    return this->voltage_divider * this->raw_reading * this->adc.get_reference_voltage() / this->adc.get_max_reading();
}

uint32_t Battery::get_voltage_raw() const {
    return this->raw_reading;
}
}  // namespace micras::proxy
