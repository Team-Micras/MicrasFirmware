/**
 * @file battery.cpp
 *
 * @brief Proxy Battery class implementation
 *
 * @date 03/2024
 */

#include "proxy/battery.hpp"

namespace proxy {
Battery::Battery(Config& battery_config) : adc{battery_config.adc} {
    this->adc.start_dma(&(this->raw_reading), 1);
}

float Battery::get_voltage() {
    return Battery::voltage_divider * this->raw_reading * hal::AdcDma::reference_voltage / hal::AdcDma::max_reading;
}

uint32_t Battery::get_voltage_raw() {
    return this->raw_reading;
}
}  // namespace proxy
