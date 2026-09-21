/**
 * @file
 */

#include "micras/proxy/battery.hpp"

namespace micras::proxy {
Battery::Battery(const Config& config) :
    adc{config.adc},
    max_voltage{config.adc.reference_voltage * config.voltage_divider},
    filter{config.filter},
    initialized{this->adc.start_dma({&(this->raw_reading), 1}) and this->adc.was_initialized()} { }

void Battery::update() {
    this->filter.update(this->get_adc_reading());
}

float Battery::get_voltage() const {
    return this->filter.get_last() * this->max_voltage;
}

float Battery::get_voltage_raw() const {
    return this->get_adc_reading() * this->max_voltage;
}

float Battery::get_adc_reading() const {
    return static_cast<float>(this->raw_reading) / this->adc.get_max_reading();
}

bool Battery::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::proxy
