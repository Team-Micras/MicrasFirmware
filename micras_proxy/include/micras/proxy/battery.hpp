/**
 * @file
 */

#ifndef MICRAS_PROXY_BATTERY_HPP
#define MICRAS_PROXY_BATTERY_HPP

#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/hal/adc_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for getting the battery voltage.
 */
class Battery {
public:
    /**
     * @brief Configuration struct for the battery.
     */
    struct Config {
        hal::AdcDma::Config adc;
        float               voltage_divider;
        float               filter_cutoff;
    };

    /**
     * @brief Construct a new Battery object.
     *
     * @param config Configuration for the battery.
     */
    explicit Battery(const Config& config);

    /**
     * @brief Update the battery reading.
     */
    void update();

    /**
     * @brief Get the battery voltage.
     *
     * @return Battery voltage in volts.
     */
    float get_voltage() const;

    /**
     * @brief Get the battery voltage in volts without the filter applied.
     *
     * @return Battery voltage in volts.
     */
    float get_voltage_raw() const;

    /**
     * @brief Get the battery reading from the ADC.
     *
     * @return Battery reading from 0 to 1.
     */
    float get_adc_reading() const;

private:
    /**
     * @brief Adc object.
     */
    hal::AdcDma adc;

    /**
     * @brief Raw reading from the battery.
     */
    uint16_t raw_reading{};

    /**
     * @brief Maximum voltage that can be read.
     */
    float max_voltage;

    /**
     * @brief Butterworth filter for the battery reading.
     */
    core::ButterworthFilter filter;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BATTERY_HPP
