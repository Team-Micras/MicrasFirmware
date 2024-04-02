/**
 * @file battery.hpp
 *
 * @brief Proxy Battery class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_BATTERY_HPP
#define MICRAS_PROXY_BATTERY_HPP

#include <cstdint>

#include "hal/adc_dma.hpp"

namespace proxy {
/**
 * @brief Class for getting the battery voltage
 */
class Battery {
    public:
        /**
         * @brief Configuration structure for the battery
         */
        struct Config {
            hal::AdcDma::Config adc;
            float               voltage_divider;
        };

        /**
         * @brief Constructor for the Battery class
         *
         * @param config Configuration for the battery
         */
        Battery(const Config& config);

        /**
         * @brief Get the battery voltage
         *
         * @return Battery voltage in volts
         */
        float get_voltage() const;

        /**
         * @brief Get the raw reading from the battery
         *
         * @return uint32_t Raw reading from the battery
         */
        uint32_t get_voltage_raw() const;

    private:
        /**
         * @brief Adc object
         */
        hal::AdcDma adc;

        /**
         * @brief Raw reading from the battery
         */
        uint32_t raw_reading{ };

        /**
         * @brief Voltage divider ratio
         */
        const float voltage_divider;
};
}  // namespace proxy

#endif // MICRAS_PROXY_BATTERY_HPP
