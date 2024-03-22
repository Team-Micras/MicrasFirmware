/**
 * @file battery.hpp
 *
 * @brief Proxy Battery class declaration
 *
 * @date 03/2024
 */

#ifndef __BATTERY_HPP__
#define __BATTERY_HPP__

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
        };

        /**
         * @brief Constructor for the Battery class
         *
         * @param battery_config Configuration for the battery
         */
        Battery(Config& battery_config);

        /**
         * @brief Get the battery voltage
         *
         * @return Battery voltage in volts
         */
        float get_voltage();

        /**
         * @brief Get the raw reading from the battery
         *
         * @return uint32_t Raw reading from the battery
         */
        uint32_t get_voltage_raw();

        /**
         * @brief Voltage divider ratio
         */
        static constexpr float voltage_divider{3.0f};

    private:
        /**
         * @brief Adc object
         */
        hal::AdcDma adc;

        /**
         * @brief Raw reading from the battery
         */
        uint32_t raw_reading;
};
}  // namespace proxy

#endif // __BATTERY_HPP__
