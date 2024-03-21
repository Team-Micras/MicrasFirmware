/**
 * @file current_sensors.hpp
 *
 * @brief Proxy CurrentSensors class header
 *
 * @date 03/2024
 */

#ifndef __CURRENT_SENSORS_HPP__
#define __CURRENT_SENSORS_HPP__

#include <array>
#include <cstdint>

#include "hal/adc_dma.hpp"

namespace proxy {
/**
 * @brief Class for controlling CurrentSensors
 */
template <uint8_t num_of_sensors>
class CurrentSensors {
    public:
        /**
         * @brief configuration structure for current sensors
         */
        struct Config {
            hal::AdcDma::Config adc;
            float               shunt_resistor;
        };

        /**
         * @brief Constructor for the CurrentSensors class
         *
         * @param current_sensors_config Configuration for the current sensors
         */
        CurrentSensors(const Config& current_sensors_config);

        /**
         * @brief Get the current from the sensor
         *
         * @param sensor_index Index of the sensor
         * @return float Current reading from the sensor
         */
        float get_current(uint8_t sensor_index) const;

        /**
         * @brief Get the raw reading from the current sensor
         *
         * @param sensor_index Index of the sensor
         * @return uint16_t Current reading from the sensor
         */
        uint32_t get_current_raw(uint8_t sensor_index) const;

    private:
        /**
         * @brief ADC DMA handle
         */
        hal::AdcDma adc;

        /**
         * @brief Buffer to store the ADC values
         */
        volatile std::array<uint32_t, num_of_sensors> dma_buffer;

        /**
         * @brief Value of the shunt resistor in ohms
         */
        float shunt_resistor;
};
}  // namespace proxy

#endif // __CURRENT_SENSORS_HPP__