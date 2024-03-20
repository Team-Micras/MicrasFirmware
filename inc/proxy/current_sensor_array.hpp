/**
 * @file current_sensor_array.hpp
 *
 * @brief Proxy CurrentSensorArray class header
 *
 * @date 03/2024
 */

#ifndef __CURRENT_SENSOR_ARRAY_HPP__
#define __CURRENT_SENSOR_ARRAY_HPP__

#include <cstdint>

#include "hal/adc_dma.hpp"

namespace proxy {
/**
 * @brief Class for controlling CurrentSensorArray.
 */
template <uint8_t num_of_sensors>
class CurrentSensorArray {
    public:
        /**
         * @brief configuration structure for current sensor
         */
        struct Config {
            hal::AdcDma::Config adc_config;
        };

        /**
         * @brief Constructor for the CurrentSensorArray class
         *
         * @param current_sensor_config Configuration for the current sensor
         */
        CurrentSensorArray(const Config& current_sensor_config);

        /**
         * @brief Get the current from the sensor
         *
         * @param sensor_index Index of the sensor
         * @return uint16_t Current reading from the sensor
         */
        uint16_t get_current(uint8_t sensor_index);

    private:
        /**
         * @brief ADC DMA handle.
         */
        hal::AdcDma current_sensor_adc;

        /**
         * @brief Buffer to store the ADC values
         */
        volatile uint32_t dma_buffer[num_of_sensors];
};
}  // namespace proxy

#endif // __CURRENT_SENSOR_ARRAY_HPP__
