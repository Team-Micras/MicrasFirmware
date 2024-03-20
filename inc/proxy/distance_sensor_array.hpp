/**
 * @file distance_sensor_array.hpp
 *
 * @brief Proxy DistanceSensorArray class header
 *
 * @date 03/2024
 */

#ifndef __DISTANCE_SENSOR_ARRAY_HPP__
#define __DISTANCE_SENSOR_ARRAY_HPP__

#include <cstdint>

#include "hal/adc_dma.hpp"
#include "hal/tim.hpp"

namespace proxy {
/**
 * @brief Class for controlling DistanceSensorArray.
 */
template <uint8_t num_of_sensors>
class DistanceSensorArray {
    public:
        /**
         * @brief configuration structure for distance sensor
         */
        struct Config {
            hal::AdcDma::Config sensors_adc_config;
            hal::Tim::Config    infrared_tim_config;
            uint32_t            infrared_led_channel;
        };

        /**
         * @brief Constructor for the DistanceSensorArray class
         *
         * @param distance_sensor_config Configuration for the distance sensor
         */
        DistanceSensorArray(const Config& distance_sensor_config);

        /**
         * @brief Set the infrared DISTANCE_SENSOR_ARRAY intensity
         *
         * @param intensity Intensity of the infrared DISTANCE_SENSOR_ARRAY
         */
        void set_infrared_led_intensity(uint8_t intensity);

        /**
         * @brief Get the distance from the sensor
         *
         * @param sensor_index Index of the sensor
         * @return uint16_t Distance reading from the sensor
         */
        uint16_t get_distance(uint8_t sensor_index);

    private:
        /**
         * @brief ADC DMA handle.
         */
        hal::AdcDma distance_sensor_adc;

        /**
         * @brief Timer handle for infrared LED
         */
        hal::Tim infrared_led_tim;

        /**
         * @brief Channel for infrared LED
         */
        uint32_t infrared_led_channel;

        /**
         * @brief Buffer to store the ADC values
         */
        volatile uint32_t dma_buffer[num_of_sensors];
};
}  // namespace proxy

#endif // __DISTANCE_SENSOR_ARRAY_HPP__
