/**
 * @file distance_sensors.hpp
 *
 * @brief Proxy DistanceSensors class header
 *
 * @date 03/2024
 */

#ifndef __DISTANCE_SENSORS_HPP__
#define __DISTANCE_SENSORS_HPP__

#include <cstdint>

#include "hal/adc_dma.hpp"
#include "hal/pwm.hpp"

namespace proxy {
/**
 * @brief Class for controlling DistanceSensors
 */
template <uint8_t num_of_sensors>
class DistanceSensors {
    public:
        /**
         * @brief configuration structure for distance sensors
         */
        struct Config {
            hal::AdcDma::Config adc;
            hal::Pwm::Config    infrared_pwm;
        };

        /**
         * @brief Constructor for the DistanceSensors class
         *
         * @param distance_sensor_config Configuration for the distance sensors
         */
        DistanceSensors(const Config& distance_sensor_config);

        /**
         * @brief Set the infrared DistanceSensors intensity
         *
         * @param intensity Intensity percentage of the infrared LED
         */
        void set_infrared_led_intensity(float intensity);

        /**
         * @brief Get the distance from a sensor
         *
         * @param sensor_index Index of the sensor
         * @return float Distance reading from the sensors
         */
        float get_distance(uint8_t sensor_index) const;

        /**
         * @brief Get the distance from a sensor
         *
         * @param sensor_index Index of the sensor
         * @return uint32_t Raw reading from the distance sensor
         */
        uint32_t get_distance_raw(uint8_t sensor_index) const;

    private:
        /**
         * @brief ADC DMA handle
         */
        hal::AdcDma distance_sensor_adc;

        /**
         * @brief PWM handle for infrared LED
         */
        hal::Pwm infrared_pwm;

        /**
         * @brief Buffer to store the ADC values
         */
        volatile std::array<uint32_t, num_of_sensors> dma_buffer;
};
}  // namespace proxy

#endif // __DISTANCE_SENSORS_HPP__
