/**
 * @file distance_sensors.hpp
 *
 * @brief Proxy DistanceSensors class header
 *
 * @date 03/2024
 */

#ifndef __DISTANCE_SENSORS_HPP__
#define __DISTANCE_SENSORS_HPP__

#include <array>
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
            hal::Pwm::Config    led_pwm;
        };

        /**
         * @brief Constructor for the DistanceSensors class
         *
         * @param distance_sensor_config Configuration for the distance sensors
         */
        DistanceSensors(Config& distance_sensor_config);

        /**
         * @brief Set the distance sensors led intensity
         *
         * @param intensity Intensity percentage of the infrared LED
         */
        void set_led_intensity(float intensity);

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
        hal::AdcDma adc;

        /**
         * @brief PWM handle for infrared LED
         */
        hal::Pwm led_pwm;

        /**
         * @brief Buffer to store the ADC values
         */
        std::array<uint32_t, num_of_sensors> buffer;

        static constexpr float max_distance{0.3f};

        static constexpr uint32_t max_adc_reading{4095};
};
}  // namespace proxy

#include "../src/proxy/distance_sensors.cpp"

#endif // __DISTANCE_SENSORS_HPP__
