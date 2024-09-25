/**
 * @file distance_sensors.hpp
 *
 * @brief Proxy DistanceSensors class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_DISTANCE_SENSORS_HPP
#define MICRAS_PROXY_DISTANCE_SENSORS_HPP

#include <array>
#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/hal/adc_dma.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling DistanceSensors
 */
template <uint8_t num_of_sensors>
class DistanceSensors {
public:
    /**
     * @brief Configuration structure for distance sensors
     */
    struct Config {
        hal::AdcDma::Config adc;
        hal::Pwm::Config    led_0_pwm;
        hal::Pwm::Config    led_1_pwm;
        float               max_distance;
        float               filter_cutoff;
    };

    /**
     * @brief Constructor for the DistanceSensors class
     *
     * @param config Configuration for the distance sensors
     */
    explicit DistanceSensors(const Config& config);

    /**
     * @brief Set the distance sensors led intensity
     *
     * @param intensity Intensity percentage of the infrared LED
     */
    void set_led_intensity(float intensity);

    /**
     * @brief Update the distance sensors readings
     */
    void update();

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
     * @return float Raw reading from the distance sensor
     */
    float get_distance_raw(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from a sensor
     *
     * @param sensor_index Index of the sensor
     * @return float ADC reading from the sensor from 0 to 1
     */
    float get_adc_reading(uint8_t sensor_index) const;

private:
    /**
     * @brief ADC DMA handle
     */
    hal::AdcDma adc;

    /**
     * @brief PWM handle for the even infrared LEDs
     */
    hal::Pwm led_0_pwm;

    /**
     * @brief PWM handle for the odd infrared LEDs
     */
    hal::Pwm led_1_pwm;

    /**
     * @brief Buffer to store the ADC values
     */
    std::array<uint16_t, num_of_sensors> buffer;

    /**
     * @brief Maximum distance reading in meters
     */
    float max_distance;

    /**
     * @brief Butterworth filter for the ADC readings
     */
    std::array<core::ButterworthFilter<2>, num_of_sensors> filters;
};
}  // namespace micras::proxy

#include "../src/distance_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_DISTANCE_SENSORS_HPP
