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
        hal::Pwm::Config    led0_pwm;
        hal::Pwm::Config    led1_pwm;
        float               max_distance;
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
     * @brief PWM handle for the even infrared LEDs
     */
    hal::Pwm led0_pwm;

    /**
     * @brief PWM handle for the odd infrared LEDs
     */
    hal::Pwm led1_pwm;

    /**
     * @brief Buffer to store the ADC values
     */
    std::array<uint16_t, num_of_sensors> buffer;

    /**
     * @brief Maximum distance reading in meters
     */
    const float max_distance;
};
}  // namespace micras::proxy

#include "../src/distance_sensors.cpp"  // NOLINT(bugprone-suspicious-include)

#endif  // MICRAS_PROXY_DISTANCE_SENSORS_HPP
