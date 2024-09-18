/**
 * @file torque_sensors.hpp
 *
 * @brief Proxy TorqueSensors class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_HPP
#define MICRAS_PROXY_TORQUE_SENSORS_HPP

#include <array>
#include <cstdint>

#include "micras/hal/adc_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring torque sensors data
 */
template <uint8_t num_of_sensors>
class TorqueSensors {
public:
    /**
     * @brief Configuration structure for torque sensors
     */
    struct Config {
        hal::AdcDma::Config adc;
        float               shunt_resistor;
        float               max_torque;
    };

    /**
     * @brief Constructor for the TorqueSensors class
     *
     * @param config Configuration for the torque sensors
     */
    explicit TorqueSensors(const Config& config);

    /**
     * @brief Get the torque from the sensor
     *
     * @param sensor_index Index of the sensor
     * @return float Torque reading from the sensor in N*m
     */
    float get_torque(uint8_t sensor_index) const;

    /**
     * @brief Get the current from the sensor
     *
     * @param sensor_index Index of the sensor
     * @return float Current reading from the sensor in amps
     */
    float get_current(uint8_t sensor_index) const;

private:
    /**
     * @brief ADC DMA handle
     */
    hal::AdcDma adc;

    /**
     * @brief Buffer to store the ADC values
     */
    std::array<uint16_t, num_of_sensors> buffer;

    /**
     * @brief Reading of each sensor when no current is flowing
     */
    std::array<uint16_t, num_of_sensors> base_reading;

    /**
     * @brief Value of the shunt resistor in ohms
     */
    float shunt_resistor;

    /**
     * @brief Maximum torque that can be measured by the sensor
     */
    float max_torque;
};
}  // namespace micras::proxy

#include "../src/torque_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_TORQUE_SENSORS_HPP
