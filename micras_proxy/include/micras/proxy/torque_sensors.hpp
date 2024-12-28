/**
 * @file
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_HPP
#define MICRAS_PROXY_TORQUE_SENSORS_HPP

#include <array>
#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/hal/adc_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring torque sensors data.
 */
template <uint8_t num_of_sensors>
class TTorqueSensors {
public:
    /**
     * @brief Configuration struct for torque sensors.
     */
    struct Config {
        hal::AdcDma::Config adc;
        float               shunt_resistor;
        float               max_torque;
        float               filter_cutoff;
    };

    /**
     * @brief Construct a newTorqueSensors object.
     *
     * @param config Configuration for the torque sensors.
     */
    explicit TTorqueSensors(const Config& config);

    /**
     * @brief Calibrate the torque sensors.
     */
    void calibrate();

    /**
     * @brief Update the torque sensors readings.
     */
    void update();

    /**
     * @brief Get the torque from the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Torque reading from the sensor in N*m.
     */
    float get_torque(uint8_t sensor_index) const;

    /**
     * @brief Get the raw torque from the sensor without filtering.
     *
     * @param sensor_index Index of the sensor.
     * @return Raw torque reading from the sensor.
     */
    float get_torque_raw(uint8_t sensor_index) const;

    /**
     * @brief Get the current from the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Current reading from the sensor in amps.
     */
    float get_current(uint8_t sensor_index) const;

    /**
     * @brief Get the raw current from the sensor without filtering.
     *
     * @param sensor_index Index of the sensor.
     * @return Raw current reading from the sensor.
     */
    float get_current_raw(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Adc reading from the sensor from 0 to 1.
     */
    float get_adc_reading(uint8_t sensor_index) const;

private:
    /**
     * @brief ADC DMA handle.
     */
    hal::AdcDma adc;

    /**
     * @brief Buffer to store the ADC values.
     */
    std::array<uint16_t, num_of_sensors> buffer;

    /**
     * @brief Reading of each sensor when no current is flowing.
     */
    std::array<float, num_of_sensors> base_reading{};

    /**
     * @brief Value of the maximum current that can be measured by the sensor.
     */
    float max_current;

    /**
     * @brief Maximum torque that can be measured by the sensor.
     */
    float max_torque;

    /**
     * @brief Butterworth filters for the torque reading.
     */
    std::array<core::ButterworthFilter, num_of_sensors> filters;
};
}  // namespace micras::proxy

#include "../src/torque_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_TORQUE_SENSORS_HPP
