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
     *
     * @note The zero reading is the reading with no current, as a fraction of the range of the
     * converter, which is where a bidirectional amplifier sits. The currents and torques are signed
     * about it.
     */
    struct Config {
        hal::AdcDma::Config             adc;
        float                           shunt_resistor;
        float                           zero_reading;
        float                           max_torque;
        core::ButterworthFilter::Config filter;
    };

    /**
     * @brief Construct a new TorqueSensors object.
     *
     * @param config Configuration for the torque sensors.
     */
    explicit TTorqueSensors(const Config& config);

    /**
     * @brief Calibrate the torque sensors.
     *
     * @note The readings start out signed about the zero reading of the configuration, which leaves
     * the offset of each amplifier. This takes the current reading as the one of no current, so it
     * is to be called with the motors disabled. Calling it again refines the baseline rather than
     * discarding it.
     */
    void calibrate();

    /**
     * @brief Update the torque sensors readings.
     *
     * @note Restarts the converter first if an error stopped it.
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
     * @brief Get the electric current through the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Current reading from the sensor in amps.
     */
    float get_current(uint8_t sensor_index) const;

    /**
     * @brief Get the raw electric current through the sensor without filtering.
     *
     * @param sensor_index Index of the sensor.
     * @return Raw current reading from the sensor in amps.
     */
    float get_current_raw(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from the sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Adc reading from the sensor from 0 to 1.
     */
    float get_adc_reading(uint8_t sensor_index) const;

    /**
     * @brief Check if the ADC was successfully initialized.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief ADC DMA handle.
     */
    hal::AdcDma adc;

    /**
     * @brief Buffer to store the ADC values.
     */
    std::array<uint16_t, num_of_sensors> buffer{};

    /**
     * @brief Reading of each sensor when no current is flowing.
     */
    std::array<float, num_of_sensors> base_reading{};

    /**
     * @brief Current that moves the reading across the whole range of the converter, in amps.
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

    /**
     * @brief Flag to check if the ADC was initialized.
     */
    bool initialized{};
};
}  // namespace micras::proxy

#include "micras/proxy/impl/torque_sensors.tpp"  // IWYU pragma: export

#endif  // MICRAS_PROXY_TORQUE_SENSORS_HPP
