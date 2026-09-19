/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_HPP
#define MICRAS_PROXY_WALL_SENSORS_HPP

#include <array>
#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/hal/adc_dma.hpp"
#include "micras/hal/pwm.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling Wall Sensors.
 */
template <uint8_t num_of_sensors>
class TWallSensors {
public:
    /**
     * @brief Configuration struct for wall sensors.
     */
    struct Config {
        hal::AdcDma::Config                          adc;
        std::array<hal::Pwm::Config, num_of_sensors> led_pwms;
        core::ButterworthFilter::Config              filter;
        std::array<float, num_of_sensors>            base_readings;
        float                                        uncertainty;
    };

    /**
     * @brief Construct a new WallSensors object.
     *
     * @param config Configuration for the wall sensors.
     */
    explicit TWallSensors(const Config& config);

    /**
     * @brief Turn on the wall sensors IR LED.
     */
    void turn_on();

    /**
     * @brief Turn off the wall sensors IR LED.
     */
    void turn_off();

    /**
     * @brief Update the wall sensors readings.
     */
    void update();

    /**
     * @brief Get the observation from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @param disturbed Whether or not there is another wall perpendicular to the one being measured.
     * @return True if the sensor detects a wall, false otherwise.
     */
    bool get_wall(uint8_t sensor_index, bool disturbed = false) const;

    /**
     * @brief Get the reading from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return Reading from the sensor.
     */
    float get_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return ADC reading from the sensor from 0 to 1.
     */
    float get_adc_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the deviation of a wall sensor reading from its calibrated baseline.
     *
     * @param sensor_index Index of the sensor.
     * @return The reading error relative to the baseline; positive if above baseline.
     */
    float get_sensor_error(uint8_t sensor_index) const;

    /**
     * @brief Calibrate a wall sensor base reading.
     */
    void calibrate_sensor(uint8_t sensor_index);

    /**
     * @brief Check if the ADC was initialized and its scan matches the buffer layout.
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
     * @brief PWM handles for the infrared LEDs, one for each sensor.
     */
    std::array<hal::Pwm, num_of_sensors> led_pwms;

    /**
     * @brief Buffer to store the ADC values, holding one emitter on and one emitter off scan.
     *
     * @details The emitter PWM timer is center aligned and triggers the ADC on its update event, so
     * the conversion sequence runs twice per emitter period: once at the underflow, while the
     * emitters are on, and once at the overflow, while they are off. Two consecutive scans
     * therefore fill the two halves of this buffer with a matching pair, and the difference between
     * the halves is the reflected signal with the ambient light canceled. Taking the absolute
     * value makes the result independent of which half currently holds which phase.
     *
     * @note This depends on the ADC scanning exactly num_of_sensors channels, on the emitter timer
     * being center aligned with its trigger on the update event, and on a scan fitting inside half
     * an emitter period. The first of those is checked by the constructor; the other two live in the
     * peripheral configuration.
     */
    std::array<uint16_t, 2 * num_of_sensors> buffer;

    /**
     * @brief Butterworth filter for the ADC readings.
     */
    std::array<core::ButterworthFilter, num_of_sensors> filters;

    /**
     * @brief Measured wall values during calibration.
     */
    std::array<float, num_of_sensors> base_readings;

    /**
     * @brief Ratio of the base reading to still consider as seeing a wall.
     */
    float uncertainty;

    /**
     * @brief Flag to check if the ADC was initialized.
     */
    bool initialized{};
};
}  // namespace micras::proxy

#include "../src/wall_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP
