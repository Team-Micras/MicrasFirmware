/**
 * @file
 */

#ifndef MICRAS_PROXY_WALL_SENSORS_HPP
#define MICRAS_PROXY_WALL_SENSORS_HPP

#include <array>
#include <cstdint>

#include "micras/core/butterworth_filter.hpp"
#include "micras/core/types.hpp"
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
        hal::AdcDma::Config               adc;
        hal::Pwm::Config                  led_0_pwm;
        hal::Pwm::Config                  led_1_pwm;
        float                             filter_cutoff;
        std::array<float, num_of_sensors> base_readings;
        float                             uncertainty;
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
     * @brief Get the observations from all sensors.
     *
     * @return Observations from all sensors.
     */
    core::Observation get_observation() const;

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
     * @brief Calibrate the wall sensors for a wall at the front.
     */
    void calibrate_front_wall();

    /**
     * @brief Calibrate the wall sensors for a wall at the left.
     */
    void calibrate_left_wall();

    /**
     * @brief Calibrate the wall sensors for a wall at the right.
     */
    void calibrate_right_wall();

private:
    /**
     * @brief ADC DMA handle.
     */
    hal::AdcDma adc;

    /**
     * @brief PWM handle for the even infrared LEDs.
     */
    hal::Pwm led_0_pwm;

    /**
     * @brief PWM handle for the odd infrared LEDs.
     */
    hal::Pwm led_1_pwm;

    /**
     * @brief Buffer to store the ADC values.
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
};
}  // namespace micras::proxy

#include "../src/wall_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP
