/**
 * @file wall_sensors.hpp
 *
 * @brief Proxy WallSensors class header
 *
 * @date 03/2024
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
 * @brief Class for controlling WallSensors
 */
template <uint8_t num_of_sensors>
class WallSensors {
public:
    /**
     * @brief Configuration structure for wall sensors
     */
    struct Config {
        hal::AdcDma::Config               adc;
        hal::Pwm::Config                  led_0_pwm;
        hal::Pwm::Config                  led_1_pwm;
        float                             filter_cutoff;
        float                             uncertainty;
        std::array<float, num_of_sensors> wall_threshold;
        std::array<float, num_of_sensors> free_threshold;
    };

    /**
     * @brief Constructor for the WallSensors class
     *
     * @param config Configuration for the wall sensors
     */
    explicit WallSensors(const Config& config);

    /**
     * @brief Turn on the wall sensors IR LED
     */
    void turn_on();

    /**
     * @brief Turn off the wall sensors IR LED
     */
    void turn_off();

    /**
     * @brief Update the wall sensors readings
     */
    void update();

    /**
     * @brief Get the observation from a sensor
     *
     * @param sensor_index Index of the sensor
     * @return core::Observation Observation from the sensor
     */
    core::Observation get_observation(uint8_t sensor_index) const;

    /**
     * @brief Get the reading from a sensor
     *
     * @param sensor_index Index of the sensor
     * @return float Reading from the sensor
     */
    float get_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the ADC reading from a sensor
     *
     * @param sensor_index Index of the sensor
     * @return float ADC reading from the sensor from 0 to 1
     */
    float get_adc_reading(uint8_t sensor_index) const;

    /**
     * @brief Calibrate the wall sensors for a wall at the front
     */
    void calibrate_front_wall();

    /**
     * @brief Calibrate the wall sensors for a wall at the left
     */
    void calibrate_left_wall();

    /**
     * @brief Calibrate the wall sensors for a wall at the right
     */
    void calibrate_right_wall();

    /**
     * @brief Calibrate the wall sensors for free space at the front
     */
    void calibrate_front_free_space();

    /**
     * @brief Calibrate the wall sensors for free space at the left
     */
    void calibrate_left_free_space();

    /**
     * @brief Calibrate the wall sensors for free space at the right
     */
    void calibrate_right_free_space();

    /**
     * @brief Update the wall sensors thresholds
     */
    void update_thresholds();

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
    std::array<uint16_t, 2 * num_of_sensors> buffer;

    /**
     * @brief Butterworth filter for the ADC readings
     */
    std::array<core::ButterworthFilter, num_of_sensors> filters;

    /**
     * @brief Uncertainty of the wall sensors
     */
    float uncertainty;

    /**
     * @brief Measured wall value during calibration
     */
    std::array<float, num_of_sensors> wall_calibration_measure;

    /**
     * @brief Measured free space value during calibration
     */
    std::array<float, num_of_sensors> free_space_calibration_measure;

    /**
     * @brief Minimum reading value to identify a wall
     */
    std::array<float, num_of_sensors> wall_threshold;

    /**
     * @brief Maximum reading value to identify a free space
     */
    std::array<float, num_of_sensors> free_space_threshold;

    friend class Interface;
};
}  // namespace micras::proxy

#include "../src/wall_sensors.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP
