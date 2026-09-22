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
        float                                        emitter_duty_cycle;
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
     * @brief Update the wall sensors readings, if the emitters completed a period since the last call.
     */
    void update();

    /**
     * @brief Check whether the last update brought readings that were not seen before.
     *
     * @note The readings come at the rate of the emitter timer, whatever the rate of the caller.
     *
     * @return True if the readings are new, false otherwise.
     */
    bool is_new() const;

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
     * @brief Check if the ADC was initialized, its scan matches the buffer layout, and the emitters
     * run at the rate the filter was designed for.
     *
     * @note The rate of the emitters lives in the peripheral configuration and the one of the filter
     * in the constants, and nothing else would notice the day only one of them changes.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Largest relative difference between the rate of the emitters and the one of the filter.
     */
    static constexpr float frequency_tolerance{0.01F};

    /**
     * @brief ADC DMA handle.
     */
    hal::AdcDma adc;

    /**
     * @brief PWM handles for the infrared LEDs, one for each sensor.
     */
    std::array<hal::Pwm, num_of_sensors> led_pwms;

    /**
     * @brief Duty cycle of the emitters while the sensors are on.
     */
    float emitter_duty_cycle;

    /**
     * @brief Buffer the DMA writes to, holding one emitter on and one emitter off scan.
     *
     * @details The emitter PWM timer is center aligned and triggers the ADC on its update event, so
     * the conversion sequence runs twice per emitter period: once at the underflow and once at the
     * overflow. The emitters fire in two groups, one centered on each of those instants, so every
     * scan reads half of the sensors lit and the other half dark, no sensor is ever lit by the
     * emitter of its neighbor, and two consecutive scans hold a lit and a dark reading of every
     * sensor. Their difference is the reflected signal with the ambient light canceled, and taking
     * its absolute value makes the result independent of which half currently holds which scan.
     *
     * @note This depends on the ADC scanning exactly num_of_sensors channels, on the emitter timer
     * being center aligned with its trigger on the update event, on the two groups being told
     * apart by the inverted flag of their PWM configuration, and on the emitter being on for at
     * least the settling time of the receiver before the scan starts and the duration of the scan
     * after it. The first of those is checked by the constructor; the others live in the
     * configuration.
     */
    std::array<uint16_t, 2 * num_of_sensors> buffer{};

    /**
     * @brief Copy of the buffer taken when a pair of scans completes, so that a pair is never torn.
     */
    std::array<uint16_t, 2 * num_of_sensors> snapshot{};

    /**
     * @brief The pair of scans the readings are computed from.
     */
    std::array<uint16_t, 2 * num_of_sensors> scans{};

    /**
     * @brief Number of pairs of scans completed when the readings were last computed.
     */
    uint32_t sequence{};

    /**
     * @brief Flag to check if the last update brought new readings.
     */
    bool fresh{};

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

#include "micras/proxy/impl/wall_sensors.tpp"  // IWYU pragma: export

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP
