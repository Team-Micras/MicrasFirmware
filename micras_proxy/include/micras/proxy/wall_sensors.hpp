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
 *
 * @details Each sensor is an infrared emitter next to a phototransistor, and what comes out of this
 * class is the distance to whatever the emitter lights up, along its optical axis, in meters. The
 * emitter has a narrow beam that lands entirely on the wall, which scatters it in every direction,
 * so the light that comes back falls with the square of the distance. One reading at a known
 * distance therefore calibrates a sensor:
 *
 *     distance = reference_distance * sqrt(reference_reading / reading)
 *
 * @tparam num_of_sensors Number of sensors.
 */
template <uint8_t num_of_sensors>
class TWallSensors {
public:
    /**
     * @brief Configuration struct for wall sensors.
     *
     * @note The reference reading of a sensor is what it reads at its reference distance, and is
     * what a calibration replaces. Both filters run at the rate the sensors produce values, not at
     * the rate of the control loop, so that is the sampling frequency they have to be given.
     * A reading below the noise floor, or one that works out to more than the maximum distance,
     * means nothing is within range. A reading above the maximum is saturated, and is reported as
     * the distance of the maximum reading. A wall is considered present when the slow distance is
     * below the wall distance, and absent again when it goes above it by the hysteresis.
     */
    struct Config {
        hal::AdcDma::Config                          adc;
        std::array<hal::Pwm::Config, num_of_sensors> led_pwms;
        float                                        emitter_duty_cycle;
        core::ButterworthFilter::Config              fast_filter;
        core::ButterworthFilter::Config              slow_filter;
        std::array<float, num_of_sensors>            reference_readings;
        std::array<float, num_of_sensors>            reference_distances;
        float                                        noise_floor;
        float                                        max_reading;
        float                                        max_distance;
        float                                        wall_distance;
        float                                        wall_hysteresis;
        uint16_t                                     calibration_samples;
    };

    /**
     * @brief Distance measured by one sensor.
     *
     * @note The fast distance is for everything that is a position and the slow one for deciding
     * whether there is a wall. The reading is valid when the sensor sees anything above its noise,
     * and it is new for a single update after the sensor produces a value.
     */
    struct Reading {
        float distance;
        float slow_distance;
        bool  valid;
        bool  is_new;
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
     *
     * @note Nothing is recomputed unless the converter completed a sequence since the last call,
     * so this can be called faster than the sensors produce values.
     */
    void update();

    /**
     * @brief Get the distance measured by a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return The reading of the sensor.
     */
    const Reading& get_reading(uint8_t sensor_index) const;

    /**
     * @brief Get the observation from a sensor.
     *
     * @param sensor_index Index of the sensor.
     * @return True if the sensor detects a wall, false otherwise.
     */
    bool get_wall(uint8_t sensor_index) const;

    /**
     * @brief Get the light a sensor receives from its emitter, with the ambient light removed.
     *
     * @param sensor_index Index of the sensor.
     * @return Reading from the sensor from 0 to 1.
     */
    float get_intensity(uint8_t sensor_index) const;

    /**
     * @brief Start calibrating a sensor, with the robot placed at the reference distance.
     *
     * @note The readings are averaged over the configured number of samples, which takes that many
     * updates with a new value.
     *
     * @param sensor_index Index of the sensor.
     */
    void calibrate_sensor(uint8_t sensor_index);

    /**
     * @brief Check if a calibration is in progress.
     *
     * @return True while any sensor is still averaging.
     */
    bool is_calibrating() const;

    /**
     * @brief Get the reference reading of a sensor, which is the result of its last calibration.
     *
     * @param sensor_index Index of the sensor.
     * @return The reading at the reference distance, from 0 to 1.
     */
    float get_reference_reading(uint8_t sensor_index) const;

    /**
     * @brief Get how much the readings varied during the last calibration of a sensor.
     *
     * @note A spread that is not small means the robot moved, or something else was wrong, while
     * the sensor was being calibrated.
     *
     * @param sensor_index Index of the sensor.
     * @return The standard deviation of the readings, as a fraction of their mean.
     */
    float get_calibration_spread(uint8_t sensor_index) const;

    /**
     * @brief Check if the ADC was initialized, its scan matches the buffer layout, and the emitters
     * run at the rate the filters were designed for.
     *
     * @note The rate of the emitters lives in the peripheral configuration and the one of the filters
     * in the constants, and nothing else would notice the day only one of them changes.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Largest relative difference between the rate of the emitters and the one of the filters.
     */
    static constexpr float frequency_tolerance{0.01F};

    /**
     * @brief Averaging of the readings of one sensor during its calibration.
     */
    struct Calibration {
        float    sum;
        float    squared_sum;
        uint16_t samples_left;
        float    spread;
    };

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
     * @brief Butterworth filters for the fast distances.
     */
    std::array<core::ButterworthFilter, num_of_sensors> fast_filters;

    /**
     * @brief Butterworth filters for the slow distances.
     */
    std::array<core::ButterworthFilter, num_of_sensors> slow_filters;

    /**
     * @brief Reading of each sensor at its reference distance.
     */
    std::array<float, num_of_sensors> reference_readings;

    /**
     * @brief Distance each sensor was calibrated at.
     */
    std::array<float, num_of_sensors> reference_distances;

    /**
     * @brief Reading below which there is only noise.
     */
    float noise_floor;

    /**
     * @brief Distance beyond which nothing is considered within range.
     */
    float max_distance;

    /**
     * @brief Reading above which the sensor is saturated.
     */
    float max_reading;

    /**
     * @brief Slow distance below which a wall is present.
     */
    float wall_distance;

    /**
     * @brief How far above the wall distance the slow distance has to go for the wall to be absent.
     */
    float wall_hysteresis;

    /**
     * @brief Number of readings averaged by a calibration.
     */
    uint16_t calibration_samples;

    /**
     * @brief Distance measured by each sensor.
     */
    std::array<Reading, num_of_sensors> readings{};

    /**
     * @brief Whether each sensor detects a wall.
     */
    std::array<bool, num_of_sensors> walls{};

    /**
     * @brief Calibration of each sensor.
     */
    std::array<Calibration, num_of_sensors> calibrations{};

    /**
     * @brief Flag to check if the ADC was initialized.
     */
    bool initialized{};
};
}  // namespace micras::proxy

#include "micras/proxy/impl/wall_sensors.tpp"  // IWYU pragma: export

#endif  // MICRAS_PROXY_WALL_SENSORS_HPP
