/**
 * @file torque_sensors.hpp
 *
 * @brief Proxy TorqueSensors class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_TORQUE_SENSORS_HPP
#define MICRAS_PROXY_TORQUE_SENSORS_HPP

#include <cstdint>

#include "proxy/current_sensors.hpp"

namespace proxy {
/**
 * @brief Class for controlling TorqueSensors
 */
template <uint8_t num_of_sensors>
class TorqueSensors {
    public:
        /**
         * @brief Configuration structure for torque sensors
         */
        struct Config {
            typename CurrentSensors<num_of_sensors>::Config current_sensors;
            float                                           max_torque;
        };

        /**
         * @brief Constructor for the TorqueSensors class
         *
         * @param config Configuration for the torque sensors
         */
        TorqueSensors(Config& config);

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
         * @brief Current sensors object
         */
        CurrentSensors<num_of_sensors> current_sensors;

        /**
         * @brief Maximum torque that can be measured by the sensor
         */
        const float max_torque;
};
}  // namespace proxy

#include "../src/proxy/torque_sensors.cpp"

#endif // MICRAS_PROXY_TORQUE_SENSORS_HPP
