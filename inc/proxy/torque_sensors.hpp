/**
 * @file torque_sensors.hpp
 *
 * @brief Proxy TorqueSensors class header
 *
 * @date 03/2024
 */

#ifndef __TORQUE_SENSORS_HPP__
#define __TORQUE_SENSORS_HPP__

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
         * @brief configuration structure for torque sensors
         */
        struct Config {
            typename CurrentSensors<num_of_sensors>::Config current_sensors;
        };

        /**
         * @brief Constructor for the TorqueSensors class
         *
         * @param torque_sensors_config Configuration for the torque sensors
         */
        TorqueSensors(Config& torque_sensors_config);

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

        /**
         * @brief Maximum torque that can be measured by the sensor
         */
        static constexpr float max_torque = 50.0f;

    private:
        /**
         * @brief Current sensors object
         */
        CurrentSensors<num_of_sensors> current_sensors;
};
}  // namespace proxy

#include "../src/proxy/torque_sensors.cpp"

#endif // __TORQUE_SENSORS_HPP__
