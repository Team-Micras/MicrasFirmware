/**
 * @file
 */

#ifndef MICRAS_CALIBRATION_RECORD_HPP
#define MICRAS_CALIBRATION_RECORD_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "micras/core/serializable.hpp"
#include "micras/nav/wall_model.hpp"

namespace micras {
/**
 * @brief Results of the calibrations measured on the robot, kept in the flash memory with the maze.
 *
 * @details The configuration in the repository is what the robot runs on, and a measured value
 * only stands in for it until it is written there. So each value keeps the one of the
 * configuration it replaced, and is used at boot only while the configuration still holds that
 * one: once the measured value is committed, or the configuration changes for any other reason,
 * the firmware that is flashed next runs on the configuration and the stored value is forgotten.
 */
class CalibrationRecord : public core::ISerializable {
public:
    /**
     * @brief A measured value and the value of the configuration it replaced.
     */
    struct Value {
        float measured{};
        float replaced{};
        bool  present{};
    };

    /**
     * @brief Record a measured value.
     *
     * @param value The value to record into.
     * @param measured What was measured.
     * @param configured What the configuration holds for it.
     */
    static void record(Value& value, float measured, float configured);

    /**
     * @brief Choose the stored value over the configured one, if it still applies.
     *
     * @note A value that replaced something else than what the configuration now holds, or that is
     * out of the range it can have, is forgotten.
     *
     * @param value The stored value.
     * @param configured What the configuration holds.
     * @param min Smallest value that makes sense.
     * @param max Largest value that makes sense.
     * @return The stored value, if it applies.
     */
    static std::optional<float> choose(Value& value, float configured, float min, float max);

    /**
     * @brief Serialize the record.
     *
     * @return A version byte, then each value as its presence and the two floats.
     */
    std::vector<uint8_t> serialize() const override;

    /**
     * @brief Deserialize the record, leaving it empty if it was written by another version.
     *
     * @param serial_data Serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override;

    /**
     * @brief Measured values.
     */
    ///@{
    std::array<Value, nav::number_of_wall_sensors> wall_reference_readings{};
    std::array<Value, nav::number_of_wall_sensors> wall_offsets{};
    Value                                          gyroscope_scale{};
    ///@}

private:
    /**
     * @brief Version of the serialized layout.
     */
    static constexpr uint8_t version{1};

    /**
     * @brief Bytes a value takes once serialized.
     */
    static constexpr uint16_t value_size{1 + 2 * sizeof(float)};

    /**
     * @brief Number of values in the record.
     */
    static constexpr uint16_t number_of_values{2 * nav::number_of_wall_sensors + 1};

    /**
     * @brief Call a function on every value, in the serialized order.
     *
     * @param function The function.
     */
    template <typename Function>
    void for_each(Function function);

    /**
     * @brief Call a function on every value, in the serialized order.
     *
     * @param function The function.
     */
    template <typename Function>
    void for_each(Function function) const;
};
}  // namespace micras

#endif  // MICRAS_CALIBRATION_RECORD_HPP
