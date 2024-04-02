/**
 * @file serializable_interface.hpp
 *
 * @brief Serializable interface for all classes that need to be serialized
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_SERIALIZABLE_INTERFACE_HPP
#define MICRAS_PROXY_SERIALIZABLE_INTERFACE_HPP

#include <cstdint>
#include <vector>

class ISerializable {
    public:
        virtual ~ISerializable() = default;

        virtual std::vector<uint8_t> serialize() const = 0;

        virtual void deserialize(uint8_t* serial_data, uint16_t size) = 0;
};

#endif // MICRAS_PROXY_SERIALIZABLE_INTERFACE_HPP
