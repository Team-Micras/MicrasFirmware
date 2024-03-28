/**
 * @file serializable_interface.hpp
 *
 * @brief Serializable interface for all classes that need to be serialized
 *
 * @date 03/2024
 */

#ifndef __SERIALIZABLE_INTERFACE_HPP__
#define __SERIALIZABLE_INTERFACE_HPP__

#include <cstdint>
#include <vector>

class ISerializable {
    public:
        virtual ~ISerializable() = default;

        virtual std::vector<uint8_t> serialize() = 0;

        virtual void deserialize(uint8_t* serial_data, uint16_t size) = 0;
};

#endif // __SERIALIZABLE_INTERFACE_HPP__
