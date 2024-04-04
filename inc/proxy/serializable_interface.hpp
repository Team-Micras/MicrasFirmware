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

/**
 * @brief Interface class for serializable classes
 */
class ISerializable {
    public:
        /**
         * @brief Virtual destructor for the ISerializable class
         */
        virtual ~ISerializable() = default;

        /**
         * @brief Serialize the class instance
         *
         * @return std::vector<uint8_t> Serialized data
         */
        virtual std::vector<uint8_t> serialize() const = 0;

        /**
         * @brief Deserialize the class instance
         *
         * @param serial_data Serialized data
         * @param size Size of the serialized data
         */
        virtual void deserialize(uint8_t* serial_data, uint16_t size) = 0;

    protected:
        /**
         * @brief Special member functions declared as default
         */
        ISerializable(const ISerializable&) = default;

        ISerializable(ISerializable&&) = default;

        ISerializable& operator =(const ISerializable&) = default;

        ISerializable& operator =(ISerializable&&) = default;
};

#endif // MICRAS_PROXY_SERIALIZABLE_INTERFACE_HPP
