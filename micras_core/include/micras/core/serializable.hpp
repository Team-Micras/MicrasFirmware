/**
 * @file
 */

#ifndef MICRAS_CORE_SERIALIZABLE_HPP
#define MICRAS_CORE_SERIALIZABLE_HPP

#include <cstdint>
#include <vector>

namespace micras::core {
/**
 * @brief Interface class for serializable classes.
 */
class ISerializable {
public:
    /**
     * @brief Virtual destructor for the ISerializable class.
     */
    virtual ~ISerializable() = default;

    /**
     * @brief Serialize the class instance.
     *
     * @return Serialized data.
     */
    virtual std::vector<uint8_t> serialize() const = 0;

    /**
     * @brief Deserialize the class instance.
     *
     * @param serial_data Serialized data.
     * @param size Size of the serialized data.
     */
    virtual void deserialize(const uint8_t* serial_data, uint16_t size) = 0;

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    ISerializable() = default;
    ISerializable(const ISerializable&) = default;
    ISerializable(ISerializable&&) = default;
    ISerializable& operator=(const ISerializable&) = default;
    ISerializable& operator=(ISerializable&&) = default;
    ///@}
};
}  // namespace micras::core

#endif  // MICRAS_CORE_SERIALIZABLE_HPP
