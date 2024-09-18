/**
 * @file storage.hpp
 *
 * @brief Proxy Storage class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_STORAGE_HPP
#define MICRAS_PROXY_STORAGE_HPP

#include <bit>
#include <cstdint>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "serializable_interface.hpp"

namespace micras::proxy {
template <typename T>
concept Fundamental = std::is_fundamental_v<T>;

/**
 * @brief Class for controlling the storage
 */
class Storage {
public:
    /**
     * @brief Configuration structure for the storage
     */
    struct Config {
        uint16_t start_page;
        uint16_t number_of_pages;
    };

    /**
     * @brief Constructor for the Storage class
     *
     * @param config Configuration for the storage
     */
    explicit Storage(const Config& config);

    /**
     * @brief Create a new primitive variable in the storage
     *
     * @tparam T Type of the variable
     * @param name Name of the variable
     * @param data Reference to the variable
     */
    template <Fundamental T>
    void create(const std::string& name, const T& data) {
        this->primitives[name].ram_pointer = &data;
        this->primitives.at(name).size = sizeof(T);
    }

    /**
     * @brief Create a new serializable variable in the storage
     *
     * @param name Name of the variable
     * @param data Reference to the variable
     */
    void create(const std::string& name, const ISerializable& data);

    /**
     * @brief Sync a primitive variable with the storage
     *
     * @tparam T Type of the variable
     * @param name Name of the variable
     * @param data Reference to the variable
     */
    template <Fundamental T>
    void sync(const std::string& name, T& data) {
        if (this->primitives.contains(name) and this->primitives.at(name).ram_pointer == nullptr) {
            data = std::bit_cast<T&>(this->buffer.at(this->primitives.at(name).buffer_address));
        }

        this->create<T>(name, data);
    }

    /**
     * @brief Sync a serializable variable with the storage
     *
     * @param name Name of the variable
     * @param data Reference to the variable
     */
    void sync(const std::string& name, ISerializable& data);

    /**
     * @brief Save the storage to the flash
     */
    void save();

private:
    /**
     * @brief Structure for primitive variables
     */
    struct PrimitiveVariable {
        const void* ram_pointer{nullptr};
        uint16_t    buffer_address{};
        uint16_t    size{};
    };

    /**
     * @brief Structure for serializable variables
     */
    struct SerializableVariable {
        const ISerializable* ram_pointer{nullptr};
        uint16_t             buffer_address{};
        uint16_t             size{};
    };

    /**
     * @brief Serialize a map of variables
     *
     * @tparam T Type of the variables
     * @param variables Map of variables
     *
     * @return std::vector<uint8_t> Serialized buffer
     */
    template <typename T>
    static std::vector<uint8_t> serialize_var_map(const std::unordered_map<std::string, T>& variables);

    /**
     * @brief Deserialize a map of variables
     *
     * @tparam T Type of the variables
     * @param buffer Serialized buffer
     * @param num_vars Number of variables
     *
     * @return std::unordered_map<std::string, T> Deserialized map of variables
     */
    template <typename T>
    static std::unordered_map<std::string, T> deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars);

    /**
     * @brief Start symbol to avoid reading garbage from flash
     */
    static constexpr uint16_t start_symbol = 0xABAB;

    /**
     * @brief Map of primitive variables
     */
    std::unordered_map<std::string, PrimitiveVariable> primitives;

    /**
     * @brief Map of serializable variables
     */
    std::unordered_map<std::string, SerializableVariable> serializables;

    /**
     * @brief Serialized buffer for the storage
     */
    std::vector<uint8_t> buffer;

    /**
     * @brief Start page of the storage in the flash memory
     */
    uint16_t start_page;

    /**
     * @brief Maximum number of pages used by the storage in the flash memory
     */
    uint16_t number_of_pages;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_STORAGE_HPP
