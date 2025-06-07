/**
 * @file
 */

#ifndef MICRAS_PROXY_STORAGE_HPP
#define MICRAS_PROXY_STORAGE_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <pair>
#include <vector>

#include "micras/core/concepts.hpp"
#include "micras/core/serial/variable_pool.hpp"

namespace micras::proxy {
/**
 * @brief Class for storing variable and classes in the flash memory.
 *
 * @note Stored Data Layout
 *
 * | Offset | Description                       | Size (bytes) |
 * |--------|-----------------------------------|--------------|
 * | 0      | Start Symbol                      | 2            |
 * | 2      | Total Size                        | 2            |
 * | 4      | Number of Primitives              | 2            |
 * | 6      | Number of Serializables           | 2            |
 * | 8      | Primitive Serial Variable Map     | Variable     |
 * | ...    | Serializable Serial Variable Map  | Variable     |
 * | ...    | Serialized Data                   | Variable     |
 */
class Storage {
public:
    /**
     * @brief Configuration struct for the storage.
     */
    struct Config {
        uint16_t start_page;
        uint16_t number_of_pages;
    };

    /**
     * @brief Construct a new Storage object.
     *
     * @param config Configuration for the storage.
     */
    explicit Storage(const Config& config);

    /**
     * @brief Create a new primitive variable in the storage.
     *
     * @tparam T Type of the variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    // template <core::Fundamental T>
    // void create(const std::string& name, const T& data) {
    //     this->primitives[name].ram_pointer = &data;
    //     this->primitives.at(name).size = sizeof(T);
    // }

    template <typename T>
    requires(core::Fundamental<T> || core::Serializable<T>)
    void create(const std::string& name, const T& data) {
        this->serial_variable_pool.add_variable(name, data);
    }

    template <typename T>
    requires(core::Fundamental<T> || core::Serializable<T>)
    void sync(const std::string& name, T& data) {
        if (this->buffer_views.contains(name) and this->buffer_views.at(name).second == 0) {
            this->serial_variable_pool.add_variable(name, data);
        }
    }

    /**
     * @brief Sync a primitive variable with the storage.
     *
     * @tparam T Type of the variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    template <core::Fundamental T>
    void sync(const std::string& name, T& data) {
        if (this->primitives.contains(name) and this->primitives.at(name).ram_pointer == nullptr) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            data = reinterpret_cast<T&>(this->buffer.at(this->primitives.at(name).buffer_address));
        }

        this->create<T>(name, data);
    }

    /**
     * @brief Save the storage to the flash.
     */
    void save();

private:
    void serialize_variables_data();

    /**
     * @brief Start symbol to avoid reading garbage from flash.
     */
    static constexpr uint32_t start_symbol = 0xABABABAB;

    core::SerialVariablePool serial_variable_pool;

    std::unordered_map<std::string, std::pair<uint16_t, uint16_t>> buffer_views;

    /**
     * @brief Serialized buffer for the storage.
     */
    std::vector<uint8_t> buffer;

    /**
     * @brief Start page of the storage in the flash memory.
     */
    uint16_t start_page;

    /**
     * @brief Maximum number of pages used by the storage in the flash memory.
     */
    uint16_t number_of_pages;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_STORAGE_HPP
