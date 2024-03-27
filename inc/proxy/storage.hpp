/**
 * @file storage.hpp
 *
 * @brief Proxy Storage class declaration
 *
 * @date 03/2024
 */

#ifndef __STORAGE_HPP__
#define __STORAGE_HPP__

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "serializable_interface.hpp"

namespace proxy {
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
        Storage(Config& config);

        template <typename T>
        void create(const std::string& name, T& data);

        void create(const std::string& name, ISerializable& data);

        template <typename T>
        void sync(const std::string& name, T& data);

        void sync(const std::string& name, ISerializable& data);

        void save();

    private:
        struct Variable {
            void*    ram_address;
            uint16_t buffer_address;
            uint16_t size;
        };

        void serialize_var_map();

        std::unordered_map<std::string, Variable> variables;

        std::vector<uint8_t> buffer;
        uint16_t start_page;
};
}  // namespace proxy

#endif // __STORAGE_HPP__
