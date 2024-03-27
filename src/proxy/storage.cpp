/**
 * @file storage.cpp
 *
 * @brief Proxy Storage class source
 *
 * @date 03/2024
 */

#include <cmath>

#include "hal/flash.hpp"
#include "proxy/storage.hpp"

namespace proxy {
Storage::Storage(Config& config) : start_page(config.start_page) {
    // [total_size][num_vars][{[name_len][name][buffer_address][size]}

    uint64_t sizes;
    hal::Flash::read(this->start_page, &sizes);

    uint32_t total_size = sizes >> 32;
    uint32_t num_vars = sizes & 0xFFFFFFFF;

    this->buffer.reserve(total_size);
    hal::Flash::read(this->start_page, 1, reinterpret_cast<uint64_t*>(this->buffer.data()),
                     std::ceil(total_size / 8.0f));

    uint16_t i = 0;

    for (uint16_t decoded_vars = 0; decoded_vars < num_vars; decoded_vars++) {
        uint8_t var_name_len = buffer.at(i);
        std::string var_name(buffer.begin() + i + 1, buffer.begin() + i + 1 + var_name_len);
        i += var_name_len + 1;

        // read next 2 bytes as buffer address
        this->variables[var_name].buffer_address = buffer.at(i) << 8 | buffer.at(i + 1);
        i += 2;

        // read next 2 bytes as size
        this->variables.at(var_name).size = buffer.at(i) << 8 | buffer.at(i + 1);
        i += 2;
    }
}

template <typename T>
void Storage::create(const std::string& name, T& data) {
    this->variables[name].ram_address = &data;
    this->variables.at(name).size = sizeof(T);
}

void Storage::create(const std::string& name, ISerializable& data) {
    this->variables[name].ram_address = &data;
}

template <typename T>
void Storage::sync(const std::string& name, T& data) {
    if (variables.contains(name)) {
        data = *reinterpret_cast<T*>(&this->buffer.at(variables.at(name).buffer_address));
    }

    this->variables[name].ram_address = &data;
    this->variables.at(name).size = sizeof(T);
}

void Storage::sync(const std::string& name, ISerializable& data) {
    if (variables.contains(name)) {
        data.deserialize(&this->buffer.at(variables.at(name).buffer_address), variables.at(name).size);
    }

    this->variables[name].ram_address = &data;
}

void Storage::save() {
    this->buffer.clear();

    for (auto [name, variable]: this->variables) {
        ISerializable* serializable = static_cast<ISerializable*>(variable.ram_address);

        if (serializable == nullptr) {
            uint8_t* aux = reinterpret_cast<uint8_t*>(variable.ram_address);
            variable.buffer_address = buffer.size();
            this->buffer.insert(buffer.end(), aux, aux + variable.size);
        } else {
            std::vector<uint8_t> aux = serializable->serialize();
            variable.buffer_address = buffer.size();
            this->buffer.insert(buffer.end(), aux.begin(), aux.end());
            variable.size = aux.size();
        }
    }

    this->serialize_var_map();
    hal::Flash::write(this->start_page, 0, reinterpret_cast<uint64_t*>(buffer.data()), std::ceil(buffer.size() / 8.0f));
}

void Storage::serialize_var_map() {
    // [total_size][num_vars][{[name_len][name][buffer_address][size]}

    for (auto [name, variable]: this->variables) {
        this->buffer.insert(this->buffer.begin(), variable.size);
        this->buffer.insert(this->buffer.begin(), variable.size >> 8);

        this->buffer.insert(this->buffer.begin(), variable.buffer_address);
        this->buffer.insert(this->buffer.begin(), variable.buffer_address >> 8);

        this->buffer.insert(this->buffer.begin(), name.begin(), name.end());

        this->buffer.insert(this->buffer.begin(), name.size());
    }

    uint32_t total_size = this->buffer.size();

    this->buffer.insert(this->buffer.begin(), this->variables.size());
    this->buffer.insert(this->buffer.begin(), this->variables.size() >> 8);
    this->buffer.insert(this->buffer.begin(), this->variables.size() >> 16);
    this->buffer.insert(this->buffer.begin(), this->variables.size() >> 24);

    this->buffer.insert(this->buffer.begin(), total_size);
    this->buffer.insert(this->buffer.begin(), total_size >> 8);
    this->buffer.insert(this->buffer.begin(), total_size >> 16);
    this->buffer.insert(this->buffer.begin(), total_size >> 24);
}
}  // namespace proxy
