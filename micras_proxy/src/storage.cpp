/**
 * @file storage.cpp
 *
 * @brief Proxy Storage class source
 *
 * @date 03/2024
 */

#include "micras/hal/flash.hpp"
#include "micras/proxy/storage.hpp"

namespace micras::proxy {
Storage::Storage(const Config& config) : start_page{config.start_page}, number_of_pages{config.number_of_pages} {
    uint64_t header{};
    hal::Flash::read(this->start_page, 0, &header);

    if (header >> 48 != 0xABAB) {
        return;
    }

    uint16_t total_size = header >> 32;
    uint16_t num_primitives = header >> 16;
    uint16_t num_serializables = header;

    this->buffer.resize(8L * total_size);
    hal::Flash::read(this->start_page, 1, reinterpret_cast<uint64_t*>(this->buffer.data()), total_size);

    this->primitives = deserialize_var_map<PrimitiveVariable>(this->buffer, num_primitives);
    this->serializables = deserialize_var_map<SerializableVariable>(this->buffer, num_serializables);
}

template <Fundamental T>
void Storage::create(const std::string& name, const T& data) {
    this->primitives[name].ram_pointer = &data;
    this->primitives.at(name).size = sizeof(T);
}

void Storage::create(const std::string& name, const ISerializable& data) {
    this->serializables[name].ram_pointer = &data;
}

template <Fundamental T>
void Storage::sync(const std::string& name, T& data) {
    if (this->primitives.contains(name) and this->primitives.at(name).ram_pointer == nullptr) {
        data = reinterpret_cast<T&>(this->buffer.at(this->primitives.at(name).buffer_address));
    }

    this->create<T>(name, data);
}

void Storage::sync(const std::string& name, ISerializable& data) {
    if (this->serializables.contains(name) and this->serializables.at(name).ram_pointer == nullptr) {
        const auto& serializable = this->serializables.at(name);
        data.deserialize(&this->buffer.at(serializable.buffer_address), serializable.size);
    }

    this->create(name, data);
}

void Storage::save() {
    this->buffer.clear();
    hal::Flash::erase_pages(this->start_page, this->number_of_pages);

    for (auto& [name, variable] : this->primitives) {
        if (variable.ram_pointer == nullptr) {
            this->primitives.erase(name);
            continue;
        }

        const auto* aux = reinterpret_cast<const uint8_t*>(variable.ram_pointer);
        variable.buffer_address = buffer.size();
        this->buffer.insert(this->buffer.end(), aux, aux + variable.size);
    }

    for (auto& [name, variable] : this->serializables) {
        if (variable.ram_pointer == nullptr) {
            this->serializables.erase(name);
            continue;
        }

        std::vector<uint8_t> aux = variable.ram_pointer->serialize();
        variable.buffer_address = this->buffer.size();
        variable.size = aux.size();
        this->buffer.insert(this->buffer.end(), aux.begin(), aux.end());
    }

    auto serialized_serializables = serialize_var_map<SerializableVariable>(this->serializables);
    this->buffer.insert(this->buffer.begin(), serialized_serializables.begin(), serialized_serializables.end());

    auto serialized_primitives = serialize_var_map<PrimitiveVariable>(this->primitives);
    this->buffer.insert(this->buffer.begin(), serialized_primitives.begin(), serialized_primitives.end());

    this->buffer.insert(this->buffer.end(), (8 - (this->buffer.size() % 8)) % 8, 0);
    uint16_t total_size = this->buffer.size() / 8;

    this->buffer.emplace_back(this->serializables.size());
    this->buffer.emplace_back(this->serializables.size() >> 8);
    this->buffer.emplace_back(this->primitives.size());
    this->buffer.emplace_back(this->primitives.size() >> 8);

    this->buffer.emplace_back(total_size);
    this->buffer.emplace_back(total_size >> 8);
    this->buffer.emplace_back(start_symbol);
    this->buffer.emplace_back(start_symbol >> 8);

    hal::Flash::write(this->start_page, 0, reinterpret_cast<uint64_t*>(buffer.data()), buffer.size() / 8);
}

template <typename T>
std::vector<uint8_t> Storage::serialize_var_map(const std::unordered_map<std::string, T>& variables) {
    std::vector<uint8_t> buffer;

    for (auto [name, variable] : variables) {
        buffer.emplace_back(name.size());
        buffer.insert(buffer.end(), name.begin(), name.end());

        buffer.emplace_back(variable.buffer_address);
        buffer.emplace_back(variable.buffer_address >> 8);

        buffer.emplace_back(variable.size);
        buffer.emplace_back(variable.size >> 8);
    }

    return buffer;
}

template <typename T>
std::unordered_map<std::string, T> Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars) {
    std::unordered_map<std::string, T> variables;

    uint16_t current_addr = 0;

    for (uint16_t decoded_vars = 0; decoded_vars < num_vars; decoded_vars++) {
        uint8_t var_name_len = buffer.at(current_addr);

        std::string var_name(buffer.begin() + current_addr + 1, buffer.begin() + current_addr + 1 + var_name_len);
        current_addr += var_name_len + 1;

        variables[var_name].buffer_address = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;

        variables.at(var_name).size = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;
    }

    buffer.erase(buffer.begin(), buffer.begin() + current_addr);
    return variables;
}
}  // namespace micras::proxy
