/**
 * @file
 */

#include <bit>

#include "micras/hal/flash.hpp"
#include "micras/proxy/storage.hpp"

namespace micras::proxy {
Storage::Storage(const Config& config) : start_page{config.start_page}, number_of_pages{config.number_of_pages} {
    uint64_t header{};
    hal::Flash::read(this->start_page, 0, &header);

    if (header >> 48 != start_symbol) {
        return;
    }

    const uint16_t total_size = header >> 32;
    const uint16_t num_primitives = header >> 16;
    const uint16_t num_serializables = header;

    this->buffer.resize(8L * total_size);
    hal::Flash::read(this->start_page, 1, std::bit_cast<uint64_t*>(this->buffer.data()), total_size);

    this->primitives = deserialize_var_map<PrimitiveVariable>(this->buffer, num_primitives);
    this->serializables = deserialize_var_map<SerializableVariable>(this->buffer, num_serializables);
}

void Storage::create(const std::string& name, const ISerializable& data) {
    this->serializables[name].ram_pointer = &data;
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

    for (auto it = this->primitives.begin(); it != this->primitives.end();) {
        auto& [name, variable] = *it;

        if (variable.ram_pointer == nullptr) {
            it = this->primitives.erase(it);
            continue;
        }

        const auto* aux = std::bit_cast<const uint8_t*>(variable.ram_pointer);
        variable.buffer_address = buffer.size();
        this->buffer.insert(this->buffer.end(), aux, aux + variable.size);
        it++;
    }

    for (auto it = this->serializables.begin(); it != this->serializables.end();) {
        auto& [name, variable] = *it;

        if (variable.ram_pointer == nullptr) {
            it = this->serializables.erase(it);
            continue;
        }

        std::vector<uint8_t> aux = variable.ram_pointer->serialize();
        variable.buffer_address = this->buffer.size();
        variable.size = aux.size();
        this->buffer.insert(this->buffer.end(), aux.begin(), aux.end());
        it++;
    }

    auto serialized_serializables = serialize_var_map<SerializableVariable>(this->serializables);
    this->buffer.insert(this->buffer.begin(), serialized_serializables.begin(), serialized_serializables.end());

    auto serialized_primitives = serialize_var_map<PrimitiveVariable>(this->primitives);
    this->buffer.insert(this->buffer.begin(), serialized_primitives.begin(), serialized_primitives.end());

    this->buffer.insert(this->buffer.end(), (8 - (this->buffer.size() % 8)) % 8, 0);
    const uint16_t total_size = this->buffer.size() / 8;

    this->buffer.emplace_back(this->serializables.size());
    this->buffer.emplace_back(this->serializables.size() >> 8);
    this->buffer.emplace_back(this->primitives.size());
    this->buffer.emplace_back(this->primitives.size() >> 8);

    this->buffer.emplace_back(total_size);
    this->buffer.emplace_back(total_size >> 8);
    this->buffer.emplace_back(start_symbol);
    this->buffer.emplace_back(start_symbol >> 8);

    hal::Flash::write(this->start_page, 0, std::bit_cast<uint64_t*>(buffer.data()), buffer.size() / 8);
}

template <typename T>
std::vector<uint8_t> Storage::serialize_var_map(const std::unordered_map<std::string, T>& variables) {
    std::vector<uint8_t> buffer;

    for (const auto& [name, variable] : variables) {
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
        const uint8_t var_name_len = buffer.at(current_addr);

        const std::string var_name(buffer.begin() + current_addr + 1, buffer.begin() + current_addr + 1 + var_name_len);
        current_addr += var_name_len + 1;

        variables[var_name].buffer_address = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;

        variables.at(var_name).size = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;
    }

    buffer.erase(buffer.begin(), buffer.begin() + current_addr);
    return variables;
}

// Explicit instantiation of template functions
template std::vector<uint8_t>
    Storage::serialize_var_map(const std::unordered_map<std::string, PrimitiveVariable>& variables);

template std::unordered_map<std::string, Storage::PrimitiveVariable>
    Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars);

template std::vector<uint8_t>
    Storage::serialize_var_map(const std::unordered_map<std::string, SerializableVariable>& variables);

template std::unordered_map<std::string, Storage::SerializableVariable>
    Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars);
}  // namespace micras::proxy
