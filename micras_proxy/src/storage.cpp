#include <bit>

#include "micras/hal/flash.hpp"
#include "micras/proxy/storage.hpp"

namespace micras::proxy {
Storage::Storage(const Config& config) : start_page{config.start_page}, number_of_pages{config.number_of_pages} {
    uint64_t header{};
    hal::Flash::read(this->start_page, 0, &header);

    if (header >> 32 != start_symbol) {
        return;
    }

    const uint16_t total_size = header >> 16;
    const uint16_t var_map_size = header;

    this->buffer.resize(8L * total_size);
    hal::Flash::read(this->start_page, 1, std::bit_cast<uint64_t*>(this->buffer.data()), total_size);

    this->serial_variable_pool.deserialize_map(this->buffer);
    this->buffer.erase(this->buffer.begin(), this->buffer.begin() + var_map_size);
}

void Storage::serialize_variables_data() {
    this->serial_variable_pool.for_each([this](uint16_t id, core::SerialVariable& variable) {
        auto serialized_variable = variable.serialize();
        auto variable_index = this->buffer.size();

        this->buffer.insert(this->buffer.end(), serialized_variable.begin(), serialized_variable.end());
        this->buffer_views[variable.get_name()] = std::make_pair<uint8_t>(variable_index, serialized_variable.size());
    });
}

void Storage::save() {
    this->buffer.clear();
    hal::Flash::erase_pages(this->start_page, this->number_of_pages);

    auto serialized_variable = this->serialize_variables_data();
    this->buffer.insert(this->buffer.begin(), serialized_variable.begin(), serialized_variable.end());

    auto serialized_map = this->serial_variable_pool.serialize_map();
    this->buffer.insert(this->buffer.begin(), serialized_map.begin(), serialized_map.end());

    this->buffer.insert(this->buffer.end(), (8 - (this->buffer.size() % 8)) % 8, 0);
    const uint16_t total_size = this->buffer.size() / 8;

    this->buffer.emplace_back(serialized_map.size());
    this->buffer.emplace_back(serialized_map.size() >> 8);

    this->buffer.emplace_back(total_size);
    this->buffer.emplace_back(total_size >> 8);

    this->buffer.emplace_back(start_symbol);
    this->buffer.emplace_back(start_symbol >> 8);
    this->buffer.emplace_back(start_symbol >> 16);
    this->buffer.emplace_back(start_symbol >> 24);

    hal::Flash::write(this->start_page, 0, std::bit_cast<uint64_t*>(buffer.data()), buffer.size() / 8);
}
}  // namespace micras::proxy
