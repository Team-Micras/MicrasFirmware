/**
 * @file
 */

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include "micras/core/serializable.hpp"
#include "micras/core/variable_pool.hpp"
#include "micras/hal/flash.hpp"
#include "micras/proxy/storage.hpp"

namespace micras::proxy {
static uint16_t read_uint16(std::span<const uint8_t> buffer, uint32_t address) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
    return static_cast<uint16_t>(buffer[address] | buffer[address + 1U] << 8);
}

static void append_uint16(std::vector<uint8_t>& buffer, uint16_t value) {
    buffer.emplace_back(value);
    buffer.emplace_back(value >> 8);
}

static constexpr uint32_t align_size(std::size_t size) {
    return (size + hal::FlashWord::size - 1) / hal::FlashWord::size * hal::FlashWord::size;
}

Storage::Storage(const Config& config) :
    start_sector{config.start_sector}, number_of_sectors{config.number_of_sectors} {
    this->load();
}

void Storage::load() {
    this->entries = {};
    this->values = {};
    this->entry_count = 0;
    this->valid = false;

    const std::span<const uint8_t> header = hal::Flash::read(this->start_sector, 0, header_size);

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
    if (header.size() < header_size or read_uint16(header, 0) != start_symbol or header[2] != format_version) {
        return;
    }

    this->entry_count = header[3];
    // NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)

    const uint32_t                 body_size = read_uint16(header, 4) * hal::FlashWord::size;
    const std::span<const uint8_t> body = hal::Flash::read(this->start_sector, header_size, body_size);

    if (body.size() < body_size) {
        return;
    }

    std::span<const uint8_t> table = body;

    for (uint8_t index = 0; index < this->entry_count; index++) {
        if (table.empty() or table.size() < table.front() + entry_overhead) {
            return;
        }

        table = table.subspan(table.front() + entry_overhead);
    }

    this->entries = body.first(body.size() - table.size());
    this->values = table;
    this->valid = true;
}

bool Storage::is_valid() const {
    return this->valid;
}

bool Storage::take_entry(std::span<const uint8_t>& table, Entry& entry) const {
    const uint8_t name_size = table.front();

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    entry.name = {std::bit_cast<const char*>(table.data() + 1), name_size};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
    entry.type = static_cast<core::TypeCode>(table[name_size + 1U]);
    entry.offset = read_uint16(table, name_size + 2U);
    entry.size = read_uint16(table, name_size + 4U);

    table = table.subspan(name_size + entry_overhead);

    return entry.offset + static_cast<std::size_t>(entry.size) <= this->values.size();
}

std::size_t Storage::restore(core::VariablePool& pool) {
    if (not this->valid) {
        return 0;
    }

    std::span<const uint8_t> table = this->entries;
    std::size_t              restored = 0;

    for (uint8_t index = 0; index < this->entry_count; index++) {
        Entry entry{};

        if (not this->take_entry(table, entry)) {
            continue;
        }

        const auto id = pool.find(entry.name);

        if (not id.has_value()) {
            continue;
        }

        const core::Variable& variable = pool.at(id.value());

        if (not variable.access.persist or variable.type != entry.type) {
            continue;
        }

        const std::span<const uint8_t> data = this->values.subspan(entry.offset, entry.size);

        if (variable.type == core::TypeCode::BLOB) {
            static_cast<core::ISerializable*>(variable.address)->deserialize(data.data(), data.size());
        } else if (variable.size != entry.size) {
            continue;
        } else {
            std::memcpy(variable.address, data.data(), data.size());
        }

        restored++;
    }

    return restored;
}

bool Storage::save(const core::VariablePool& pool) {
    std::vector<uint8_t> table;
    std::vector<uint8_t> payload;
    uint8_t              count = 0;

    for (const core::Variable& variable : pool.all()) {
        if (not variable.access.persist) {
            continue;
        }

        const std::size_t name_size = variable.prefix.size() + variable.name.size();
        const std::size_t offset = payload.size();

        if (variable.type == core::TypeCode::BLOB) {
            const std::vector<uint8_t> data = static_cast<const core::ISerializable*>(variable.address)->serialize();
            payload.insert(payload.end(), data.begin(), data.end());
        } else {
            const auto* bytes = std::bit_cast<const uint8_t*>(variable.address);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            payload.insert(payload.end(), bytes, bytes + variable.size);
        }

        const std::size_t size = payload.size() - offset;

        if (count == UINT8_MAX or name_size > UINT8_MAX or offset > UINT16_MAX or size > UINT16_MAX) {
            return false;
        }

        table.emplace_back(name_size);
        table.insert(table.end(), variable.prefix.begin(), variable.prefix.end());
        table.insert(table.end(), variable.name.begin(), variable.name.end());
        table.emplace_back(std::to_underlying(variable.type));
        append_uint16(table, offset);
        append_uint16(table, size);

        count++;
    }

    std::vector<uint8_t> body = std::move(table);
    body.insert(body.end(), payload.begin(), payload.end());
    body.resize(align_size(body.size()), hal::FlashWord::erased_value);

    if (body.size() / hal::FlashWord::size > UINT16_MAX or
        header_size + body.size() > this->number_of_sectors * hal::Flash::sector_size) {
        return false;
    }

    if (hal::Flash::erase_sectors(this->start_sector, this->number_of_sectors) != hal::Flash::Status::OK) {
        return false;
    }

    if (hal::Flash::write(this->start_sector, header_size, body) != hal::Flash::Status::OK) {
        return false;
    }

    std::vector<uint8_t> header;
    header.reserve(header_size);
    append_uint16(header, start_symbol);
    header.emplace_back(format_version);
    header.emplace_back(count);
    append_uint16(header, body.size() / hal::FlashWord::size);
    header.resize(header_size, hal::FlashWord::erased_value);

    if (hal::Flash::write(this->start_sector, 0, header) != hal::Flash::Status::OK) {
        return false;
    }

    this->load();
    return this->valid;
}
}  // namespace micras::proxy
