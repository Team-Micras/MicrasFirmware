#ifndef MICRAS_HAL_FLASH_HPP
#define MICRAS_HAL_FLASH_HPP
#include <array>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace micras::hal {
class FlashWord {
public:
    static constexpr uint32_t words{8};
    static constexpr uint32_t size{words * sizeof(uint32_t)};
    static constexpr uint8_t  erased_value{0xFF};
};

class Flash {
public:
    enum class Status : uint8_t {
        OK = 0,
        MISALIGNED = 1,
        OUT_OF_BOUNDS = 2,
        ERROR = 3
    };
    static constexpr uint32_t sector_size{128 * 1024};
    static constexpr uint16_t total_sectors{2};
    static constexpr uint32_t total_size{total_sectors * sector_size};

    static inline std::vector<uint8_t> memory = std::vector<uint8_t>(total_size, 0xFF);
    static inline uint32_t             write_budget = UINT32_MAX;  // flash words allowed before a simulated power loss

    static std::span<const uint8_t> read(uint16_t sector, uint32_t sector_address, uint32_t size) {
        uint64_t address = uint64_t(sector) * sector_size + sector_address;
        if (address + size > total_size)
            return {};
        return std::span<const uint8_t>{memory.data() + address, size};
    }

    static Status write(uint16_t sector, uint32_t sector_address, std::span<const uint8_t> data) {
        if (sector_address % FlashWord::size != 0)
            return Status::MISALIGNED;
        uint64_t address = uint64_t(sector) * sector_size + sector_address;
        uint32_t padded = (data.size() + FlashWord::size - 1) / FlashWord::size * FlashWord::size;
        if (address + padded > total_size)
            return Status::OUT_OF_BOUNDS;
        for (uint32_t i = 0; i < padded; i += FlashWord::size) {
            if (write_budget == 0)
                return Status::ERROR;
            write_budget--;
            for (uint32_t j = 0; j < FlashWord::size; j++) {
                uint32_t k = i + j;
                memory[address + k] = k < data.size() ? data[k] : FlashWord::erased_value;
            }
        }
        return Status::OK;
    }

    static Status erase_sectors(uint16_t start_sector, uint16_t number_of_sectors = 1) {
        uint64_t address = uint64_t(start_sector) * sector_size;
        if (address + uint64_t(number_of_sectors) * sector_size > total_size)
            return Status::OUT_OF_BOUNDS;
        std::memset(memory.data() + address, 0xFF, size_t(number_of_sectors) * sector_size);
        return Status::OK;
    }
};
}  // namespace micras::hal
#endif
