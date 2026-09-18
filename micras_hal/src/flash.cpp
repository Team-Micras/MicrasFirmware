/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <stm32h7xx_hal.h>

#include "micras/hal/flash.hpp"

namespace micras::hal {
/**
 * @brief First address of the flash memory region reserved for data storage.
 *
 * @note Needs to be here because it is not defined at compile time.
 */
static const uint32_t base_address = FLASH_BASE + FLASH_SIZE / 2;

/**
 * @brief Index of the first sector of the flash memory region reserved for data storage.
 */
static constexpr uint16_t base_sector{FLASH_SECTOR_TOTAL / 2};

/**
 * @brief Round a number of bytes up to a whole number of flash words.
 *
 * @param size Number of bytes to round up.
 * @return Number of bytes occupied in the flash memory.
 */
static constexpr uint32_t align_size(uint32_t size) {
    return (size + FlashWord::size - 1) / FlashWord::size * FlashWord::size;
}

FlashWord::FlashWord(std::span<const uint8_t> data) {
    const auto data_address = std::bit_cast<uintptr_t>(data.data());

    if (data.size() >= size and data_address % alignof(uint32_t) == 0) {
        this->source = std::bit_cast<const uint32_t*>(data.data());
        return;
    }

    const std::span<uint8_t> bytes{std::bit_cast<uint8_t*>(this->buffer.data()), size};

    std::ranges::fill(bytes, erased_value);
    std::ranges::copy(data.first(std::min<std::size_t>(data.size(), size)), bytes.begin());

    this->source = this->buffer.data();
}

const uint32_t* FlashWord::data() const {
    return this->source;
}

bool FlashWord::is_padded() const {
    return this->source == this->buffer.data();
}

std::span<const uint8_t> Flash::read(uint32_t address, uint32_t size) {
    if (address > total_size or size > total_size - address) {
        return {};
    }

    return {std::bit_cast<const uint8_t*>(base_address + address), size};
}

std::span<const uint8_t> Flash::read(uint16_t sector, uint32_t sector_address, uint32_t size) {
    if (sector >= total_sectors or sector_address > sector_size) {
        return {};
    }

    return read(sector * sector_size + sector_address, size);
}

Flash::Status Flash::write(uint32_t address, std::span<const uint8_t> data) {
    if (address % FlashWord::size != 0) {
        return MISALIGNED;
    }

    if (address > total_size or align_size(data.size()) > total_size - address) {
        return OUT_OF_BOUNDS;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return ERROR;
    }

    Status status = OK;

    for (uint32_t offset = 0; offset < data.size(); offset += FlashWord::size) {
        const FlashWord word{data.subspan(offset)};

        if (HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_FLASHWORD, base_address + address + offset, std::bit_cast<uint32_t>(word.data())
            ) != HAL_OK) {
            status = ERROR;
            break;
        }
    }

    HAL_FLASH_Lock();

    return status;
}

Flash::Status Flash::write(uint16_t sector, uint32_t sector_address, std::span<const uint8_t> data) {
    if (sector >= total_sectors or sector_address > sector_size) {
        return OUT_OF_BOUNDS;
    }

    return write(sector * sector_size + sector_address, data);
}

Flash::Status Flash::erase_sectors(uint16_t start_sector, uint16_t number_of_sectors) {
    if (start_sector >= total_sectors or number_of_sectors > total_sectors - start_sector) {
        return OUT_OF_BOUNDS;
    }

    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_1,
        .Sector = static_cast<uint32_t>(base_sector + start_sector),
        .NbSectors = number_of_sectors,
        .VoltageRange = FLASH_VOLTAGE_RANGE_4
    };

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return ERROR;
    }

    pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

    uint32_t sector_error{};

    const HAL_StatusTypeDef hal_status = HAL_FLASHEx_Erase(&erase_struct, &sector_error);

    HAL_FLASH_Lock();

    if (hal_status != HAL_OK or sector_error != 0xFFFFFFFFU) {
        return ERROR;
    }

    return OK;
}
}  // namespace micras::hal
