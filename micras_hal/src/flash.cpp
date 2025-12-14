/**
 * @file
 */

#include <bit>
#include <stm32h7xx_hal_flash.h>

#include "micras/hal/flash.hpp"

namespace micras::hal {
/**
 * @brief Last address of the flash memory.
 *
 * @note Needs to be here because it is not defined at compile time.
 */
static const uint32_t base_address = FLASH_BASE + FLASH_SIZE - 8;

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::read(uint32_t address, uint32_t data[], uint32_t size) {
    const uint32_t end = base_address - 8 * address;

    for (address = end - 8 * (size - 1); address <= end; address += 8, data++) {
        (*data) = *(std::bit_cast<uint32_t*>(address));
    }
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::read(uint16_t page, uint16_t page_address, uint32_t data[], uint32_t size) {
    read(page * flash_words_per_sector + page_address, data, size);
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::write(uint32_t address, const uint32_t data[], uint32_t size) {
    HAL_FLASH_Unlock();

    const uint32_t end = base_address - 8 * address;
    address = end - 8 * (size - 1 + (bytes_per_flashword - size % bytes_per_flashword));

    while (size > 0) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, std::bit_cast<uint32_t>(data));
        size -= bytes_per_flashword;
        address += bytes_per_flashword;
        data += bytes_per_flashword;
    }

    HAL_FLASH_Lock();
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::write(uint16_t page, uint16_t page_address, const uint32_t data[], uint32_t size) {
    write(page * flash_words_per_sector + page_address, data, size);
}

void Flash::erase_pages(uint16_t page, uint16_t number_of_pages) {
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_1,
        .Sector = FLASH_SECTOR_TOTAL - page - number_of_pages,
        .NbSectors = number_of_pages,
        .VoltageRange = FLASH_VOLTAGE_RANGE_4
    };

    uint32_t sector_error{};

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_struct, &sector_error);
    HAL_FLASH_Lock();
}
}  // namespace micras::hal
