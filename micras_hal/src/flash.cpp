/**
 * @file
 */

#include <bit>

#include "micras/hal/flash.hpp"

namespace micras::hal {
/**
 * @brief Last address of the flash memory.
 *
 * @note Needs to be here because it is not defined at compile time.
 */
static const uint32_t base_address = FLASH_BASE + FLASH_SIZE - 8;

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::read(uint32_t address, uint64_t data[], uint32_t size) {
    uint32_t end = base_address - 8 * address;

    for (address = end - 8 * (size - 1); address <= end; address += 8, data++) {
        (*data) = *(std::bit_cast<uint64_t*>(address));
    }
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::read(uint16_t page, uint16_t page_address, uint64_t data[], uint32_t size) {
    read(page * double_words_per_page + page_address, data, size);
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::write(uint32_t address, const uint64_t data[], uint32_t size) {
    HAL_FLASH_Unlock();

    uint32_t end = base_address - 8 * address;
    address = end - 8 * (size - 1);

    while (address <= end) {
        if (size >= double_words_per_row) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, address, std::bit_cast<uint32_t>(data));
            address += bytes_per_row;
            data += double_words_per_row;
            size -= double_words_per_row;
        } else {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *data);
            address += 8;
            data++;
            size--;
        }
    }

    HAL_FLASH_Lock();
}

// NOLINTNEXTLINE(*-avoid-c-arrays)
void Flash::write(uint16_t page, uint16_t page_address, const uint64_t data[], uint32_t size) {
    write(page * double_words_per_page + page_address, data, size);
}

void Flash::erase_pages(uint16_t page, uint16_t number_of_pages) {
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_2,
        .Page = FLASH_PAGE_NB - page - number_of_pages,
        .NbPages = number_of_pages,
    };

    uint32_t page_error{};

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_struct, &page_error);
    HAL_FLASH_Lock();
}
}  // namespace micras::hal
