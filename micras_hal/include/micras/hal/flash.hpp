/**
 * @file
 */

#ifndef MICRAS_HAL_FLASH_HPP
#define MICRAS_HAL_FLASH_HPP

#include <cstdint>
#include <main.h>

namespace micras::hal {
/**
 * @brief Class to handle flash memory on STM32 microcontrollers.
 */
class Flash {
public:
    /**
     * @brief Deleted constructor for static class.
     */
    Flash() = delete;

    // NOLINTBEGIN(*-avoid-c-arrays)

    /**
     * @brief Read data from flash memory at an absolute address.
     *
     * @param address Address to read from (indexed by double words).
     * @param data Pointer to store the data read.
     * @param size Size in double words of the data to read.
     */
    static void read(uint32_t address, uint64_t data[], uint32_t size = 1);

    /**
     * @brief Read data from flash memory at an address relative to a page.
     *
     * @param page Page to read from (counting from the last to the first).
     * @param page_address Address inside the page to read from (indexed by double words).
     * @param data Pointer to store the data read.
     * @param size Size in double words of the data to read.
     */
    static void read(uint16_t page, uint16_t page_address, uint64_t data[], uint32_t size = 1);

    /**
     * @brief Write data to flash memory at an absolute address.
     *
     * @param address Address to write to (indexed by double words).
     * @param data Pointer to the data to write.
     * @param size Size in double words of the data to write.
     */
    static void write(uint32_t address, const uint64_t data[], uint32_t size = 1);

    /**
     * @brief Write data to flash memory at an address relative to a page.
     *
     * @param page Page to write to (counting from the last to the first).
     * @param page_address Address inside the page to write to (indexed by double words).
     * @param data Pointer to the data to write.
     * @param size Size in double words of the data to write.
     */
    static void write(uint16_t page, uint16_t page_address, const uint64_t data[], uint32_t size = 1);

    // NOLINTEND(*-avoid-c-arrays)

    /**
     * @brief Erase flash memory pages.
     *
     * @param page First page to erase (counting from the last to the first).
     * @param number_of_pages Number of pages to erase.
     */
    static void erase_pages(uint16_t page, uint16_t number_of_pages = 1);

private:
    /**
     * @brief Number of double words per row.
     */
    static constexpr uint32_t double_words_per_row{32};

    /**
     * @brief Number of bytes per row.
     */
    static constexpr uint32_t bytes_per_row{8 * double_words_per_row};

    /**
     * @brief Number of double words per page.
     */
    static constexpr uint32_t double_words_per_page{FLASH_PAGE_SIZE / 8};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_FLASH_HPP
