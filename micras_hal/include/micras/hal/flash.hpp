/**
 * @file
 */

#ifndef MICRAS_HAL_FLASH_HPP
#define MICRAS_HAL_FLASH_HPP

#include <array>
#include <cstdint>
#include <span>

#include <main.h>

namespace micras::hal {
/**
 * @brief Class to handle the smallest unit of data that can be programmed into the flash memory.
 *
 * @note The buffer of the caller is used directly whenever it already covers a whole flash word, so only a
 * trailing incomplete flash word is copied to the internal buffer and padded.
 */
class FlashWord {
public:
/**
 * @brief Number of 32 bit words inside a flash word.
 */
#ifdef FLASH_NB_32BITWORD_IN_FLASHWORD
    static constexpr uint32_t words{FLASH_NB_32BITWORD_IN_FLASHWORD};
#else
    static constexpr uint32_t words{sizeof(uint64_t) / sizeof(uint32_t)};
#endif

    /**
     * @brief Number of bytes inside a flash word.
     */
    static constexpr uint32_t size{words * sizeof(uint32_t)};

    /**
     * @brief Value of every byte of an erased flash memory.
     */
    static constexpr uint8_t erased_value{0xFF};

    /**
     * @brief Construct a new Flash Word object from the beginning of a buffer.
     *
     * @param data Buffer to take the data from, padded with the erased value if smaller than a flash word.
     */
    explicit FlashWord(std::span<const uint8_t> data);

    /**
     * @brief Get the data ready to be programmed into the flash memory.
     *
     * @return Pointer to size bytes of data aligned to 32 bits.
     */
    const uint32_t* data() const;

    /**
     * @brief Check if the data had to be copied to the internal buffer.
     *
     * @return True if the data was padded or realigned, false otherwise.
     */
    bool is_padded() const;

private:
    /**
     * @brief Buffer used only when the data needs to be padded or realigned.
     */
    std::array<uint32_t, words> buffer{};

    /**
     * @brief Data to be programmed, pointing either to the buffer of the caller or to the internal one.
     */
    const uint32_t* source{};
};

/**
 * @brief Class to handle flash memory on STM32 microcontrollers.
 *
 * @note Only the region of the flash memory reserved for data storage is accessible, and every address is
 * relative to the beginning of that region.
 */
class Flash {
public:
    /**
     * @brief Enum for the status of a flash memory operation.
     */
    enum class Status : uint8_t {
        OK = 0,
        MISALIGNED = 1,
        OUT_OF_BOUNDS = 2,
        ERROR = 3
    };

    /**
     * @brief Deleted constructor for static class.
     */
    Flash() = delete;

    /**
     * @brief Number of bytes of an erasable sector.
     */
    static constexpr uint32_t sector_size{FLASH_SECTOR_SIZE};

    /**
     * @brief Number of sectors reserved for data storage.
     */
    static constexpr uint16_t total_sectors{FLASH_SECTOR_TOTAL / 2};

    /**
     * @brief Number of bytes reserved for data storage.
     */
    static constexpr uint32_t total_size{total_sectors * sector_size};

    /**
     * @brief Read data from the flash memory at an absolute address.
     *
     * @param address Address to read from in bytes, relative to the beginning of the reserved region.
     * @param size Number of bytes to read.
     * @return View over the flash memory, empty if the data is outside of the reserved region.
     */
    static std::span<const uint8_t> read(uint32_t address, uint32_t size);

    /**
     * @brief Read data from the flash memory at an address relative to a sector.
     *
     * @param sector Sector to read from, counting from the first reserved sector.
     * @param sector_address Address inside the sector to read from in bytes.
     * @param size Number of bytes to read.
     * @return View over the flash memory, empty if the data is outside of the reserved region.
     */
    static std::span<const uint8_t> read(uint16_t sector, uint32_t sector_address, uint32_t size);

    /**
     * @brief Write data to the flash memory at an absolute address.
     *
     * @note The data is padded with the erased value up to the next flash word boundary, so the number of
     * bytes actually written is the size of the data rounded up to a multiple of FlashWord::size.
     *
     * @param address Address to write to in bytes, must be a multiple of FlashWord::size.
     * @param data Data to write.
     * @return Status of the operation.
     */
    static Status write(uint32_t address, std::span<const uint8_t> data);

    /**
     * @brief Write data to the flash memory at an address relative to a sector.
     *
     * @note The data is padded with the erased value up to the next flash word boundary, so the number of
     * bytes actually written is the size of the data rounded up to a multiple of FlashWord::size.
     *
     * @param sector Sector to write to, counting from the first reserved sector.
     * @param sector_address Address inside the sector to write to in bytes, must be a multiple of
     * FlashWord::size.
     * @param data Data to write.
     * @return Status of the operation.
     */
    static Status write(uint16_t sector, uint32_t sector_address, std::span<const uint8_t> data);

    /**
     * @brief Erase sectors of the flash memory.
     *
     * @note This operation blocks the processor for around 2 s per sector, up to 4 s in the worst case, since
     * the flash memory cannot be read while it is being erased.
     *
     * @param start_sector First sector to erase, counting from the first reserved sector.
     * @param number_of_sectors Number of sectors to erase.
     * @return Status of the operation.
     */
    static Status erase_sectors(uint16_t start_sector, uint16_t number_of_sectors = 1);
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_FLASH_HPP
