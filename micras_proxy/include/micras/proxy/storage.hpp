/**
 * @file
 */

#ifndef MICRAS_PROXY_STORAGE_HPP
#define MICRAS_PROXY_STORAGE_HPP

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "micras/core/variable_pool.hpp"
#include "micras/hal/flash.hpp"

namespace micras::proxy {
/**
 * @brief Class for storing the persistent variables of a pool in the flash memory.
 *
 * @note Registration and loading are two separate phases. Everything registers into the pool
 * first, and a single call to restore then fills whatever the image happens to carry, so nothing
 * has to remember which variables have already been claimed.
 */
class Storage {
public:
    /**
     * @brief Configuration struct for the storage.
     */
    struct Config {
        uint16_t start_sector;
        uint16_t number_of_sectors;
    };

    /**
     * @brief Construct a new Storage object, reading and validating the image in the flash memory.
     *
     * @param config Configuration for the storage.
     */
    explicit Storage(const Config& config);

    /**
     * @brief Check if a valid image was found in the flash memory.
     *
     * @return True if an image was found and parsed, false if the storage started empty.
     */
    bool is_valid() const;

    /**
     * @brief Load every variable of the pool that the image carries.
     *
     * @note Entries whose name is not registered are ignored, and so are entries whose type no
     * longer matches the one registered under that name, which is what stops a float saved by one
     * firmware from being read back as an integer by the next.
     *
     * @param pool Pool to load the variables into.
     * @return Number of variables that were loaded.
     */
    std::size_t restore(core::VariablePool& pool);

    /**
     * @brief Write every persistent variable of the pool to the flash memory.
     *
     * @note This operation blocks the processor for a few seconds, so it must only be called with
     * the robot stopped.
     *
     * @note The header is written last and occupies a flash word of its own, so an interrupted
     * save leaves it erased and the image is rejected on the next boot. That is what makes a torn
     * write safe without a checksum, which the error correction of the flash memory would make
     * redundant anyway.
     *
     * @param pool Pool to take the variables from.
     * @return True if the data was successfully written to the flash, false otherwise.
     */
    bool save(const core::VariablePool& pool);

private:
    /**
     * @brief One variable as it is described in the flash image.
     */
    struct Entry {
        std::string_view name;
        core::TypeCode   type;
        uint16_t         offset;
        uint16_t         size;
    };

    /**
     * @brief Read and validate the image currently in the flash memory.
     */
    void load();

    /**
     * @brief Parse the entry at the front of a table and advance past it.
     *
     * @param table Remaining entry table, advanced past the entry that was read.
     * @param entry Entry to fill.
     * @return True if an entry could be read and its data fits in the value area, false otherwise.
     */
    bool take_entry(std::span<const uint8_t>& table, Entry& entry) const;

    /**
     * @brief Start symbol, to tell a written image apart from erased flash.
     */
    static constexpr uint16_t start_symbol{0xABAB};

    /**
     * @brief Version of the layout described by this class, so that an image written by a firmware
     * with a different layout is ignored instead of being misread.
     */
    static constexpr uint8_t format_version{1};

    /**
     * @brief Number of bytes reserved for the header, which is a whole flash word so that writing
     * it cannot disturb the rest of the image.
     */
    static constexpr uint32_t header_size{hal::FlashWord::size};

    /**
     * @brief Number of bytes of an entry, not counting its name.
     */
    static constexpr uint32_t entry_overhead{6};

    /**
     * @brief Description of every variable in the image, as a view over the flash memory.
     */
    std::span<const uint8_t> entries;

    /**
     * @brief Values of every variable in the image, as a view over the flash memory.
     */
    std::span<const uint8_t> values;

    /**
     * @brief Number of entries in the image.
     */
    uint8_t entry_count{};

    /**
     * @brief Start sector of the storage in the flash memory.
     */
    uint16_t start_sector;

    /**
     * @brief Maximum number of sectors used by the storage in the flash memory.
     */
    uint16_t number_of_sectors;

    /**
     * @brief Whether a valid image was found in the flash memory.
     */
    bool valid{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_STORAGE_HPP
