/**
 * @file
 */

#ifndef MICRAS_HAL_SPI_HPP
#define MICRAS_HAL_SPI_HPP

#include <array>
#include <cstdint>
#include <span>

#include <main.h>
#include "micras/hal/gpio.hpp"

namespace micras::hal {
/**
 * @brief Class to handle SPI peripheral on STM32 microcontrollers.
 */
class Spi {
public:
    /**
     * @brief SPI configuration struct.
     *
     * @note The clock polarity and phase belong to the device, not to the bus: several devices with
     * different SPI modes can share one peripheral, and select_device reconfigures it when the mode
     * of the device being selected differs from the one currently programmed.
     */
    struct Config {
        void (*init_function)();
        SPI_HandleTypeDef* handle;
        hal::Gpio::Config  cs_gpio;
        uint32_t           timeout;
        uint32_t           clock_polarity;
        uint32_t           clock_phase;
    };

    /**
     * @brief State of the transfer started by start_transfer.
     *
     * @note A transfer that could not be started and one that ended in a bus error both read as
     * failed, and the received data is valid only when complete.
     */
    enum class Transfer : uint8_t {
        NONE = 0,
        RUNNING = 1,
        COMPLETE = 2,
        FAILED = 3,
    };

    /**
     * @brief Construct a new Spi object.
     *
     * @param config Configuration for the SPI.
     */
    explicit Spi(const Config& config);

    /**
     * @brief Special member functions, of which only the destructor exists.
     *
     * @note The interrupt that ends a transfer finds its device by address, so an object cannot be
     * copied or moved.
     */
    ///@{
    ~Spi();
    Spi(const Spi&) = delete;
    Spi(Spi&&) = delete;
    Spi& operator=(const Spi&) = delete;
    Spi& operator=(Spi&&) = delete;
    ///@}

    /**
     * @brief Activate the chip select, reconfiguring the bus for this device if needed.
     *
     * @note Blocks for up to the configured timeout waiting for the peripheral to become ready, so
     * that an SPI stuck in an error state is reported instead of hanging the caller.
     *
     * @return True if the device was successfully selected, false otherwise.
     */
    bool select_device();

    /**
     * @brief Deactivate the chip select.
     */
    void unselect_device();

    /**
     * @brief Transmit data over SPI.
     *
     * @param data Data to transmit.
     * @return True if the transfer completed, false otherwise.
     */
    bool transmit(std::span<const uint8_t> data);

    /**
     * @brief Receive data over SPI.
     *
     * @param data Buffer to receive data into.
     * @return True if the transfer completed, false otherwise.
     */
    bool receive(std::span<uint8_t> data);

    /**
     * @brief Transmit and receive data over SPI in the same transfer.
     *
     * @param transmitted Data to transmit.
     * @param received Buffer to receive data into, at least as large as the transmitted one.
     * @return True if the transfer completed, false otherwise.
     */
    bool transmit_receive(std::span<const uint8_t> transmitted, std::span<uint8_t> received);

    /**
     * @brief Start a transfer in both directions that the DMA carries out, without waiting for it.
     *
     * @details The device is selected here and unselected by the interrupt that ends the transfer,
     * so the bus is free for the other devices on it as soon as the last bit is out, and the caller
     * has nothing to do but look at the state of the transfer later.
     *
     * @note Both buffers are borrowed until the transfer ends, and have to be in memory that the DMA
     * reaches, which on the parts with tightly coupled memory excludes it.
     *
     * @param transmitted Data to transmit.
     * @param received Buffer to receive data into, at least as large as the transmitted one.
     * @return True if the transfer was started, false otherwise.
     */
    bool start_transfer(std::span<const uint8_t> transmitted, std::span<uint8_t> received);

    /**
     * @brief Get the state of the last transfer started by start_transfer.
     *
     * @return State of the transfer.
     */
    Transfer get_transfer() const;

    /**
     * @brief Check if the SPI was successfully initialized.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

    /**
     * @brief Unselect the device whose transfer ended and record how it ended.
     *
     * @note To be called by the transfer callbacks only.
     *
     * @param handle Handle of the bus whose transfer ended.
     * @param succeeded Whether the transfer ended without a bus error.
     */
    static void on_transfer_end(const SPI_HandleTypeDef* handle, bool succeeded);

private:
    /**
     * @brief Largest number of transfers that can be running at once, which is one per bus.
     */
    static constexpr uint8_t max_transfers{6};

    /**
     * @brief Devices with a transfer running, for the interrupt to find the one that ended.
     */
    static std::array<Spi*, max_transfers> transferring;

    /**
     * @brief Handle for the SPI.
     */
    SPI_HandleTypeDef* handle;

    /**
     * @brief GPIO for the chip select pin.
     */
    hal::Gpio cs_gpio;

    /**
     * @brief Timeout for the SPI operations in ms.
     */
    uint32_t timeout;

    /**
     * @brief Clock polarity required by the device behind this chip select.
     */
    uint32_t clock_polarity;

    /**
     * @brief Clock phase required by the device behind this chip select.
     */
    uint32_t clock_phase;

    /**
     * @brief State of the last transfer started by start_transfer.
     */
    volatile Transfer transfer{Transfer::NONE};

    /**
     * @brief Flag to check if the SPI was initialized.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_SPI_HPP
