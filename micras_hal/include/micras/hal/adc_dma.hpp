/**
 * @file
 */

#ifndef MICRAS_HAL_ADC_DMA_HPP
#define MICRAS_HAL_ADC_DMA_HPP

#include <array>
#include <cstdint>
#include <main.h>
#include <span>

namespace micras::hal {
/**
 * @brief Class to handle ADC peripheral on STM32 microcontrollers using DMA.
 */
class AdcDma {
public:
    /**
     * @brief Configuration struct for ADC DMA.
     *
     * @note The reference voltage is a property of the board, not of the silicon, so it is
     * configured per instance next to the resolution instead of being fixed by this class.
     */
    struct Config {
        void (*init_function)();
        ADC_HandleTypeDef* handle;
        uint16_t           max_reading;
        float              reference_voltage;
    };

    /**
     * @brief Construct a new AdcDma object.
     *
     * @note The object registers itself for the interrupts to find it, and a converter that finds
     * no free place is not initialized.
     *
     * @param config ADC DMA configuration struct.
     */
    explicit AdcDma(const Config& config);

    /**
     * @brief Special member functions, of which only the destructor exists.
     *
     * @note The interrupt finds a converter by its address, so an object cannot be copied or moved.
     */
    ///@{
    ~AdcDma();
    AdcDma(const AdcDma&) = delete;
    AdcDma(AdcDma&&) = delete;
    AdcDma& operator=(const AdcDma&) = delete;
    AdcDma& operator=(AdcDma&&) = delete;
    ///@}

    /**
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA.
     *
     * @param buffer 32 bit destination buffer address.
     * @return True if the conversion was started, false otherwise.
     */
    bool start_dma(std::span<uint32_t> buffer);

    /**
     * @brief Enable ADC, start conversion of regular group and transfer result through DMA.
     *
     * @param buffer 16 bit destination buffer address.
     * @return True if the conversion was started, false otherwise.
     */
    bool start_dma(std::span<uint16_t> buffer);

    /**
     * @brief Start the conversions as start_dma does, keeping a copy of every complete sequence.
     *
     * @details The DMA rewrites the buffer continuously, so reading it while it is being written
     * can mix values of two different sequences. Every time the DMA reaches the end of the buffer,
     * the transfer complete interrupt copies it to the snapshot, which is therefore always one whole
     * sequence. The copy is a few words long, so the interrupt costs a few tens of cycles.
     *
     * @note Both buffers are borrowed and have to outlive the conversions.
     *
     * @param buffer 16 bit destination buffer of the DMA.
     * @param snapshot Buffer of the same size that receives the copies.
     * @return True if the conversion was started, false otherwise.
     */
    bool start_dma(std::span<uint16_t> buffer, std::span<uint16_t> snapshot);

    /**
     * @brief Read the last complete sequence.
     *
     * @note The snapshot may be replaced while it is being read, which is detected with the
     * sequence counter and answered by reading it again. The counter is volatile but the snapshot
     * is not, so the copy is fenced on both sides to keep the compiler from moving it past either
     * read of the counter.
     *
     * @param destination Buffer of the size of the snapshot that receives it, of which a smaller
     * one receives only what fits.
     * @return The number of sequences completed so far, which tells whether this one is new.
     */
    uint32_t read_snapshot(std::span<uint16_t> destination) const;

    /**
     * @brief Stop ADC conversion of regular group (and injected group in case of auto_injection mode).
     */
    void stop_dma();

    /**
     * @brief Restart the conversions if an error stopped them.
     *
     * @details After an overrun the converter stops requesting transfers until its overrun flag is
     * cleared, and clearing only the flag would leave one conversion missing and every following
     * value in the place of another channel. A restart begins the buffer again at its first
     * channel. The error callback only marks the converter, since stopping and starting it waits on
     * the hardware, and this restarts it outside the interrupt. A transfer error of the DMA calls
     * the error callback too, and leaves the vendor HAL in an error state that only a new
     * initialization clears and that turns every complete transfer into another error, so that
     * state is cleared before the start.
     *
     * @note To be called once per iteration by the owner of the converter. No sequence completes
     * until then, so read_snapshot reports none as new.
     */
    void recover();

    /**
     * @brief Get the maximum reading of the ADC.
     *
     * @return Maximum reading of the ADC.
     */
    uint16_t get_max_reading() const;

    /**
     * @brief Get the reference voltage of the ADC measurement.
     *
     * @return Reference voltage in volts.
     */
    float get_reference_voltage() const;

    /**
     * @brief Check if the ADC was calibrated and started successfully.
     *
     * @note A failed calibration or start leaves the DMA buffer at zero, which reads exactly like
     * a real measurement, so this is the only way to tell one from the other.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool was_initialized() const;

    /**
     * @brief Get the number of times any converter was restarted.
     *
     * @note Anything but zero means that a converter stopped, and how often.
     *
     * @return Number of restarts since power on.
     */
    static uint32_t get_restarts();

    /**
     * @brief Count a complete sequence of a converter and copy its buffer to its snapshot.
     *
     * @note To be called by the conversion complete callback only.
     *
     * @param handle Handle of the converter that completed a sequence.
     */
    static void on_sequence_complete(const ADC_HandleTypeDef* handle);

    /**
     * @brief Mark a converter that an error stopped, for recover to restart it.
     *
     * @note To be called by the error callback only.
     *
     * @param handle Handle of the converter that stopped.
     */
    static void on_error(const ADC_HandleTypeDef* handle);

private:
    /**
     * @brief Find the object of a converter.
     *
     * @param handle Handle of the converter.
     * @return Object of the converter, or nullptr if none was constructed for it.
     */
    static AdcDma* find(const ADC_HandleTypeDef* handle);

    /**
     * @brief Largest number of converters.
     */
    static constexpr uint8_t max_instances{4};

    /**
     * @brief Every converter, for the interrupts to find the one that completed or stopped.
     */
    static std::array<AdcDma*, max_instances> instances;

    /**
     * @brief Number of times any converter was restarted.
     */
    static uint32_t restarts;

    /**
     * @brief Maximum ADC reading.
     */
    uint16_t max_reading;

    /**
     * @brief Reference voltage for the ADC measurement.
     */
    float reference_voltage;

    /**
     * @brief ADC handle.
     */
    ADC_HandleTypeDef* handle;

    /**
     * @brief Destination of the DMA as the vendor HAL takes it, whose size is the number of transfers.
     */
    std::span<uint32_t> transfer;

    /**
     * @brief Destination buffer of the DMA, of a converter that keeps a snapshot.
     */
    std::span<uint16_t> buffer;

    /**
     * @brief Copy of the last complete sequence.
     */
    std::span<uint16_t> snapshot;

    /**
     * @brief Number of sequences completed so far.
     */
    volatile uint32_t sequence{};

    /**
     * @brief Flag set by the error callback when an error stopped the converter.
     */
    volatile bool stopped{};

    /**
     * @brief Flag to check if the ADC was calibrated and started.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ADC_DMA_HPP
