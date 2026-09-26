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
     * @brief Copy the buffer of the converter that completed a sequence to its snapshot.
     *
     * @note To be called by the conversion complete callback only.
     *
     * @param handle Handle of the converter that completed a sequence.
     */
    static void on_sequence_complete(const ADC_HandleTypeDef* handle);

private:
    /**
     * @brief Largest number of converters that can keep a snapshot.
     */
    static constexpr uint8_t max_instances{4};

    /**
     * @brief Converters that keep a snapshot, for the interrupt to find the one that completed.
     */
    static std::array<AdcDma*, max_instances> instances;

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
     * @brief Destination buffer of the DMA.
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
     * @brief Flag to check if the ADC was calibrated and started.
     */
    bool initialized{};
};
}  // namespace micras::hal

#endif  // MICRAS_HAL_ADC_DMA_HPP
