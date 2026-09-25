/**
 * @file
 */

#ifndef MICRAS_PROXY_IMU_HPP
#define MICRAS_PROXY_IMU_HPP

#include <array>
#include <cstdint>
#include <lsm6dsv_reg.h>
#include <numbers>

#include "micras/hal/spi.hpp"

namespace micras::proxy {
/**
 * @brief Class for acquiring IMU data.
 *
 * @note The rates are delivered as the sensor reports them, with their bias still in them. The bias
 * of a gyroscope drifts with temperature and time, so it is estimated continuously by whoever
 * integrates the rate, not measured once here.
 *
 * @note The samples are read by the DMA into a buffer of this object, which therefore has to live in
 * memory that the DMA reaches.
 */
class Imu {
public:
    /**
     * @brief IMU configuration struct.
     *
     * @note The data rates of the high accuracy mode, which are the ones that are round numbers,
     * exist only with both sensors in that mode.
     */
    struct Config {
        hal::Spi::Config                spi;
        lsm6dsv_gy_mode_t               gyroscope_mode;
        lsm6dsv_xl_mode_t               accelerometer_mode;
        lsm6dsv_data_rate_t             gyroscope_data_rate;
        lsm6dsv_data_rate_t             accelerometer_data_rate;
        lsm6dsv_gy_full_scale_t         gyroscope_scale;
        lsm6dsv_xl_full_scale_t         accelerometer_scale;
        lsm6dsv_filt_gy_lp1_bandwidth_t gyroscope_filter;
        lsm6dsv_filt_xl_lp2_bandwidth_t accelerometer_filter;
    };

    /**
     * @brief Enum to select the axis of the IMU.
     */
    enum class Axis : uint8_t {
        X = 0,
        Y = 1,
        Z = 2
    };

    /**
     * @brief Construct a new Imu object.
     *
     * @param config Configuration for the IMU.
     */
    explicit Imu(const Config& config);

    /**
     * @brief Take the sample that the last call asked for, and ask for the next one.
     *
     * @details The status, the angular rate and the acceleration are read in one transfer that the
     * DMA carries out while the caller does something else, so a call costs the time to start a
     * transfer and never the time the transfer takes. The price is that a sample is one call old
     * when it is used.
     */
    void update();

    /**
     * @brief Check whether the last update brought an angular rate that was not seen before.
     *
     * @note The sensor samples on its own clock, so a caller running faster than the output data
     * rate, or close to it, gets the same sample more than once.
     *
     * @return True if the angular rate is a new sample, false otherwise.
     */
    bool is_new() const;

    /**
     * @brief Get the IMU angular velocity over an axis.
     *
     * @param axis Axis to get the angular velocity from.
     * @return Angular velocity over the desired axis in rad/s.
     */
    float get_angular_velocity(Axis axis) const;

    /**
     * @brief Get the IMU linear acceleration over an axis.
     *
     * @param axis Axis to get the linear acceleration from.
     * @return Linear acceleration over the desired axis in m/s².
     */
    float get_linear_acceleration(Axis axis) const;

    /**
     * @brief Check if IMU was initialized.
     *
     * @return True if the device was successfully initialized, false otherwise.
     */
    bool was_initialized() const;

private:
    /**
     * @brief Check the IMU device.
     *
     * @return True if the device is correct, false otherwise.
     */
    bool check_whoami();

    /**
     * @brief Convert the registers that the last transfer brought.
     */
    void read_response();

    /**
     * @brief Read data from the IMU.
     *
     * @param handle Pointer to a SPI object.
     * @param reg Register to read from.
     * @param bufp Buffer to read.
     * @param len Length of the buffer.
     * @return 0 if the operation was successful, -1 otherwise.
     */
    static int32_t platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);

    /**
     * @brief Write data to the IMU.
     *
     * @param handle Pointer to a SPI object.
     * @param reg Register to write to.
     * @param bufp Buffer to write.
     * @param len Length of the buffer.
     * @return 0 if the operation was successful, -1 otherwise.
     */
    static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);

    /**
     * @brief Bit of the register address that makes a transfer a read.
     */
    static constexpr uint8_t read_flag{0x80};

    /**
     * @brief Number of registers from the status to the last byte of the acceleration.
     */
    static constexpr uint8_t burst_size{16};

    /**
     * @brief Offsets of the angular rate and of the acceleration from the status register.
     */
    ///@{
    static constexpr uint8_t angular_rate_offset{4};
    static constexpr uint8_t acceleration_offset{10};
    ///@}

    /**
     * @brief Bits of the status register that tell which sensor has a new sample.
     */
    ///@{
    static constexpr uint8_t accelerometer_ready{0x01};
    static constexpr uint8_t gyroscope_ready{0x02};
    ///@}

    /**
     * @brief Conversion constants.
     */
    static constexpr float mdps_to_radps{std::numbers::pi_v<float> / 180000.0F};
    static constexpr float mg_to_mps2{0.00980665F};

    /**
     * @brief SPI for the IMU communication.
     */
    hal::Spi spi;

    /**
     * @brief Device context for the IMU library.
     */
    stmdev_ctx_t dev_ctx{};

    /**
     * @brief Address of the status register, followed by the padding that clocks the registers out.
     */
    std::array<uint8_t, burst_size + 1> command{LSM6DSV_STATUS_REG | read_flag};

    /**
     * @brief Registers received by the last transfer, after the byte that answers the address.
     */
    std::array<uint8_t, burst_size + 1> response{};

    /**
     * @brief Current angular velocity on each axis.
     */
    std::array<float, 3> angular_velocity{};

    /**
     * @brief Current linear acceleration on each axis.
     */
    std::array<float, 3> linear_acceleration{};

    /**
     * @brief Gyroscope conversion factor.
     */
    float gy_factor;

    /**
     * @brief Accelerometer conversion factor.
     */
    float xl_factor;

    /**
     * @brief Flag to check if the last update brought a new angular rate.
     */
    bool fresh{};

    /**
     * @brief Flag to check if the IMU was initialized.
     */
    bool initialized{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_IMU_HPP
