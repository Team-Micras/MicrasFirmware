/**
 * @file
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include <doctest/doctest.h>

#include "micras/hal/crc.hpp"
#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/timer.hpp"
#include "micras/models/as5047u_model.hpp"
#include "micras/models/lsm6dsv_model.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "target.hpp"

namespace micras::sim {
namespace {
using hal::host::Board;

/**
 * @brief The firmware's IMU and rotary sensor proxies, compiled as they are, over the chip models.
 */
class SpiChips {
protected:
    SpiChips() {
        Board::reset();
        hal::host::Clock::instance().reset();
        hal::host::Clock::instance().configure(SystemCoreClock / 1000000);
        hal::Timer::init();
        MX_GPIO_Init();
        MX_SPI3_Init();
        MX_CRC_Init();
    }

    static void attach(const hal::Spi::Config& config, hal::host::SpiDevice& device) {
        Board::spi_device(config.handle, config.cs_gpio.port, config.cs_gpio.pin, device);
    }

    static void wait_us(uint32_t microseconds) {
        const uint32_t start = hal::Timer::get_counter();

        while (hal::Timer::to_microseconds(hal::Timer::get_counter() - start) < microseconds) { }
    }
};

TEST_CASE_FIXTURE(SpiChips, "SpiChips.SetsTheImuUpAsItsConfigurationSays") {
    models::Lsm6dsvModel chip;
    attach(imu_config.spi, chip);

    const proxy::Imu imu{imu_config};

    CHECK(imu.was_initialized());
    CHECK_EQ(chip.gyroscope_data_rate(), 8000.0);
    CHECK_EQ(chip.accelerometer_data_rate(), 8000.0);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL6) & 0x0F, LSM6DSV_4000dps);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL8) & 0x03, LSM6DSV_8g);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL3) & 0x40, 0x40);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL7) & 0x01, 0x01);
    CHECK_EQ(chip.peek(LSM6DSV_CTRL9) & 0x08, 0x08);
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.DelaysTheImuUntilItsChipHasStarted") {
    models::Lsm6dsvModel chip;
    attach(imu_config.spi, chip);
    const uint64_t start = hal::host::Clock::instance().now();

    const proxy::Imu imu{imu_config};
    const uint64_t   elapsed_us = (hal::host::Clock::instance().now() - start) / (SystemCoreClock / 1000000);

    CHECK(imu.was_initialized());
    CHECK_GE(elapsed_us, 39900U);
    CHECK_LT(elapsed_us, 41000U);
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.ReadsASampleOneUpdateAfterTheUpdateThatAskedForIt") {
    models::Lsm6dsvModel chip;
    attach(imu_config.spi, chip);
    proxy::Imu imu{imu_config};

    chip.push_sample({0.1, -0.2, 3.0}, {1.0, -2.0, 9.80665});
    imu.update();

    CHECK_FALSE(imu.is_new());

    wait_us(125);
    imu.update();

    REQUIRE(imu.is_new());
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_angular_velocity(proxy::Imu::Axis::X)) - 0.1), chip.gyroscope_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_angular_velocity(proxy::Imu::Axis::Y)) - (-0.2)),
        chip.gyroscope_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_angular_velocity(proxy::Imu::Axis::Z)) - 3.0), chip.gyroscope_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_linear_acceleration(proxy::Imu::Axis::X)) - 1.0),
        chip.accelerometer_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_linear_acceleration(proxy::Imu::Axis::Y)) - (-2.0)),
        chip.accelerometer_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_linear_acceleration(proxy::Imu::Axis::Z)) - 9.80665),
        chip.accelerometer_sensitivity()
    );

    wait_us(125);
    imu.update();

    CHECK_FALSE(imu.is_new());
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.ReadsTheSameMotionAtAnotherFullScale") {
    proxy::Imu::Config config = imu_config;
    config.gyroscope_scale = LSM6DSV_250dps;
    config.accelerometer_scale = LSM6DSV_2g;
    models::Lsm6dsvModel chip;
    attach(config.spi, chip);
    proxy::Imu imu{config};

    chip.push_sample({0.5, 0.0, -1.5}, {0.0, 3.0, 9.80665});
    imu.update();
    wait_us(125);
    imu.update();

    REQUIRE(imu.is_new());
    CHECK_EQ(chip.peek(LSM6DSV_CTRL6) & 0x0F, LSM6DSV_250dps);
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_angular_velocity(proxy::Imu::Axis::X)) - 0.5), chip.gyroscope_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_angular_velocity(proxy::Imu::Axis::Z)) - (-1.5)),
        chip.gyroscope_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_linear_acceleration(proxy::Imu::Axis::Y)) - 3.0),
        chip.accelerometer_sensitivity()
    );
    CHECK_LE(
        std::abs(static_cast<double>(imu.get_linear_acceleration(proxy::Imu::Axis::Z)) - 9.80665),
        chip.accelerometer_sensitivity()
    );
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.FailsTheImuWithoutItsChip") {
    const proxy::Imu imu{imu_config};

    CHECK_FALSE(imu.was_initialized());
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.FailsTheImuInTheWrongSpiMode") {
    proxy::Imu::Config config = imu_config;
    config.spi.clock_polarity = SPI_POLARITY_LOW;
    models::Lsm6dsvModel chip;
    attach(config.spi, chip);

    const proxy::Imu imu{config};

    CHECK_FALSE(imu.was_initialized());
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.SetsTheRotarySensorUpAndReadsItsResolutionBack") {
    models::As5047uModel left_chip;
    models::As5047uModel right_chip;
    attach(rotary_sensor_left_config.spi, left_chip);
    attach(rotary_sensor_right_config.spi, right_chip);

    const proxy::RotarySensor left{rotary_sensor_left_config};
    const proxy::RotarySensor right{rotary_sensor_right_config};

    CHECK(left.was_initialized());
    CHECK(right.was_initialized());
    CHECK_EQ(left.get_resolution(), 16384U);
    CHECK_EQ(left_chip.peek(models::As5047uModel::settings3_address), rotary_sensor_reg_config.settings3.raw);
    CHECK_EQ(left_chip.peek(models::As5047uModel::disable_address), rotary_sensor_reg_config.disable.raw);
    CHECK_EQ(right_chip.peek(models::As5047uModel::settings3_address), rotary_sensor_reg_config.settings3.raw);
    CHECK_EQ(left_chip.peek(models::As5047uModel::errfl_address), 0);
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.FailsTheRotarySensorWhoseFramesTheChipRejects") {
    CRC_HandleTypeDef wrong_crc = hcrc;
    wrong_crc.Init.InitValue = 0x00;
    proxy::RotarySensor::Config config = rotary_sensor_left_config;
    config.crc.handle = &wrong_crc;
    models::As5047uModel chip;
    attach(config.spi, chip);

    const proxy::RotarySensor sensor{config};

    CHECK_FALSE(sensor.was_initialized());
    CHECK_EQ(chip.peek(models::As5047uModel::settings3_address), 0);
    CHECK_NE(chip.peek(models::As5047uModel::errfl_address) & models::As5047uModel::crc_error, 0);
}

TEST_CASE_FIXTURE(SpiChips, "SpiChips.ComputesTheFrameCrcAsTheCrcUnitDoes") {
    hal::Crc crc{{.handle = &hcrc}};

    for (const std::array<uint8_t, 2> bytes : std::vector<std::array<uint8_t, 2>>{
             {0x00, 0x00}, {0x40, 0x01}, {0x40, 0x1A}, {0x00, 0x80}, {0x3F, 0xFF}, {0xC0, 0x15}
         }) {
        CHECK_EQ(models::As5047uModel::crc(std::get<0>(bytes), std::get<1>(bytes)), crc.calculate(bytes) ^ 0xFFU);
    }
}
}  // namespace
}  // namespace micras::sim
