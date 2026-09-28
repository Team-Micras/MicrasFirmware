/**
 * @file
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <doctest/doctest.h>

#include "gpio.h"
#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/host/ports.hpp"
#include "micras/hal/host/spi_device.hpp"
#include "micras/hal/spi.hpp"
#include "micras/hal/timer.hpp"
#include "micras/sim/core/span_at.hpp"
#include "spi.h"
#include "target.hpp"

namespace micras::sim {
namespace {
using hal::host::Board;
using hal::host::SpiDevice;

/**
 * @brief A device that records what reaches it and answers each byte with the byte plus an offset.
 */
class RecordingDevice : public SpiDevice {
public:
    RecordingDevice(Mode mode, uint8_t offset) : SpiDevice{mode}, offset{offset} { }

    void select() override { this->events.emplace_back("select"); }

    void exchange(std::span<const uint8_t> transmitted, std::span<uint8_t> received) override {
        this->events.push_back("exchange " + std::to_string(transmitted.size()));

        for (std::size_t index = 0; index < transmitted.size(); index++) {
            this->bytes.push_back(at(transmitted, index));
            at(received, index) = static_cast<uint8_t>(at(transmitted, index) + this->offset);
        }
    }

    void deselect() override { this->events.emplace_back("deselect"); }

    // NOLINTBEGIN(*-non-private-member-variables-in-classes): what the test reads back.
    std::vector<std::string> events;
    std::vector<uint8_t>     bytes;
    // NOLINTEND(*-non-private-member-variables-in-classes)

private:
    uint8_t offset;
};

class SpiSlot {
protected:
    SpiSlot() {
        Board::reset();
        hal::host::Clock::instance().reset();
        hal::host::Clock::instance().configure(SystemCoreClock / 1000000);
        hal::Timer::init();
        MX_GPIO_Init();
        MX_SPI3_Init();
    }

    static void attach(const hal::Spi::Config& config, SpiDevice& device) {
        Board::spi_device(config.handle, config.cs_gpio.port, config.cs_gpio.pin, device);
    }

    /**
     * @brief Poll a transfer until it ends, reading the timer as a waiting firmware does.
     *
     * @param spi The Spi whose transfer runs.
     * @return Microseconds the host clock advanced.
     */
    static uint32_t wait_for_transfer(const hal::Spi& spi) {
        const uint64_t start = hal::host::Clock::instance().now();

        while (spi.get_transfer() == hal::Spi::Transfer::RUNNING) {
            hal::Timer::get_counter();
        }

        return static_cast<uint32_t>((hal::host::Clock::instance().now() - start) / (SystemCoreClock / 1000000));
    }
};

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.RoutesEachTransferToTheDeviceItsChipSelectSelects") {
    RecordingDevice imu_chip{SpiDevice::Mode::MODE_3, 0x10};
    RecordingDevice encoder_chip{SpiDevice::Mode::MODE_1, 0x20};
    attach(imu_config.spi, imu_chip);
    attach(rotary_sensor_left_config.spi, encoder_chip);
    hal::Spi imu{imu_config.spi};
    hal::Spi encoder{rotary_sensor_left_config.spi};

    const std::array<uint8_t, 1> command{0x8F};
    const std::array<uint8_t, 2> frame{0x40, 0x01};
    std::array<uint8_t, 2>       answer{};

    REQUIRE(imu.select_device());
    CHECK(imu.transmit(command));
    imu.unselect_device();
    REQUIRE(encoder.select_device());
    CHECK(encoder.transmit_receive(frame, answer));
    encoder.unselect_device();

    CHECK_EQ(imu_chip.bytes, std::vector<uint8_t>{0x8F});
    CHECK_EQ(encoder_chip.bytes, (std::vector<uint8_t>{0x40, 0x01}));
    CHECK_EQ(answer, (std::array<uint8_t, 2>{0x60, 0x21}));
    CHECK(Board::unbound().empty());
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.MakesOneTransactionOfTheCallsBetweenSelectAndUnselect") {
    RecordingDevice chip{SpiDevice::Mode::MODE_3, 1};
    attach(imu_config.spi, chip);
    hal::Spi spi{imu_config.spi};

    const std::array<uint8_t, 1> command{0x8F};
    std::array<uint8_t, 2>       data{};

    REQUIRE(spi.select_device());
    CHECK(spi.transmit(command));
    CHECK(spi.receive(data));
    spi.unselect_device();

    CHECK_EQ(chip.events, (std::vector<std::string>{"select", "exchange 1", "exchange 2", "deselect"}));
    CHECK_EQ(data, (std::array<uint8_t, 2>{1, 1}));
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.ReachesNoDeviceWhoseChipSelectIsHigh") {
    RecordingDevice chip{SpiDevice::Mode::MODE_3, 1};
    attach(imu_config.spi, chip);
    hal::Spi spi{imu_config.spi};

    std::array<uint8_t, 2> data{};
    CHECK(spi.receive(data));

    CHECK(chip.events.empty());
    CHECK_EQ(data, (std::array<uint8_t, 2>{0xFF, 0xFF}));
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.AnswersAllOnesWhereNoDeviceIsAttached") {
    hal::Spi spi{imu_config.spi};

    std::array<uint8_t, 2> data{};
    REQUIRE(spi.select_device());
    CHECK(spi.receive(data));
    spi.unselect_device();

    CHECK_EQ(data, (std::array<uint8_t, 2>{0xFF, 0xFF}));
    CHECK_EQ(Board::unbound(), (std::vector<std::string>{"IMU_SPI_CSn", "hspi3 IMU_SPI_CSn"}));
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.AnswersAllOnesInAModeTheDeviceDoesNotAnswerIn") {
    RecordingDevice chip{SpiDevice::Mode::MODE_0, 1};
    attach(imu_config.spi, chip);
    hal::Spi spi{imu_config.spi};

    const std::array<uint8_t, 2> transmitted{0x8F, 0x00};
    std::array<uint8_t, 2>       received{};
    REQUIRE(spi.select_device());
    CHECK(spi.transmit_receive(transmitted, received));
    spi.unselect_device();

    CHECK_EQ(chip.events, (std::vector<std::string>{"select", "deselect"}));
    CHECK_EQ(received, (std::array<uint8_t, 2>{0xFF, 0xFF}));
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.KeepsTheDeviceSelectedWhileATransferRuns") {
    RecordingDevice chip{SpiDevice::Mode::MODE_3, 1};
    attach(imu_config.spi, chip);
    hal::Spi spi{imu_config.spi};

    std::array<uint8_t, 17>    transmitted{};
    std::array<uint8_t, 17>    received{};
    const hal::host::GpioPort& cs = Board::gpio(imu_config.spi.cs_gpio.port, imu_config.spi.cs_gpio.pin);

    REQUIRE(spi.start_transfer(transmitted, received));

    CHECK_EQ(spi.get_transfer(), hal::Spi::Transfer::RUNNING);
    CHECK_FALSE(cs.output);
    CHECK_EQ(chip.events, (std::vector<std::string>{"select", "exchange 17"}));
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.CompletesATransferWhenItsLastBitIsOut") {
    RecordingDevice chip{SpiDevice::Mode::MODE_3, 1};
    attach(imu_config.spi, chip);
    hal::Spi spi{imu_config.spi};

    std::array<uint8_t, 17>    transmitted{};
    std::array<uint8_t, 17>    received{};
    const hal::host::GpioPort& cs = Board::gpio(imu_config.spi.cs_gpio.port, imu_config.spi.cs_gpio.pin);

    REQUIRE(spi.start_transfer(transmitted, received));
    const uint32_t elapsed = wait_for_transfer(spi);

    CHECK_EQ(spi.get_transfer(), hal::Spi::Transfer::COMPLETE);
    CHECK(cs.output);
    CHECK_EQ(chip.events, (std::vector<std::string>{"select", "exchange 17", "deselect"}));
    CHECK_EQ(received.back(), 1);
    CHECK_GE(elapsed, 35U);
    CHECK_LE(elapsed, 36U);
}

TEST_CASE_FIXTURE(SpiSlot, "SpiSlot.WaitsForTheBusBeforeSelectingAnotherDevice") {
    RecordingDevice imu_chip{SpiDevice::Mode::MODE_3, 1};
    RecordingDevice encoder_chip{SpiDevice::Mode::MODE_1, 1};
    attach(imu_config.spi, imu_chip);
    attach(rotary_sensor_right_config.spi, encoder_chip);
    hal::Spi imu{imu_config.spi};
    hal::Spi encoder{rotary_sensor_right_config.spi};

    std::array<uint8_t, 17> transmitted{};
    std::array<uint8_t, 17> received{};
    const uint64_t          start = hal::host::Clock::instance().now();

    REQUIRE(imu.start_transfer(transmitted, received));
    REQUIRE(encoder.select_device());
    const uint64_t waited = hal::host::Clock::instance().now() - start;
    encoder.unselect_device();

    CHECK_EQ(imu.get_transfer(), hal::Spi::Transfer::COMPLETE);
    CHECK_EQ(imu_chip.events.back(), "deselect");
    CHECK_GE(waited, 35U * (SystemCoreClock / 1000000));
}
}  // namespace
}  // namespace micras::sim
