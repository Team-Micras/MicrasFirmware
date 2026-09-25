/**
 * @file
 */

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "micras/hal/crc.hpp"
#include "micras/hal/gpio.hpp"
#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/hal/timer.hpp"
#include "target.hpp"

namespace micras::sim {
namespace {
class HostHal : public testing::Test {
protected:
    void SetUp() override {
        hal::host::Board::reset();
        hal::host::Clock::instance().reset();
        hal::host::Clock::instance().configure(SystemCoreClock / 1000000);
        hal::Timer::init();
    }
};

TEST_F(HostHal, ComputesTheCrcTheUnitIsConfiguredFor) {
    CRC_HandleTypeDef handle{};
    handle.Init = {
        .DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE,
        .DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE,
        .GeneratingPolynomial = 0x1D,
        .CRCLength = CRC_POLYLENGTH_8B,
        .InitValue = 0xFF,
        .InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE,
        .OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE,
    };
    hal::Crc crc{{.handle = &handle}};

    constexpr std::string_view check{"123456789"};
    const std::vector<uint8_t> bytes(check.begin(), check.end());

    EXPECT_EQ(crc.calculate(bytes) ^ 0xFFU, 0x4BU);
}

TEST_F(HostHal, ChargesAQuantumPerReadAndHandsOverAtEveryStep) {
    int steps = 0;
    hal::host::Clock::instance().set_handover(125, [&steps] { steps++; });

    const uint32_t start = hal::Timer::get_counter();

    while (hal::Timer::to_microseconds(hal::Timer::get_counter() - start) < 1000) { }

    EXPECT_EQ(steps, 8);
    EXPECT_EQ(hal::Timer::get_counter_ms(), 1U);
}

TEST_F(HostHal, ReadsWhatTheOutsideDrivesAndOtherwiseWhatTheFirmwareWrote) {
    hal::Gpio led{led_config.gpio};
    led.write(true);

    hal::host::GpioPort& port = hal::host::Board::gpio(led_config.gpio.port, led_config.gpio.pin);
    EXPECT_TRUE(port.output);
    EXPECT_TRUE(led.read());

    port.input = false;
    EXPECT_FALSE(led.read());
    EXPECT_TRUE(port.touched);
}

TEST_F(HostHal, RunsTheWallEmittersAtTheirConfiguredFrequency) {
    const hal::Pwm emitter{std::get<0>(wall_sensors_config.led_pwms)};

    EXPECT_TRUE(emitter.was_initialized());
    EXPECT_NEAR(emitter.get_frequency(), wall_sensors_frequency, 1.0F);
}

TEST_F(HostHal, ReportsPortsTheFirmwareTouchedThatNothingIsBoundTo) {
    hal::Gpio led{led_config.gpio};
    led.write(true);

    EXPECT_EQ(hal::host::Board::unbound().size(), 1U);

    hal::host::Board::gpio(led_config.gpio.port, led_config.gpio.pin).bound = true;
    EXPECT_TRUE(hal::host::Board::unbound().empty());
}
}  // namespace
}  // namespace micras::sim
