/**
 * @file
 */

#include <cmath>
#include <cstdint>
#include <string_view>
#include <vector>

#include <doctest/doctest.h>

#include "constants.hpp"
#include "micras/hal/crc.hpp"
#include "micras/hal/gpio.hpp"
#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/host/ports.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/hal/timer.hpp"
#include "micras/nav/robot_model.hpp"
#include "target.hpp"

namespace micras::sim {
namespace {
class HostHal {
protected:
    HostHal() {
        hal::host::Board::reset();
        hal::host::Clock::instance().reset();
        hal::host::Clock::instance().configure(SystemCoreClock / 1000000);
        hal::Timer::init();
    }
};

TEST_CASE_FIXTURE(HostHal, "HostHal.ComputesTheCrcTheUnitIsConfiguredFor") {
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

    CHECK_EQ(crc.calculate(bytes) ^ 0xFFU, 0x4BU);
}

TEST_CASE_FIXTURE(HostHal, "HostHal.ChargesAQuantumPerReadAndHandsOverAtEveryStep") {
    int steps = 0;
    hal::host::Clock::instance().set_handover(125, [&steps] { steps++; });

    const uint32_t start = hal::Timer::get_counter();

    while (hal::Timer::to_microseconds(hal::Timer::get_counter() - start) < 1000) { }

    CHECK_EQ(steps, 8);
    CHECK_EQ(hal::Timer::get_counter_ms(), 1U);
}

TEST_CASE_FIXTURE(HostHal, "HostHal.ReadsWhatTheOutsideDrivesAndOtherwiseWhatTheFirmwareWrote") {
    hal::Gpio led{led_config.gpio};
    led.write(true);

    hal::host::GpioPort& port = hal::host::Board::gpio(led_config.gpio.port, led_config.gpio.pin);
    CHECK(port.output);
    CHECK(led.read());

    port.input = false;
    CHECK_FALSE(led.read());
    CHECK(port.touched);
}

TEST_CASE_FIXTURE(HostHal, "HostHal.RunsTheWallEmittersAtTheirConfiguredFrequency") {
    const hal::Pwm emitter{std::get<0>(wall_sensors_config.led_pwms)};

    CHECK(emitter.was_initialized());
    const float cycles = 2.0F * emitter.get_frequency() / static_cast<float>(nav::number_of_wall_sensors + 1);

    CHECK_LE(std::abs(cycles - wall_sensors_frequency), 1.0F);
}

TEST_CASE_FIXTURE(HostHal, "HostHal.ReportsPortsTheFirmwareTouchedThatNothingIsBoundTo") {
    hal::Gpio led{led_config.gpio};
    led.write(true);

    CHECK_EQ(hal::host::Board::unbound().size(), 1U);

    hal::host::Board::gpio(led_config.gpio.port, led_config.gpio.pin).bound = true;
    CHECK(hal::host::Board::unbound().empty());
}
}  // namespace
}  // namespace micras::sim
