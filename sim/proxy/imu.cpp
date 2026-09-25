/**
 * @file
 *
 * @brief Imu for the simulator: the samples come from the simulated chip, not over SPI.
 *
 * @note Replaces micras_proxy/src/imu.cpp. The real driver talks to the LSM6DSV
 *       through ST's register driver; here the simulator's IMU device writes the
 *       chip's raw output words into the "imu" sample port, gyroscope X, Y, Z then
 *       accelerometer X, Y, Z, and this file scales them with the same factors
 *       the real driver derives from the configured full scales. A new sample is
 *       one the port's sequence has not shown yet, as the chip's data ready bits
 *       say on the robot.
 */

#include <cstdint>
#include <unordered_map>

#include "micras/hal/host/board.hpp"
#include "micras/proxy/imu.hpp"

namespace micras::proxy {
namespace {
/**
 * @brief Sequence of the last sample each Imu read.
 *
 * @note Kept outside the class because the header, shared with the robot, has no
 *       member for it.
 */
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): per-object state the header has no room for.
std::unordered_map<const Imu*, uint32_t> last_sequences;
}  // namespace

Imu::Imu(const Config& config) :
    spi{config.spi},
    gy_factor{
        mdps_to_radps * 4.375F *
        (1 << (config.gyroscope_scale == LSM6DSV_4000dps ? 5 : static_cast<uint8_t>(config.gyroscope_scale)))
    },
    xl_factor{mg_to_mps2 * (0.061F * (1 << static_cast<uint8_t>(config.accelerometer_scale)))},
    initialized{this->spi.was_initialized()} {
    hal::host::Board::samples("imu").touched = true;
    last_sequences[this] = hal::host::Board::samples("imu").sequence;
}

void Imu::update() {
    const hal::host::SamplePort& port = hal::host::Board::samples("imu");
    uint32_t&                    last = last_sequences[this];

    this->fresh = port.sequence != last;

    if (not this->fresh) {
        return;
    }

    last = port.sequence;

    for (uint8_t axis = 0; axis < 3; axis++) {
        this->angular_velocity.at(axis) =
            static_cast<float>(static_cast<int16_t>(port.values.at(axis))) * this->gy_factor;
        this->linear_acceleration.at(axis) =
            static_cast<float>(static_cast<int16_t>(port.values.at(3 + axis))) * this->xl_factor;
    }
}

bool Imu::is_new() const {
    return this->fresh;
}

float Imu::get_angular_velocity(Axis axis) const {
    return this->angular_velocity.at(static_cast<uint8_t>(axis));
}

float Imu::get_linear_acceleration(Axis axis) const {
    return this->linear_acceleration.at(static_cast<uint8_t>(axis));
}

bool Imu::was_initialized() const {
    return this->initialized;
}
}  // namespace micras::proxy
