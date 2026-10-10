/**
 * @file
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "constants.hpp"
#include "micras/hal/gpio.hpp"
#include "micras/hal/host/board.hpp"
#include "micras/hal/host/ports.hpp"
#include "micras/hal/host/spi_device.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/hal/spi.hpp"
#include "micras/models/as5047u_model.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/motor.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/sim/app/wiring.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/core/span_at.hpp"
#include "micras/sim/devices/dc_motor.hpp"
#include "micras/sim/devices/digital_input.hpp"
#include "micras/sim/devices/imu.hpp"
#include "micras/sim/devices/power.hpp"
#include "micras/sim/devices/quadrature_encoder.hpp"
#include "micras/sim/devices/serial_link.hpp"
#include "micras/sim/devices/wall_sensors.hpp"
#include "micras/sim/micras/bindings.hpp"
#include "micras/sim/micras/micras_target.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/robot/robot_model.hpp"
#include "target.hpp"

namespace micras::sim {
namespace {
using hal::host::Board;
}  // namespace

/**
 * @brief Add a device to the run and keep a view of it.
 *
 * @param context Context whose devices are added to.
 * @param device The device.
 * @return The view.
 */
template <typename T>
static T* add(RunContext& context, std::unique_ptr<T> device) {
    T* view = device.get();
    context.devices.push_back(std::move(device));
    return view;
}

/**
 * @brief Get a PWM port, marked as bound.
 *
 * @param config The firmware's configuration of the channel.
 * @return The port.
 */
static hal::host::PwmPort& pwm_port(const hal::Pwm::Config& config) {
    hal::host::PwmPort& port = Board::pwm(config.handle, config.timer_channel);
    port.bound = true;
    return port;
}

/**
 * @brief Get a GPIO port, marked as bound.
 *
 * @param config The firmware's configuration of the pin.
 * @return The port.
 */
static hal::host::GpioPort& gpio_port(const hal::Gpio::Config& config) {
    hal::host::GpioPort& port = Board::gpio(config.port, config.pin);
    port.bound = true;
    return port;
}

/**
 * @brief Build the schedule of the wall emitters from the table the emitter timer's update DMA loads.
 *
 * @note The timer counts up and down from zero and converts at both ends, the first being an
 * overflow: an inverted output is lit around an overflow while its compare value is within the
 * count, any other around an underflow while its compare value is above zero, in both halves of
 * the count that meet at that end. Counting the updates from zero, the first row of the burst is in
 * force until update 0, the second until update 1, and row k of the table from update k + 1 on. The
 * scans start over with the cycle whenever the firmware arms the burst again, and each half of the
 * cycle is a frame the firmware reads.
 *
 * @param burst The port of the emitter timer.
 * @return The schedule.
 */
static std::function<std::optional<WallSensors::Schedule>()>
    make_emitter_schedule(const hal::host::TimerBurstPort& burst) {
    struct Progress {
        uint32_t arms{};
        uint64_t ends{};
    };

    return [&burst, progress = Progress{}]() mutable -> std::optional<WallSensors::Schedule> {
        if (not burst.running or burst.first.empty()) {
            return std::nullopt;
        }

        if (burst.arms != progress.arms) {
            progress = {.arms = burst.arms, .ends = 0};
        }

        const std::size_t registers = burst.first.size();
        const std::size_t length = burst.table.size() / registers;
        const uint64_t    end = progress.ends++;
        const uint32_t    autoreload = wall_sensors_config.burst.handle->Instance->ARR;

        const auto row = [&burst, registers, length](uint64_t half) -> std::span<const uint32_t> {
            if (half < 2) {
                return half == 0 ? burst.first : burst.second;
            }

            return burst.table.subspan(((half - 2) % length) * registers, registers);
        };

        const std::span<const uint32_t> before = row(end);
        const std::span<const uint32_t> after = row(end + 1);
        const bool                      overflow = end % 2 == 0;
        std::vector<bool>               lit(registers, false);

        for (std::size_t emitter = 0; emitter < registers; emitter++) {
            const bool inverted = wall_sensors_config.led_pwms.at(emitter).inverted;
            const auto on = [inverted, autoreload](uint32_t compare) {
                return inverted ? compare <= autoreload : compare > 0;
            };

            lit.at(emitter) = inverted == overflow and on(sim::at(before, emitter)) and on(sim::at(after, emitter));
        }

        const std::size_t position = end % length;
        const std::size_t frame = length / 2;

        return WallSensors::Schedule{
            .position = position,
            .length = length,
            .lit = std::move(lit),
            .last = position % frame == frame - 1,
        };
    };
}

/**
 * @brief Build one wheel's motor.
 *
 * @param context Context whose devices are added to.
 * @param world The robot.
 * @param motor The firmware's configuration of the motor.
 * @param side "left" or "right".
 * @return The motor.
 */
static DcMotor* bind_motor(
    RunContext& context, const WorldInfo& world, const proxy::Motor::Config& motor, const std::string& side
) {
    const RobotModelNames names = RobotModelNames::of(*world.robot);
    hal::host::PwmPort&   forward = pwm_port(motor.forward_pwm);
    hal::host::PwmPort&   backward = pwm_port(motor.backwards_pwm);
    hal::host::GpioPort&  enable = gpio_port(locomotion_config.enable_gpio);

    return add(
        context, std::make_unique<DcMotor>(
                     context.world, DcMotor::Config{
                                        .name = "motor_" + side,
                                        .actuator = side == "left" ? names.left_motor : names.right_motor,
                                        .joint = side == "left" ? names.left_wheel : names.right_wheel,
                                        .drive = world.robot->drive,
                                        .forward_duty = [&forward] { return forward.duty_cycle; },
                                        .backward_duty = [&backward] { return backward.duty_cycle; },
                                        .enabled = [&enable] { return enable.output; },
                                    }
                 )
    );
}

/**
 * @brief Attach a chip to the bus and chip select the firmware's configuration names.
 *
 * @param spi The firmware's configuration of the chip's SPI.
 * @param chip The chip.
 */
static void attach(const hal::Spi::Config& spi, hal::host::SpiDevice& chip) {
    Board::spi_device(spi.handle, spi.cs_gpio.port, spi.cs_gpio.pin, chip);
}

/**
 * @brief Build one wheel's encoder.
 *
 * @param context Context whose devices are added to.
 * @param world The robot.
 * @param sensor The firmware's configuration of the rotary sensor.
 * @param chip The rotary sensor's chip.
 * @param side "left" or "right".
 */
static void bind_encoder(
    RunContext& context, const WorldInfo& world, const proxy::RotarySensor::Config& sensor, models::As5047uModel& chip,
    const std::string& side
) {
    const RobotModelNames   names = RobotModelNames::of(*world.robot);
    hal::host::EncoderPort& port = Board::encoder(sensor.encoder.handle);
    port.bound = true;
    attach(sensor.spi, chip);

    add(context, std::make_unique<QuadratureEncoder>(
                     context.world, QuadratureEncoder::Config{
                                        .name = "encoder_" + side,
                                        .joint = side == "left" ? names.left_wheel : names.right_wheel,
                                        .counts_per_revolution = world.robot->encoders.counts_per_revolution,
                                        .write = [&port](int32_t count) { port.count = count; },
                                    }
                 ));
}

/**
 * @brief Build a two-position input driving a GPIO.
 *
 * @param context Context whose devices are added to.
 * @param name Name of the input.
 * @param gpio The firmware's configuration of the pin.
 * @param active_low Whether pressed, or on, reads low.
 * @return The input.
 */
static DigitalInput*
    bind_input(RunContext& context, const std::string& name, const hal::Gpio::Config& gpio, bool active_low) {
    hal::host::GpioPort& port = gpio_port(gpio);

    return add(
        context, std::make_unique<DigitalInput>(DigitalInput::Config{
                     .name = name,
                     .active_low = active_low,
                     .drive = [&port](bool level) { port.input = level; },
                 })
    );
}

MicrasBoard bind_devices(RunContext& context, const WorldInfo& world, MicrasChips& chips) {
    const RobotModelNames names = RobotModelNames::of(*world.robot);
    MicrasBoard           board;

    board.left_motor = bind_motor(context, world, locomotion_config.left_motor, "left");
    board.right_motor = bind_motor(context, world, locomotion_config.right_motor, "right");
    bind_encoder(context, world, rotary_sensor_left_config, chips.left_encoder, "left");
    bind_encoder(context, world, rotary_sensor_right_config, chips.right_encoder, "right");

    attach(imu_config.spi, chips.imu);
    const ImuDescription& imu = world.robot->imu;

    add(context,
        std::make_unique<Imu>(
            context.world,
            Imu::Config{
                .name = "lsm6dsv",
                .gyro = names.gyro,
                .accelerometer = names.accelerometer,
                .description = imu,
                .write =
                    [&chip = chips.imu, gyro = imu.gyro_resolution,
                     accel = imu.accel_resolution](std::span<const float> sample) {
                        chip.push_sample(
                            {static_cast<double>(at(sample, 0)) * gyro, static_cast<double>(at(sample, 1)) * gyro,
                             static_cast<double>(at(sample, 2)) * gyro},
                            {static_cast<double>(at(sample, 3)) * accel, static_cast<double>(at(sample, 4)) * accel,
                             static_cast<double>(at(sample, 5)) * accel}
                        );
                    },
            },
            context.noise
        ));

    hal::host::AdcPort& wall_adc = Board::adc(wall_sensors_config.adc.handle);
    wall_adc.bound = true;
    std::array<hal::host::PwmPort*, 4> emitters{};

    for (std::size_t sensor = 0; sensor < emitters.size(); sensor++) {
        emitters.at(sensor) = &pwm_port(wall_sensors_config.led_pwms.at(sensor));
    }

    hal::host::TimerBurstPort& burst = Board::timer_burst(wall_sensors_config.burst.handle);
    burst.bound = true;

    const double scan_period_us =
        1.0e6 / (static_cast<double>(nav::number_of_wall_sensors + 1) * static_cast<double>(wall_sensors_frequency));

    board.wall_sensors =
        add(context,
            std::make_unique<WallSensors>(
                context.world,
                WallSensors::Config{
                    .name = "wall",
                    .description = world.robot->wall_sensors,
                    .scan_ticks = 1,
                    .emitter_duty = [emitters](std::size_t sensor) { return emitters.at(sensor)->duty_cycle; },
                    .write = [&wall_adc](std::size_t index, uint32_t counts) { wall_adc.write(index, counts); },
                    .finish_sequence = [&wall_adc] { wall_adc.finish_sequence(); },
                    .reflectance = world.reflectance,
                    .minnaert = world.minnaert,
                    .schedule = make_emitter_schedule(burst),
                    .scan_period_us = scan_period_us,
                },
                context.noise
            ));

    hal::host::AdcPort& battery_adc = Board::adc(battery_config.adc.handle);
    battery_adc.bound = true;
    board.battery =
        add(context, std::make_unique<Battery>(
                         Battery::Config{
                             .name = "pack",
                             .description = world.robot->battery,
                             .divider = static_cast<double>(battery_config.voltage_divider),
                             .adc_reference = static_cast<double>(battery_config.adc.reference_voltage),
                             .adc_max_counts = static_cast<double>(battery_config.adc.max_reading),
                             .adc_noise_counts = 1.0,
                             .write = [&battery_adc](uint32_t counts) { battery_adc.write(0, counts); },
                         },
                         context.noise
                     ));

    hal::host::PwmPort&  fan_pwm = pwm_port(fan_config.pwm);
    hal::host::GpioPort& fan_enable = gpio_port(fan_config.enable_gpio);
    const Battery*       battery = board.battery;
    board.fan =
        add(context, std::make_unique<Fan>(
                         context.world, Fan::Config{
                                            .name = "fan",
                                            .actuator = names.fan,
                                            .description = world.robot->fan,
                                            .duty = [&fan_pwm] { return fan_pwm.duty_cycle; },
                                            .enabled = [&fan_enable] { return fan_enable.output; },
                                            .supply_voltage = [battery] { return battery->voltage(); },
                                        }
                     ));

    hal::host::AdcPort& torque_adc = Board::adc(torque_sensors_config.adc.handle);
    torque_adc.bound = true;
    const DcMotor* left = board.left_motor;
    const DcMotor* right = board.right_motor;
    add(context,
        std::make_unique<CurrentSense>(
            CurrentSense::Config{
                .currents = {[right] { return right->current(); }, [left] { return left->current(); }},
                .zero_voltage = static_cast<double>(
                    torque_sensors_config.zero_reading * torque_sensors_config.adc.reference_voltage
                ),
                .volts_per_amp = static_cast<double>(torque_sensors_config.shunt_resistor),
                .adc_reference = static_cast<double>(torque_sensors_config.adc.reference_voltage),
                .adc_max_counts = static_cast<double>(torque_sensors_config.adc.max_reading),
                .adc_noise_counts = 4.0,
                .write = [&torque_adc](std::size_t index, uint32_t counts) { torque_adc.write(index, counts); },
            },
            context.noise
        ));

    hal::host::UartPort& uart = Board::uart(bluetooth_config.handle);
    uart.bound = true;
    add(context, std::make_unique<SerialLink>(
                     context.serial, SerialLink::Config{
                                         .baud_rate = world.robot->link.baud_rate,
                                         .take_sent = [&uart]() -> std::optional<uint8_t> {
                                             if (uart.tx.empty()) {
                                                 return std::nullopt;
                                             }

                                             const uint8_t byte = uart.tx.front();
                                             uart.tx.pop_front();
                                             return byte;
                                         },
                                         .receive = [&uart](uint8_t byte) { uart.receive(byte); },
                                     }
                 ));

    board.button = bind_input(
        context, "button", button_config.gpio, button_config.pull_resistor == proxy::Button::PullResistor::PULL_UP
    );

    for (std::size_t index = 0; index < board.dip_switches.size(); index++) {
        board.dip_switches.at(index) = bind_input(
            context, std::string{"dip_"} + dip_names.at(index), dip_switch_config.gpio_array.at(index),
            dip_switch_config.active_low
        );
    }

    gpio_port(led_config.gpio);
    pwm_port(buzzer_config.pwm);
    Board::pwm_dma(argb_config.pwm.handle, argb_config.pwm.timer_channel).bound = true;
    Board::mcu().bound = true;
    Board::flash().bound = true;

    return board;
}
}  // namespace micras::sim
