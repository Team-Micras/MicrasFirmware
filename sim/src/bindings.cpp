/**
 * @file
 */

#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#include "constants.hpp"
#include "micras/hal/host/board.hpp"
#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/micras/bindings.hpp"
#include "micras/sim/robot/robot_model.hpp"
#include "target.hpp"

namespace micras::sim {
namespace {
using hal::host::Board;

/**
 * @brief Add a device to the run and keep a view of it.
 *
 * @param context Context whose devices are added to.
 * @param device The device.
 * @return The view.
 */
template <typename T>
T* add(RunContext& context, std::unique_ptr<T> device) {
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
hal::host::PwmPort& pwm_port(const hal::Pwm::Config& config) {
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
hal::host::GpioPort& gpio_port(const hal::Gpio::Config& config) {
    hal::host::GpioPort& port = Board::gpio(config.port, config.pin);
    port.bound = true;
    return port;
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
DcMotor* bind_motor(
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
 * @brief Build one wheel's encoder.
 *
 * @param context Context whose devices are added to.
 * @param world The robot.
 * @param sensor The firmware's configuration of the rotary sensor.
 * @param side "left" or "right".
 */
void bind_encoder(
    RunContext& context, const WorldInfo& world, const proxy::RotarySensor::Config& sensor, const std::string& side
) {
    const RobotModelNames   names = RobotModelNames::of(*world.robot);
    hal::host::EncoderPort& port = Board::encoder(sensor.encoder.handle);
    port.bound = true;
    gpio_port(sensor.spi.cs_gpio);

    add(context, std::make_unique<QuadratureEncoder>(
                     context.world,
                     QuadratureEncoder::Config{
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
DigitalInput* bind_input(RunContext& context, const std::string& name, const hal::Gpio::Config& gpio, bool active_low) {
    hal::host::GpioPort& port = gpio_port(gpio);

    return add(
        context, std::make_unique<DigitalInput>(DigitalInput::Config{
                     .name = name,
                     .active_low = active_low,
                     .drive = [&port](bool level) { port.input = level; },
                 })
    );
}
}  // namespace

MicrasBoard bind_devices(RunContext& context, const WorldInfo& world) {
    const RobotModelNames names = RobotModelNames::of(*world.robot);
    MicrasBoard           board;

    board.left_motor = bind_motor(context, world, locomotion_config.left_motor, "left");
    board.right_motor = bind_motor(context, world, locomotion_config.right_motor, "right");
    bind_encoder(context, world, rotary_sensor_left_config, "left");
    bind_encoder(context, world, rotary_sensor_right_config, "right");

    hal::host::SamplePort& imu = Board::samples("imu");
    imu.bound = true;
    gpio_port(imu_config.spi.cs_gpio);

    add(context, std::make_unique<Imu>(
                     context.world,
                     Imu::Config{
                         .name = "lsm6dsv",
                         .gyro = names.gyro,
                         .accelerometer = names.accelerometer,
                         .description = world.robot->imu,
                         .write =
                             [&imu](std::span<const float> sample) {
                                 std::ranges::copy(sample, imu.values.begin());
                                 imu.sequence++;
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

    const double scan_period = 1.0 / (2.0 * wall_sensors_frequency);

    board.wall_sensors = add(
        context,
        std::make_unique<WallSensors>(
            context.world,
            WallSensors::Config{
                .name = "wall",
                .description = world.robot->wall_sensors,
                .scan_ticks = static_cast<uint32_t>(std::lround(scan_period / (context.clock.us_per_tick() * 1e-6))),
                .emitter_duty = [emitters](std::size_t sensor) { return emitters.at(sensor)->duty_cycle; },
                .write = [&wall_adc](std::size_t index, uint32_t counts) { wall_adc.write(index, counts); },
                .finish_sequence = [&wall_adc] { wall_adc.finish_sequence(); },
                .reflectance = world.reflectance,
                .robot_group = RobotModelNames::robot_group,
                .paint_group = Maze::paint_group,
            },
            context.noise
        )
    );

    hal::host::AdcPort& battery_adc = Board::adc(battery_config.adc.handle);
    battery_adc.bound = true;
    board.battery =
        add(context, std::make_unique<Battery>(
                         Battery::Config{
                             .name = "pack",
                             .description = world.robot->battery,
                             .divider = battery_config.voltage_divider,
                             .adc_reference = battery_config.adc.reference_voltage,
                             .adc_max_counts = static_cast<double>(battery_config.adc.max_reading),
                             .adc_noise_counts = 1.0,
                             .load_current = nullptr,
                             .write = [&battery_adc](uint32_t counts) { battery_adc.write(0, counts); },
                         },
                         context.noise
                     ));

    hal::host::PwmPort&  fan_pwm = pwm_port(fan_config.pwm);
    hal::host::GpioPort& fan_enable = gpio_port(fan_config.enable_gpio);
    Battery*             battery = board.battery;
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
    DcMotor* left = board.left_motor;
    DcMotor* right = board.right_motor;
    add(context,
        std::make_unique<CurrentSense>(
            CurrentSense::Config{
                .currents = {[left] { return left->current(); }, [right] { return right->current(); }},
                .volts_per_amp = torque_sensors_config.shunt_resistor,
                .adc_reference = torque_sensors_config.adc.reference_voltage,
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
            context, "dip_" + std::to_string(index), dip_switch_config.gpio_array.at(index),
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
