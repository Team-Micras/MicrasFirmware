/**
 * @file
 */

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "constants.hpp"
#include "micras/comm/frame.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/hal/host/board.hpp"
#include "micras/hal/host/clock.hpp"
#include "micras/hal/host/ports.hpp"
#include "micras/micras.hpp"
#include "micras/sim/app/wiring.hpp"
#include "micras/sim/core/firmware_thread.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/micras/bindings.hpp"
#include "micras/sim/micras/micras_target.hpp"
#include "micras/sim/micras/pool_variables.hpp"
#include "micras/sim/recording/ground_truth.hpp"
#include "micras/sim/recording/run_metadata.hpp"
#include "micras/sim/scenario/scenario.hpp"
#include "micras/sim/view/panel_spec.hpp"
#include "micras/states/base.hpp"
#include "micras_firmware_sha.hpp"
#include "target.hpp"

/**
 * @brief The firmware's own main, renamed at compile time.
 *
 * @return What main returns on the robot, never reached.
 */
extern int micras_firmware_main();

namespace micras::sim {
namespace {
/**
 * @brief Names of the states of the firmware's state machine, indexed as micras::State.
 *
 * @note The firmware has no names for its states, so they are written here, and the build checks
 *       that there is one for each.
 */
constexpr auto state_name_table = std::to_array<std::string_view>({
    "INIT",
    "IDLE",
    "WAIT_FOR_RUN",
    "RUN",
    "PLAN",
    "SAVE",
    "WAIT_FOR_CALIBRATE",
    "CALIBRATE",
    "WAIT_FOR_IDENTIFY",
    "IDENTIFY",
    "WAIT_FOR_GYROSCOPE",
    "CALIBRATE_GYROSCOPE",
    "ERROR",
});

static_assert(
    state_name_table.size() == std::to_underlying(State::NUMBER_OF_STATES), "every firmware state needs a name"
);

/**
 * @brief Link commands a scenario sends by name, with their codes in Micras::Command.
 */
constexpr std::array<std::pair<const char*, Micras::Command>, 5> commands{{
    {"explore", Micras::Command::EXPLORE},
    {"solve", Micras::Command::SOLVE},
    {"calibrate", Micras::Command::CALIBRATE},
    {"save", Micras::Command::SAVE},
    {"reset", Micras::Command::RESET},
}};
}  // namespace

/**
 * @brief Get the names of the states, as the panel, the overlay and the scenarios take them.
 *
 * @return The names, in the order of State.
 */
static const std::vector<std::string>& state_names() {
    static const std::vector<std::string> names{state_name_table.begin(), state_name_table.end()};
    return names;
}

/**
 * @brief Encode a link command, as micras-monitor sends it.
 *
 * @param command The command.
 * @return The frame's bytes.
 */
static std::vector<uint8_t> command_frame(Micras::Command command) {
    std::array<uint8_t, 5> payload{};
    comm::Writer           writer{payload};
    writer.u8(std::to_underlying(command));
    writer.u32(0);

    std::array<uint8_t, comm::max_frame_size> frame{};
    const std::size_t size = comm::encode_frame(comm::MessageType::COMMAND, writer.done(), frame);
    return {frame.begin(), std::next(frame.begin(), static_cast<std::ptrdiff_t>(size))};
}

/**
 * @brief Decode the colors of addressable LEDs from the compare values of their last transfer.
 *
 * @param port The DMA-fed timer channel.
 * @param count Number of LEDs.
 * @return Their colors, green-red-blue on the wire, as the panel shows them.
 */
static std::vector<Color> decode_argb(const hal::host::PwmDmaPort& port, std::size_t count) {
    constexpr std::size_t bits_per_led{24};
    std::vector<Color>    colors(count);

    for (std::size_t led = 0; led < count; led++) {
        uint32_t data = 0;

        for (std::size_t bit = 0; bit < bits_per_led; bit++) {
            const std::size_t index = led * bits_per_led + bit;
            const bool        high = index < port.compares.size() and 2 * port.compares.at(index) > port.period;
            data = (data << 1U) | static_cast<uint32_t>(high);
        }

        colors.at(led) = {
            .red = static_cast<uint8_t>(data >> 8U),
            .green = static_cast<uint8_t>(data >> 16U),
            .blue = static_cast<uint8_t>(data)
        };
    }

    return colors;
}

std::string MicrasTarget::name() const {
    return "micras";
}

std::string MicrasTarget::firmware_sha() const {
    return MICRAS_FIRMWARE_SHA;
}

uint32_t MicrasTarget::loop_time_us() const {
    return micras::loop_time_us;
}

std::vector<CliOption> MicrasTarget::options() {
    return {{
        .name = "--flash",
        .argument = "<file>",
        .apply = [this](const std::string& value) { this->flash_file = value; },
    }};
}

RunContext& MicrasTarget::context() {
    return this->run_context;
}

std::filesystem::path MicrasTarget::directory() const {
    return std::filesystem::path{MICRAS_TARGET_DIR};
}

std::filesystem::path MicrasTarget::robot_file() const {
    return this->directory() / "robot.toml";
}

GroundTruthConfig MicrasTarget::ground_truth() const {
    return {
        .body = "micras",
        .columns = {
            {.name = "wheel_angle_left", .probe = Probe::JOINT_POSITION, .object = "left_wheel"},
            {.name = "wheel_angle_right", .probe = Probe::JOINT_POSITION, .object = "right_wheel"},
            {.name = "wheel_speed_left", .probe = Probe::JOINT_VELOCITY, .object = "left_wheel"},
            {.name = "wheel_speed_right", .probe = Probe::JOINT_VELOCITY, .object = "right_wheel"},
            {.name = "motor_torque_left", .probe = Probe::ACTUATOR_FORCE, .object = "motor_left"},
            {.name = "motor_torque_right", .probe = Probe::ACTUATOR_FORCE, .object = "motor_right"},
            {.name = "left_ncon", .probe = Probe::CONTACT_COUNT, .object = "left_wheel"},
            {.name = "left_fn", .probe = Probe::CONTACT_NORMAL_FORCE, .object = "left_wheel"},
            {.name = "left_slip", .probe = Probe::CONTACT_SLIP, .object = "left_wheel"},
            {.name = "left_penetration", .probe = Probe::CONTACT_PENETRATION, .object = "left_wheel"},
            {.name = "right_ncon", .probe = Probe::CONTACT_COUNT, .object = "right_wheel"},
            {.name = "right_fn", .probe = Probe::CONTACT_NORMAL_FORCE, .object = "right_wheel"},
            {.name = "right_slip", .probe = Probe::CONTACT_SLIP, .object = "right_wheel"},
            {.name = "right_penetration", .probe = Probe::CONTACT_PENETRATION, .object = "right_wheel"},
            {.name = "board_ncon", .probe = Probe::CONTACT_COUNT, .object = "board"},
            {.name = "rear_skid_fn", .probe = Probe::CONTACT_NORMAL_FORCE, .object = "rear_skid"},
            {.name = "front_skid_fn", .probe = Probe::CONTACT_NORMAL_FORCE, .object = "front_skid"},
            {.name = "solver_niter", .probe = Probe::SOLVER_ITERATIONS, .object = ""},
        },
    };
}

std::string MicrasTarget::video_camera() const {
    return "side tracking";
}

FirmwareThread::Program MicrasTarget::program() {
    return [](FirmwareThread&) { micras_firmware_main(); };
}

Wiring MicrasTarget::wire(FirmwareThread& firmware, const WorldInfo& world) {
    hal::host::Board::reset();

    if (not this->flash_file.empty() and std::filesystem::exists(this->flash_file)) {
        std::ifstream file(this->flash_file, std::ios::binary);
        hal::host::Board::flash().bytes.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    }

    hal::host::Clock& clock = hal::host::Clock::instance();
    clock.reset();
    clock.configure(SystemCoreClock / 1000000);
    clock.set_handover(this->run_context.clock.us_per_tick(), [&firmware] { firmware.yield_tick(); });

    this->board = bind_devices(this->run_context, world, this->chips);
    this->variables = std::make_unique<PoolVariables>();

    return {
        .columns = {this->variables.get()},
        .variables = this->variables.get(),
        .panel = this->make_panel(),
        .overlay =
            {.state = StateLabel{.variable = "state", .names = state_names()},
             .lines =
                 {{.label = "v reference", .variable = "reference/linear_speed", .unit = "m/s"},
                  {.label = "v estimate", .variable = "pose/linear_speed", .unit = "m/s"}}},
        .hooks = this->make_hooks(),
    };
}

void MicrasTarget::unwire() {
    hal::host::Clock::instance().clear_handover();

    for (const std::string& port : hal::host::Board::unbound()) {
        std::cerr << "warning: the firmware used " << port << ", which nothing in the simulator is bound to\n";
    }

    if (not this->flash_file.empty()) {
        const std::vector<uint8_t>& bytes = hal::host::Board::flash().bytes;
        std::ofstream(this->flash_file, std::ios::binary)
            .write(std::bit_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }

    hal::host::Board::reset();
}

std::vector<MetadataField> MicrasTarget::metadata() const {
    const hal::host::McuPort& mcu = hal::host::Board::mcu();

    return {
        {.name = "unbound_ports", .value = static_cast<int64_t>(hal::host::Board::unbound().size())},
        {.name = "watchdog_expiries", .value = mcu.watchdog_expiries},
        {.name = "emergency_stops", .value = mcu.emergency_stops},
    };
}

PanelSpec MicrasTarget::make_panel() const {
    const MicrasBoard& devices = this->board;
    PanelSpec          panel{
        .state = StateLabel{.variable = "state", .names = state_names()},
        .buttons = {{.name = "button", .press = [&devices](bool pressed) { devices.button->set(pressed); }}},
        .switches = {},
        .lamps = {},
        .readouts = {},
        .plots = {"reference/linear_speed", "pose/linear_speed", "reference/angular_speed", "pose/angular_speed"},
        .take_over = {},
    };

    for (std::size_t index = 0; index < devices.dip_switches.size(); index++) {
        DigitalInput* input = devices.dip_switches.at(index);
        panel.switches.push_back(
            {.name = dip_names.at(index),
             .state = [input] { return input->is_active(); },
             .set = [input](bool on) { input->set(on); }}
        );
    }

    const hal::host::GpioPort& led = hal::host::Board::gpio(led_config.gpio.port, led_config.gpio.pin);
    panel.lamps.push_back({.name = "led", .color = [&led] {
                               return led.output ? Color{.red = 255, .green = 40, .blue = 40} :
                                                   Color{.red = 40, .green = 40, .blue = 40};
                           }});

    const hal::host::PwmDmaPort& argb =
        hal::host::Board::pwm_dma(argb_config.pwm.handle, argb_config.pwm.timer_channel);

    for (std::size_t index = 0; index < 2; index++) {
        panel.lamps.push_back({.name = std::format("argb {}", index), .color = [&argb, index] {
                                   return decode_argb(argb, 2).at(index);
                               }});
    }

    const hal::host::PwmPort& buzzer = hal::host::Board::pwm(buzzer_config.pwm.handle, buzzer_config.pwm.timer_channel);
    panel.readouts.push_back({.name = "buzzer", .text = [&buzzer] {
                                  return buzzer.duty_cycle > 0.0F ? std::format("{:.0f} Hz", buzzer.frequency) : "off";
                              }});
    panel.readouts.push_back(
        {.name = "motors", .text = [&devices] {
             return std::format("{:6.2f} {:6.2f} A", devices.left_motor->current(), devices.right_motor->current());
         }}
    );
    panel.readouts.push_back({.name = "battery", .text = [&devices] {
                                  return std::format("{:.2f} V", devices.battery->voltage());
                              }});

    return panel;
}

ScenarioHooks MicrasTarget::make_hooks() const {
    ScenarioHooks hooks;
    hooks.inputs.emplace("button", this->board.button);

    for (std::size_t index = 0; index < this->board.dip_switches.size(); index++) {
        hooks.inputs.emplace(std::string{"dip_"} + dip_names.at(index), this->board.dip_switches.at(index));
    }

    for (const auto& [name, command] : commands) {
        hooks.messages.emplace(name, command_frame(command));
    }

    hooks.state_names.emplace("state", state_names());
    return hooks;
}
}  // namespace micras::sim
