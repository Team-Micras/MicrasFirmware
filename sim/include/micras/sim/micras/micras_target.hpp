/**
 * @file
 *
 * @brief The Micras micromouse, as a robot target of the simulator.
 */

#ifndef MICRAS_SIM_MICRAS_MICRAS_TARGET_HPP
#define MICRAS_SIM_MICRAS_MICRAS_TARGET_HPP

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "micras/sim/app/target.hpp"
#include "micras/sim/devices/dc_motor.hpp"
#include "micras/sim/devices/digital_input.hpp"
#include "micras/sim/devices/imu.hpp"
#include "micras/sim/devices/power.hpp"
#include "micras/sim/devices/quadrature_encoder.hpp"
#include "micras/sim/devices/serial_link.hpp"
#include "micras/sim/devices/wall_sensors.hpp"
#include "micras/sim/micras/pool_variables.hpp"

namespace micras::sim {
/**
 * @brief The devices the bindings built, for the panel and the scenarios.
 *
 * @note The run's devices own them; these are views.
 */
struct MicrasBoard {
    DigitalInput*                button{nullptr};
    std::array<DigitalInput*, 4> dip_switches{};
    DcMotor*                     left_motor{nullptr};
    DcMotor*                     right_motor{nullptr};
    Battery*                     battery{nullptr};
    Fan*                         fan{nullptr};
    WallSensors*                 wall_sensors{nullptr};
};

/**
 * @brief Micras on the firmware's main branch, through the host micras_hal.
 *
 * @note The firmware runs as it does on the robot: its own main, its own proxies
 *       and its own configuration, over the host backend in hal_host/ and the
 *       fake Cube layer in cube/. Only the IMU and rotary sensor proxies are
 *       replaced, by proxy/. The host timer hands every step over to the world.
 */
class MicrasTarget : public Target {
public:
    /**
     * @brief Get the robot's name.
     *
     * @return "micras".
     */
    std::string name() const override;

    /**
     * @brief Get the commit of the firmware submodule this binary was built from.
     *
     * @return The commit, suffixed -dirty when the checkout had changes.
     */
    std::string firmware_sha() const override;

    /**
     * @brief Get the firmware's loop period.
     *
     * @return Period in microseconds, from the firmware's constants.
     */
    uint32_t loop_time_us() const override;

    /**
     * @brief Get the robot's own option: --flash, a file the flash is loaded from and saved to.
     *
     * @return The options.
     */
    std::vector<CliOption> options() override;

    /**
     * @brief Get the context the run advances.
     *
     * @return This target's context.
     */
    RunContext& context() override;

    /**
     * @brief Get the robot's physical description.
     *
     * @return Path of targets/micras/robot.toml.
     */
    std::filesystem::path robot_file() const override;

    /**
     * @brief Get the ground truth columns of a Micras run.
     *
     * @return The recorder configuration.
     */
    GroundTruthConfig ground_truth() const override;

    /**
     * @brief Get the camera that follows the robot.
     *
     * @return "side tracking".
     */
    std::string video_camera() const override;

    /**
     * @brief Get the firmware's own main.
     *
     * @return The program.
     */
    FirmwareThread::Program program() override;

    /**
     * @brief Bind the devices to the host ports, hand time over, and build the panel.
     *
     * @param firmware Thread the host timer hands every step over through.
     * @param world The robot and the arena.
     * @return What Micras adds to the run.
     */
    Wiring wire(FirmwareThread& firmware, const WorldInfo& world) override;

    /**
     * @brief Stop handing time over, name the ports the firmware used that nothing is bound to, and
     *        save the flash when --flash asked for it.
     */
    void unwire() override;

    /**
     * @brief Get what the run's board reports: unbound ports, watchdog expiries, emergency stops.
     *
     * @return The counters.
     */
    std::vector<MetadataField> metadata() const override;

private:
    /**
     * @brief Build the panel.
     *
     * @return What the panel shows.
     */
    PanelSpec make_panel() const;

    /**
     * @brief Build the scenario hooks: inputs, link commands, state names.
     *
     * @return The hooks.
     */
    ScenarioHooks make_hooks() const;

    RunContext                     run_context;
    std::filesystem::path          flash_file;
    MicrasBoard                    board;
    std::unique_ptr<PoolVariables> variables;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_MICRAS_MICRAS_TARGET_HPP
