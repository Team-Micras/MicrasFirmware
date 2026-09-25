/**
 * @file
 *
 * @brief Which host port drives which simulated device, for the Micras v1 board.
 */

#ifndef MICRAS_SIM_MICRAS_BINDINGS_HPP
#define MICRAS_SIM_MICRAS_BINDINGS_HPP

#include "micras/sim/app/target.hpp"
#include "micras/sim/core/run_context.hpp"
#include "micras/sim/micras/micras_target.hpp"

namespace micras::sim {
/**
 * @brief Build the board's devices and connect them to the firmware's ports.
 *
 * @note Every port is found through the firmware's own configuration in
 *       target.hpp, so a moved pin or channel moves the binding with it. Ports
 *       that only exist on the robot to talk to a chip the simulator replaces,
 *       the SPI chip selects, are marked bound without a device.
 *
 * @param context Context whose devices are added to.
 * @param world The robot and the arena.
 * @return Views of the devices the panel and the scenarios drive.
 */
MicrasBoard bind_devices(RunContext& context, const WorldInfo& world);
}  // namespace micras::sim

#endif  // MICRAS_SIM_MICRAS_BINDINGS_HPP
