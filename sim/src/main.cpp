/**
 * @file
 *
 * @brief Runs the Micras firmware, its own main, in the simulator.
 */

#include <span>

#include "micras/sim/app/application.hpp"
#include "micras/sim/micras/micras_target.hpp"

int main(int argc, char** argv) {
    micras::sim::MicrasTarget target;
    return micras::sim::run(std::span(argv, static_cast<std::size_t>(argc)), target);
}
