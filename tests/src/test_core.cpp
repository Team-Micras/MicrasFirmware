/**
 * @file
 */

#include "micras/hal/mcu.hpp"
#include "test_core.hpp"

namespace micras {
void TestCore::init(int /*argc*/, char** /*argv*/) {
    hal::Mcu::init();
}

void TestCore::loop(VoidFunction auto loop_func) {
    while (true) {
        loop_func();
    }
}
}  // namespace micras
