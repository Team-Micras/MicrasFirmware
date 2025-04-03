/**
 * @file
 */

#ifndef MICRAS_TEST_CORE_HPP
#define MICRAS_TEST_CORE_HPP

#include "target.hpp"

namespace micras {
template <typename F>
concept VoidFunction = requires(F void_function) {
    { void_function() } -> std::same_as<void>;
};

/**
 * @brief Core class to the tests.
 */
class TestCore {
public:
    /**
     * @brief Delete the default constructor.
     */
    TestCore() = delete;

    /**
     * @brief Initialize the test core.
     *
     * @param argc Number of main arguments.
     * @param argv Main arguments.
     */
    static void init(int /*argc*/ = 0, char** /*argv*/ = nullptr) { hal::Mcu::init(); }

    /**
     * @brief Loop the test core with a custom function.
     *
     * @param loop_func Custom loop function.
     */
    static void loop(VoidFunction auto loop_func) {
        while (true) {
            loop_func();
        }
    }
};
}  // namespace micras

#endif  // MICRAS_TEST_CORE_HPP
