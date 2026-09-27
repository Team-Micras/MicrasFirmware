/**
 * @file
 */

#ifndef MICRAS_TEST_HOST_HPP
#define MICRAS_TEST_HOST_HPP

#include <cstdio>
#include <cstdlib>

/**
 * @brief Fail the test with the condition and where it was written.
 *
 * @note Spelled out instead of using assert so that it holds whatever the build turns NDEBUG into.
 */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage) it has to capture the text and the line
#define CHECK(condition)                                                         \
    do {                                                                         \
        if (not(condition)) {                                                    \
            std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
            std::abort();                                                        \
        }                                                                        \
    } while (false)

#endif  // MICRAS_TEST_HOST_HPP
