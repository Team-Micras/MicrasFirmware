###############################################################################
## Micras target unit tests: the host micras_hal and the fake Cube layer
###############################################################################

include(GoogleTest)

add_executable(micras_target_tests
    unit/hal_host_test.cpp
)

target_link_libraries(micras_target_tests PRIVATE micras_sim_micras GTest::gtest_main)
target_compile_options(micras_target_tests PRIVATE -Wall -Wextra)

gtest_discover_tests(micras_target_tests)
