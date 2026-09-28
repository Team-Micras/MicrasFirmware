###############################################################################
## Micras target unit tests: the host micras_hal, the fake Cube layer, the SPI device
## slot and the firmware's SPI chip proxies over the chip models
###############################################################################

add_executable(micras_target_tests
    unit/hal_host_test.cpp
    unit/spi_chips_test.cpp
    unit/spi_slot_test.cpp
)

target_link_libraries(micras_target_tests PRIVATE micras_sim_micras doctest::doctest_with_main)
micras_apply_warnings(micras_target_tests WERROR ${MICRAS_SIM_WERROR})

doctest_discover_tests(micras_target_tests)
