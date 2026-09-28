###############################################################################
## Tests of the Micras board on the host backend: the host HAL over the Cube layer of cube/, the SPI
## device slot on the Micras buses and the firmware's SPI chip proxies over the chip models
###############################################################################

include(FetchContent)

# doctest v2.5.3, the commit micras-lib and the simulator test with. SYSTEM keeps the project warnings
# out of its header.
set(DOCTEST_NO_INSTALL ON)

FetchContent_Declare(doctest
    URL      "https://github.com/doctest/doctest/archive/2d0a9359a60c51affe2a9bebb1be1dca47868151.tar.gz"
    URL_HASH "SHA256=e64542c4ea68e9f381ccf6eae924cfdd652567c87c142d76fe92644fb4608149"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM
)
FetchContent_MakeAvailable(doctest)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

add_executable(micras_sim_tests
    unit/host_hal_test.cpp
    unit/spi_chips_test.cpp
    unit/spi_slot_test.cpp
)

target_link_libraries(micras_sim_tests PRIVATE
    micras_sim_micras
    doctest::doctest_with_main
)

micras_apply_warnings(micras_sim_tests WERROR ${MICRAS_WERROR})

doctest_discover_tests(micras_sim_tests)
