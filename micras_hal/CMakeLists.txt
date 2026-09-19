project(micras_hal)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

add_library(micras::hal ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

# stm32cubemx is the interface target STM32CubeMX generates, carrying the CMSIS and vendor HAL
# include directories and the device macros. The public headers name them, so it propagates.
target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::core
    stm32cubemx
)
