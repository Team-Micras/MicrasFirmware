###############################################################################
## Generated tree
###############################################################################

# Workaround for the STM32CubeMX CMake generator writing into ${CMAKE_PROJECT_NAME}
# instead of a target of its own (present since 6.14, still present in 6.18.1)
add_library(cube_app OBJECT)
set(CUBE_PROJECT_NAME ${CMAKE_PROJECT_NAME})
set(CMAKE_PROJECT_NAME cube_app)
add_subdirectory(cube/cmake/stm32cubemx)
set(CMAKE_PROJECT_NAME ${CUBE_PROJECT_NAME})

# The generated headers are not ours to warn about either, and stm32cubemx is a normal interface
# library, so its include directories are not system by default the way an imported target's would
# be. Without this, -Wuseless-cast fires inside stm32h7xx_ll_adc.h.
set_target_properties(stm32cubemx PROPERTIES SYSTEM TRUE)

# The generated objects link into the executables and not into a package, since no package refers
# to a symbol in them. Routing them through micras_hal is what made its archive need
# --whole-archive to keep the interrupt handlers, and what broke link time optimisation.
set(MICRAS_CUBE_OBJECT_LIBRARIES cube_app STM32_Drivers)

# The generated sources are not ours to warn about
foreach(CUBE_OBJECT_LIBRARY ${MICRAS_CUBE_OBJECT_LIBRARIES})
    target_compile_options(${CUBE_OBJECT_LIBRARY} PRIVATE -w)
endforeach()
