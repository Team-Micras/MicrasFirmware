###############################################################################
## Generated tree
###############################################################################

# The STM32CubeMX CMake generator writes its sources into ${CMAKE_PROJECT_NAME} instead of a
# target of its own, so the variable is pointed at a target created to hold them
add_library(cube_app OBJECT)
set(CUBE_PROJECT_NAME ${CMAKE_PROJECT_NAME})
set(CMAKE_PROJECT_NAME cube_app)
add_subdirectory(cube/cmake/stm32cubemx)
set(CMAKE_PROJECT_NAME ${CUBE_PROJECT_NAME})

# A plain interface library's include directories are not system by default, which would apply
# the project's warning flags to the generated headers
set_target_properties(stm32cubemx PROPERTIES SYSTEM TRUE)

# The vector table, the interrupt handlers and the MX_*_Init functions belong to the application,
# so the objects carrying them are linked into the executables rather than into a package
set(MICRAS_CUBE_OBJECT_LIBRARIES cube_app STM32_Drivers)

# The generated sources are not ours to warn about
foreach(CUBE_OBJECT_LIBRARY ${MICRAS_CUBE_OBJECT_LIBRARIES})
    target_compile_options(${CUBE_OBJECT_LIBRARY} PRIVATE -w)
endforeach()
