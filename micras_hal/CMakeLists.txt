project(micras_hal)

get_target_property(CUBE_INCLUDE_DIRECTORIES stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(CUBE_SOURCES stm32cubemx INTERFACE_SOURCES)
get_target_property(CUBE_COMPILE_DEFINITIONS stm32cubemx INTERFACE_COMPILE_DEFINITIONS)

# Remove warnings from Cube sources
set_source_files_properties(
    ${CUBE_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    stm32cubemx
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${CUBE_INCLUDE_DIRECTORIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    ${CUBE_COMPILE_DEFINITIONS}
)

add_library(micras_hal_whole_archive INTERFACE)
add_library(micras::hal ALIAS micras_hal_whole_archive)
target_link_libraries(micras_hal_whole_archive INTERFACE
    "-Wl,--whole-archive" ${PROJECT_NAME} "-Wl,--no-whole-archive"
)
