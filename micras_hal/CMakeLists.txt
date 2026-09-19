project(micras_hal)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

# The object libraries have to be linked directly, since CMake only adds an object library's
# files to the targets that name it, never through an interface target
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${MICRAS_CUBE_OBJECT_LIBRARIES}
    micras::warnings
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::core
    micras::cube
)

add_library(micras_hal_whole_archive INTERFACE)
add_library(micras::hal ALIAS micras_hal_whole_archive)
target_link_libraries(micras_hal_whole_archive INTERFACE
    "-Wl,--whole-archive" ${PROJECT_NAME} "-Wl,--no-whole-archive"
)
