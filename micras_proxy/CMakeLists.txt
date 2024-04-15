project(micras_proxy)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")
file(GLOB_RECURSE PROJECT_LIBS_SOURCES CONFIGURE_DEPENDS "libs/*.c*")

# Remove warnings from libraries sources
set_source_files_properties(
    ${PROJECT_LIBS_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
    ${PROJECT_LIBS_SOURCES}
)

add_library(micras::proxy ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    libs/lsm6dsv-pid
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::hal
)
