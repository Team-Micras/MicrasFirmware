project(micras_proxy)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")
file(GLOB_RECURSE PROJECT_LIB_SOURCES CONFIGURE_DEPENDS "lib/*.c*")

# Remove warnings from lib sources
set_source_files_properties(
    ${PROJECT_LIB_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

add_library(${PROJECT_NAME} STATIC)
add_library(micras::proxy ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    lib/lsm6dsv-pid
)

target_sources(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCES}
    ${PROJECT_LIB_SOURCES}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::hal
)
