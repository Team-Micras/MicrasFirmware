project(micras_proxy)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

include(FetchContent)
FetchContent_Declare(
    lsm6dsv-pid
    GIT_REPOSITORY https://github.com/STMicroelectronics/lsm6dsv-pid.git
    GIT_TAG v3.3.0
)

FetchContent_Populate(lsm6dsv-pid)
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "${lsm6dsv-pid_SOURCE_DIR}/*.c")

# Remove warnings from libraries sources
set_source_files_properties(
    ${LIB_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
    ${LIB_SOURCES}
)

add_library(micras::proxy ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${lsm6dsv-pid_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::hal
)
