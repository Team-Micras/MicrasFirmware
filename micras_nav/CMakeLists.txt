project(micras_nav)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

add_library(micras::nav ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

# The navigation takes plain data and gives plain data back, so it links nothing that can reach the
# hardware and builds anywhere micras::core does
target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::core
)
