project(micras_comm)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

add_library(micras::comm ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

# Only the reflection and the framing, so that the whole session layer can be built and tested
# without a single peripheral behind it
target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::core
)
