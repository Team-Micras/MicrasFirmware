project(micras_comm)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

add_library(micras::comm ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::proxy
)
