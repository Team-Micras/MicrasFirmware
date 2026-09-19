project(micras_core)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
)

add_library(micras::core ALIAS ${PROJECT_NAME})

# C++23 is a requirement of the sources, not a preference of whoever is building, so it is declared
# here and propagated
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)
