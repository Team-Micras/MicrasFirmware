# Name: linter.cmake
# Micras Team
# Brief: This file checks if linter was enabled and sets the proper flags
# 04/2023

###############################################################################
## Linter Check
###############################################################################

if(CMAKE_LINTER STREQUAL "ON")
    set(CMAKE_CXX_CLANG_TIDY "clang-tidy")
    add_compile_options(-fms-extensions)
    message(STATUS "Enabling clang-tidy")

elseif(CMAKE_LINTER STREQUAL "FIX")
    set(CMAKE_CXX_CLANG_TIDY "clang-tidy;--fix")
    add_compile_options(-fms-extensions)
    message(STATUS "Enabling clang-tidy with fix")

else()
    set(CMAKE_LINTER "OFF")
    message(STATUS "Linter is disabled")

endif()
