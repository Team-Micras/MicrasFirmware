if(ENABLE_LINTER OR ENABLE_LINTER_FIX)
    set(CMAKE_CXX_CLANG_TIDY "clang-tidy")
    add_compile_options(-fms-extensions)

    if(ENABLE_LINTER)
        message(STATUS "Enabling clang-tidy")
        set(ENABLE_LINTER_FIX OFF)

    else(ENABLE_LINTER)
        message(STATUS "Enabling clang-tidy with fix")
        set(CMAKE_CXX_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY};--fix")

    endif(ENABLE_LINTER)

endif()
