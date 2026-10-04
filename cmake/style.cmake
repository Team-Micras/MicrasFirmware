###############################################################################
## Format and lint targets, for the robot build and the simulation's
###############################################################################

# The clang tools are found by their versioned names, so that a distribution update or another
# version first on the PATH can move neither the formatting rules nor the enabled check set
set(MICRAS_CLANG_VERSION 22)

foreach(TOOL clang-format clang-tidy run-clang-tidy clang-apply-replacements)
    string(TOUPPER ${TOOL} TOOL_VARIABLE)
    string(REPLACE "-" "_" TOOL_VARIABLE ${TOOL_VARIABLE})

    find_program(MICRAS_${TOOL_VARIABLE} ${TOOL}-${MICRAS_CLANG_VERSION})

    if(NOT MICRAS_${TOOL_VARIABLE})
        message(FATAL_ERROR
            "${TOOL}-${MICRAS_CLANG_VERSION} was not found. The format and lint targets need clang "
            "${MICRAS_CLANG_VERSION} (apt install clang-format-${MICRAS_CLANG_VERSION} "
            "clang-tidy-${MICRAS_CLANG_VERSION}), found by that name.")
    endif()
endforeach()

function(generate_format_target)
    foreach(FILE ${ARGV})
        list(APPEND FILES_LIST ${${FILE}})
    endforeach()

    add_custom_target(format
        COMMAND ${MICRAS_CLANG_FORMAT} -style=file -i ${FILES_LIST} --verbose
    )

    add_custom_target(format_check
        COMMAND ${MICRAS_CLANG_FORMAT} -style=file --dry-run --Werror ${FILES_LIST}
    )
endfunction()

# The sources are checked against the compilation database of the build that defines the target: the
# robot's lints the firmware compiled for the ARM core, the simulation's the sources of sim/
function(generate_lint_target)
    foreach(FILE ${ARGV})
        list(APPEND FILES_LIST ${${FILE}})
    endforeach()

    # clang does not know where the ARM toolchain keeps its sysroot and C++ library; on the host it
    # finds the compiler's own
    set(TIDY_EXTRA_ARGS "")

    if(CMAKE_CROSSCOMPILING)
        execute_process(
            COMMAND ${CMAKE_CXX_COMPILER} -print-search-dirs
            OUTPUT_VARIABLE _SEARCH_DIRS
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        string(REGEX MATCH "install: ([^\n]+)/" _ ${_SEARCH_DIRS})
        set(COMPILER_INSTALL_DIR "${CMAKE_MATCH_1}")
        get_filename_component(COMPILER_VERSION "${COMPILER_INSTALL_DIR}" NAME)

        execute_process(
            COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine
            OUTPUT_VARIABLE TARGET_TRIPLE
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        get_filename_component(_PARENT3 "${COMPILER_INSTALL_DIR}/../../../" REALPATH)
        set(SYSROOT "${_PARENT3}/${TARGET_TRIPLE}")

        set(CXX_INCLUDE_DIR      "${SYSROOT}/include/c++/${COMPILER_VERSION}")
        set(CXX_TRIPLE_INCLUDE_DIR "${CXX_INCLUDE_DIR}/${TARGET_TRIPLE}")

        set(TIDY_EXTRA_ARGS
            --sysroot=${SYSROOT}/
            -I${CXX_INCLUDE_DIR}/
            -I${CXX_TRIPLE_INCLUDE_DIR}/
        )
    endif()

    list(JOIN TIDY_EXTRA_ARGS "\n" MICRAS_TIDY_EXTRA_ARGS)

    set(SCRIPT_SAVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/run_clang_tidy.sh")
    configure_file(
        ${PROJECT_SOURCE_DIR}/cmake/templates/run_clang_tidy.sh.in
        ${SCRIPT_SAVE_PATH}
        @ONLY
    )

    add_custom_target(lint
        COMMAND ${SCRIPT_SAVE_PATH} ${FILES_LIST}
    )

    add_custom_target(lint_fix
        COMMAND ${SCRIPT_SAVE_PATH} --fix ${FILES_LIST}
    )
endfunction()
