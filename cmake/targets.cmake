###############################################################################
## Auxiliary Targets
###############################################################################

add_custom_target(helpme
    COMMAND cat ${CMAKE_CURRENT_BINARY_DIR}/helpme
)

add_custom_target(cube
    COMMAND echo "Generating cube files..."

    COMMAND echo "config load ${CUBE_SOURCE_DIR}/${PROJECT_RELEASE}.ioc" > ${CMAKE_CURRENT_BINARY_DIR}/cube_script.txt
    COMMAND echo "project generate" >> ${CMAKE_CURRENT_BINARY_DIR}/cube_script.txt
    COMMAND echo "exit" >> ${CMAKE_CURRENT_BINARY_DIR}/cube_script.txt

    COMMAND ${CUBE_CMD} -q ${CMAKE_CURRENT_BINARY_DIR}/cube_script.txt
)

add_custom_target(info
    COMMAND ${PROGRAMMER_CMD} -c port=SWD
)

add_custom_target(reset
    COMMAND echo "Resetting device"
    COMMAND ${PROGRAMMER_CMD} -c port=SWD -rst
)

add_custom_target(clear
    COMMAND echo "Cleaning all build files..."
    COMMAND rm -rf ${CMAKE_CURRENT_BINARY_DIR}/*
)

add_custom_target(clear_cube
    COMMAND echo "Cleaning cube files..."
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/cube/*.ioc .
    COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/cube
    COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/cube
    COMMAND mv *.ioc ${CMAKE_CURRENT_SOURCE_DIR}/cube/
)

add_custom_target(clear_all
    COMMAND ${CMAKE_MAKE_PROGRAM} clear_cube
    COMMAND echo "Cleaning all build files..."
    COMMAND rm -rf ${CMAKE_CURRENT_BINARY_DIR}/*
)

add_custom_target(rebuild
    COMMAND ${CMAKE_MAKE_PROGRAM} clear
    COMMAND cmake ..
    COMMAND ${CMAKE_MAKE_PROGRAM}
)

add_custom_target(rebuild_all
    COMMAND ${CMAKE_MAKE_PROGRAM} clear_all
    COMMAND cmake ..
    COMMAND ${CMAKE_MAKE_PROGRAM}
)

add_custom_target(docs
    COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && doxygen Doxyfile
    COMMAND yes Q | make -C ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex || true
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex/refman.pdf ${CMAKE_CURRENT_SOURCE_DIR}/docs/
    COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex
)

function(generate_test_all_target)
    foreach(FILE ${ARGV})
        get_filename_component(TEST_NAME ${FILE} NAME_WLE)
        list(APPEND TEST_TARGETS ${TEST_NAME})
    endforeach()

    add_custom_target(test_all
        COMMAND ${CMAKE_MAKE_PROGRAM} ${TEST_TARGETS}
    )
endfunction()

function(generate_format_target)
    foreach(FILE ${ARGV})
        list(APPEND FILES_LIST ${${FILE}})
    endforeach()

    add_custom_target(format
        COMMAND clang-format -style=file -i ${FILES_LIST} --verbose
    )
endfunction()

function(generate_lint_target)
    foreach(FILE ${ARGV})
        list(APPEND FILES_LIST ${${FILE}})
    endforeach()

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

    set(SCRIPT_SAVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/run_clang_tidy.sh")
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/run_clang_tidy.sh.in
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

# Flash via st-link or jlink
function(generate_flash_target TARGET)
    if("${TARGET}" STREQUAL "${PROJECT_NAME}")
        set(TARGET_SUFFIX "")
    else()
        set(TARGET_SUFFIX "_${TARGET}")
    endif()

    add_custom_target(flash${TARGET_SUFFIX}
        COMMAND echo "Flashing..."
        COMMAND ${PROGRAMMER_CMD} -c port=SWD -w ${TARGET}.hex -v -rst
    )

    add_dependencies(flash${TARGET_SUFFIX} ${TARGET})
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/jlink.in
        ${CMAKE_CURRENT_BINARY_DIR}/jlinkflash/.jlink-flash${TARGET_SUFFIX}
    )

    add_custom_target(jflash${TARGET_SUFFIX}
        COMMAND echo "Flashing ${PROJECT_NAME}.hex with J-Link"
        COMMAND ${JLINK_CMD} ${CMAKE_CURRENT_BINARY_DIR}/jlinkflash/.jlink-flash${TARGET_SUFFIX}
    )

    add_dependencies(jflash${TARGET_SUFFIX} ${TARGET})
endfunction()

function(generate_debug_target TARGET)
    if("${TARGET}" STREQUAL "${PROJECT_NAME}")
        set(TARGET_SUFFIX "")
    else()
        set(TARGET_SUFFIX "_${TARGET}")
    endif()

    set(DEBUG_FILE_NAME ${TARGET})
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/launch.json.in
        ${CMAKE_CURRENT_BINARY_DIR}/vsfiles/.vsfiles${TARGET_SUFFIX}
    )

    add_custom_target(debug${TARGET_SUFFIX}
        COMMAND echo "Configuring VS Code files for ${TARGET}"
        COMMAND cat ${output_save_file} > ${LAUNCH_JSON_PATH}
    )

    add_dependencies(debug${TARGET_SUFFIX} ${TARGET})
endfunction()
