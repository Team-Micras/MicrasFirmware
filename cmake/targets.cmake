# Name: targets.cmake
# ThundeRatz Robotics Team
# Brief: This file contains auxiliary custom targets declarations
# 04/2023

###############################################################################
## Auxiliary Targets
###############################################################################

add_custom_target(helpme
    COMMAND cat ${CMAKE_CURRENT_BINARY_DIR}/.helpme
)

add_custom_target(cube
    COMMAND echo "Generating cube files..."

    COMMAND echo "config load ${CMAKE_CURRENT_SOURCE_DIR}/cube/${PROJECT_RELEASE}.ioc" > ${CMAKE_CURRENT_BINARY_DIR}/.cube
    COMMAND echo "project generate" >> ${CMAKE_CURRENT_BINARY_DIR}/.cube
    COMMAND echo "exit" >> ${CMAKE_CURRENT_BINARY_DIR}/.cube

    COMMAND ${JAVA_EXE} -jar ${CUBE_JAR} -q ${CMAKE_CURRENT_BINARY_DIR}/.cube
)

add_custom_target(info
    STM32_Programmer_CLI -c port=SWD
)

add_custom_target(reset
    COMMAND echo "Reseting device"
    COMMAND STM32_Programmer_CLI -c port=SWD -rst
)

add_custom_target(clean_all
    COMMAND echo "Cleaning all build files..."
    COMMAND rm -rf ${CMAKE_CURRENT_BINARY_DIR}/*
)

add_custom_target(clean_cube
    COMMAND echo "Cleaning cube files..."
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/cube/*.ioc .
    COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/cube/*
    COMMAND mv *.ioc ${CMAKE_CURRENT_SOURCE_DIR}/cube/
)

add_custom_target(rebuild
    COMMAND ${CMAKE_MAKE_PROGRAM} clean_all
    COMMAND cmake ..
    COMMAND ${CMAKE_MAKE_PROGRAM}
)

add_custom_target(rebuild_all
    COMMAND ${CMAKE_MAKE_PROGRAM} clean_cube
    COMMAND ${CMAKE_MAKE_PROGRAM} clean_all
    COMMAND cmake ..
    COMMAND ${CMAKE_MAKE_PROGRAM}
)

add_custom_target(docs
    COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && doxygen Doxyfile
    COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex/refman.pdf ${CMAKE_CURRENT_SOURCE_DIR}/docs/
    COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex
)

function(targets_generate_format_target)
    set(FILES_LIST "")
    foreach(FILE ${ARGV})
        list(APPEND FILES_LIST ${${FILE}})
    endforeach()
    add_custom_target(format
        COMMAND clang-format -style=file -i ${FILES_LIST} --verbose
    )
endfunction()

# Flash via st-link or jlink
function(targets_generate_flash_target TARGET)
    if("${TARGET}" STREQUAL "${PROJECT_NAME}")
        set(TARGET_SUFFIX "")
    else()
        set(TARGET_SUFFIX _${TARGET})
    endif()

    add_custom_target(flash${TARGET_SUFFIX}
        COMMAND echo "Flashing..."
        COMMAND STM32_Programmer_CLI -c port=SWD -w ${TARGET}.hex -v -rst
    )

    add_dependencies(flash${TARGET_SUFFIX} ${TARGET})

    set(input_file ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/jlink.in)
    configure_file(${input_file} ${CMAKE_CURRENT_BINARY_DIR}/jlinkflash/.jlink-flash${TARGET_SUFFIX})

    add_custom_target(jflash${TARGET_SUFFIX}
        COMMAND echo "Flashing ${PROJECT_NAME}.hex with J-Link"
        COMMAND ${JLINK_EXE} ${CMAKE_CURRENT_BINARY_DIR}/jlinkflash/.jlink-flash${TARGET_SUFFIX}
    )

    add_dependencies(jflash${TARGET_SUFFIX} ${TARGET})
endfunction()

function(targets_generate_vsfiles_target TARGET)
    if("${TARGET}" STREQUAL "${PROJECT_NAME}")
        set(TARGET_SUFFIX "")
    else()
        set(TARGET_SUFFIX _${TARGET})
    endif()

    set(DEBUG_FILE_NAME ${TARGET})

    set(input_file ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/launch.json.in)
    set(ouput_save_file ${CMAKE_CURRENT_BINARY_DIR}/vsfiles/.vsfiles${TARGET_SUFFIX})

    configure_file(${input_file} ${ouput_save_file})

    add_custom_target(debug${TARGET_SUFFIX}
        COMMAND echo "Configuring VS Code files for ${TARGET}"
        COMMAND cat ${ouput_save_file} > ${LAUNCH_JSON_PATH}
    )

    add_dependencies(debug${TARGET_SUFFIX} ${TARGET})
endfunction()

function(targets_generate_helpme_target)
    set(input_file ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/helpme.in)
    set(ouput_save_file ${CMAKE_CURRENT_BINARY_DIR}/.helpme)

    configure_file(${input_file} ${ouput_save_file})
endfunction()
