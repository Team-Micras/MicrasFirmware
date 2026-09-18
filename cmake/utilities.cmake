###############################################################################
## Utilities Functions
###############################################################################

# This function adds a target with name '${TARGET}_always_display_size'. The new
# target builds a TARGET and then calls the program defined in CMAKE_SIZE to
# display the size of the final ELF.
function(print_size_of_target TARGET)
    add_custom_command(
        TARGET ${TARGET}
        POST_BUILD
        COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${TARGET}>"
        COMMENT "Target Sizes: "
    )
endfunction()

# This function calls the objcopy program defined in CMAKE_OBJCOPY to generate
# file with object format specified in OBJCOPY_BFD_OUTPUT.
# The generated file has the name of the target output but with extension
# corresponding to the OUTPUT_EXTENSION argument value.
# The generated file will be placed in the same directory as the target output file.
function(_generate_file TARGET OUTPUT_EXTENSION OBJCOPY_BFD_OUTPUT)
    set(OUTPUT_FILE_NAME "${TARGET}.${OUTPUT_EXTENSION}")

    add_custom_command(
        TARGET ${TARGET}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ${OBJCOPY_BFD_OUTPUT} "$<TARGET_FILE:${TARGET}>" ${OUTPUT_FILE_NAME}
        BYPRODUCTS ${OUTPUT_FILE_NAME}
        COMMENT "Generating ${OBJCOPY_BFD_OUTPUT} file ${OUTPUT_FILE_NAME}"
    )
endfunction()

# This function adds post-build generation of the binary file from the target ELF.
# The generated file will be placed in the same directory as the ELF file.
function(generate_binary_file TARGET)
    _generate_file(${TARGET} "bin" "binary")
endfunction()

# This function adds post-build generation of the Motorola S-record file from the target ELF.
# The generated file will be placed in the same directory as the ELF file.
function(generate_srec_file TARGET)
    _generate_file(${TARGET} "srec" "srec")
endfunction()

# This function adds post-build generation of the Intel hex file from the target ELF.
# The generated file will be placed in the same directory as the ELF file.
function(generate_hex_file TARGET)
    _generate_file(${TARGET} "hex" "ihex")
endfunction()

# This function makes the linker emit a map file named after the target, instead of the
# single fixed name the CubeMX toolchain file would otherwise use for every executable.
function(generate_map_file TARGET)
    target_link_options(${TARGET} PRIVATE "-Wl,-Map=$<TARGET_FILE_BASE_NAME:${TARGET}>.map")
endfunction()

function(generate_helpme_text)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/helpme.in
        ${CMAKE_CURRENT_BINARY_DIR}/helpme
    )
endfunction()

# The file is rendered into the build directory at configure time and only copied into the
# source tree by the `vscode` target, so that configuring never writes to the source directory.
function(generate_vscode_target)
    set(TASKS_SAVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/vsfiles/tasks.json")

    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/tasks.json.in
        ${TASKS_SAVE_PATH}
    )

    add_custom_target(vscode
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/.vscode
        COMMAND ${CMAKE_COMMAND} -E copy ${TASKS_SAVE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/.vscode/tasks.json
        COMMENT "Writing .vscode/tasks.json"
    )
endfunction()
