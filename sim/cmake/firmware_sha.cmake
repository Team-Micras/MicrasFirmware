###############################################################################
## Writes the firmware commit into a header, at build time
###############################################################################

# Run as a script (cmake -P) by the micras_firmware_sha target on every build, so a
# submodule bump is picked up without reconfiguring. file(CONFIGURE) rewrites the
# header only when the text changes, so an unchanged commit rebuilds nothing.
# Expects FIRMWARE_DIR and OUTPUT.

execute_process(
    COMMAND git -C "${FIRMWARE_DIR}" rev-parse HEAD
    OUTPUT_VARIABLE FIRMWARE_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(NOT FIRMWARE_SHA)
    set(FIRMWARE_SHA "unknown")
else()
    execute_process(
        COMMAND git -C "${FIRMWARE_DIR}" diff --quiet HEAD
        RESULT_VARIABLE FIRMWARE_DIRTY
        ERROR_QUIET
    )

    if(NOT FIRMWARE_DIRTY EQUAL 0)
        string(APPEND FIRMWARE_SHA "-dirty")
    endif()
endif()

file(CONFIGURE OUTPUT "${OUTPUT}" CONTENT "#define MICRAS_FIRMWARE_SHA \"${FIRMWARE_SHA}\"\n")
