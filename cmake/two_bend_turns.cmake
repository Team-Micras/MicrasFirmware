###############################################################################
## The turns of two bends
###############################################################################

# Runs the turn designer into the header that config/dynamics_config.cpp includes, whenever the
# designer is rebuilt. The robot model, the margins and the turn code of micras-lib are compiled into
# it, so a change to any of them rebuilds it and designs the turns again.
#
# micras_add_two_bend_turns(<designer target> <generated include directory>)
#
# Adds the target micras_two_bend_turns, which every target compiling dynamics_config.cpp has to
# depend on.
function(micras_add_two_bend_turns DESIGNER INCLUDE_DIRECTORY)
    set(HEADER "${INCLUDE_DIRECTORY}/two_bend_turns.hpp")

    add_custom_command(
        OUTPUT "${HEADER}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${INCLUDE_DIRECTORY}"
        COMMAND "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../sim/scripts/turn_designs.sh" "$<TARGET_FILE:${DESIGNER}>"
                "${HEADER}"
        DEPENDS ${DESIGNER} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../sim/scripts/turn_designs.sh"
        COMMENT "Designing the turns of two bends"
        VERBATIM
    )

    add_custom_target(micras_two_bend_turns ALL
        DEPENDS "${HEADER}"
    )
endfunction()
