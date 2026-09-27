###############################################################################
## Documentation
###############################################################################

include(FetchContent)

# The Doxygen default theme is referenced by the Doxyfile through $(DOXYGEN_AWESOME_DIR)
FetchContent_Declare(
    doxygen-awesome-css
    GIT_REPOSITORY https://github.com/jothepro/doxygen-awesome-css.git
    GIT_TAG v2.5.0
)

FetchContent_MakeAvailable(doxygen-awesome-css)

# MICRAS_VERSION is resolved by Doxygen through $(MICRAS_VERSION) in the Doxyfile.
# Backticks are used instead of $() so that the make generator does not expand it first.
add_custom_target(docs
    COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && MICRAS_VERSION=`git describe --always --dirty --tags 2>/dev/null || echo unknown` DOXYGEN_AWESOME_DIR=${doxygen-awesome-css_SOURCE_DIR} doxygen Doxyfile
    COMMENT "Generating HTML documentation and LaTeX sources in docs/"
)

# Kept separate from `docs` because it needs a LaTeX distribution, which the CI image
# deliberately does not carry
add_custom_target(docs_pdf
    COMMAND yes Q | ${CMAKE_MAKE_PROGRAM} -C ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/docs/latex/refman.pdf ${CMAKE_CURRENT_SOURCE_DIR}/docs/
    COMMENT "Building docs/refman.pdf from the generated LaTeX sources"
)

add_dependencies(docs_pdf docs)
