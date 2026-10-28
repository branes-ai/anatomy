# Regression tests. Each .cpp under a layer subdirectory becomes a Catch2 test
# executable automatically — see cmake/compile_all.cmake. To add a test, drop a
# `<name>.cpp` into the matching tests/<layer>/ subdirectory; no edit here is
# needed. File stems must be globally unique (they name the targets).

include(${PROJECT_SOURCE_DIR}/cmake/compile_all.cmake)

# ── SDK layer ──────────────────────────────────────────────────────────────
file(GLOB SDK_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sdk/*.cpp)
compile_all(FOLDER "Tests/sdk" LIBS branes::anatomy SOURCES ${SDK_SRC})
foreach(source ${SDK_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE ANATOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Simulator and glue (sim/, glue/) ───────────────────────────────────────
file(GLOB SIM_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp)
compile_all(FOLDER "Tests/sim" LIBS branes::anatomy_sim SOURCES ${SIM_SRC})
foreach(source ${SIM_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE ANATOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Scalar types ───────────────────────────────────────────────────────────
# The casts and the contracts in every type of the precision sweep
# (sim/sweep_types.hpp); these tests link Universal.
file(GLOB PRECISION_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/precision/*.cpp)
compile_all(FOLDER "Tests/precision" LIBS branes::anatomy_sim universal::universal SOURCES ${PRECISION_SRC})
foreach(source ${PRECISION_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE ANATOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Code that must not compile ─────────────────────────────────────────────
# A TimeBase whose Wide type has fewer significand digits than T is a
# static_assert (msg/time.hpp). The object library sits outside `all`, so
# building it compiles that one file and nothing else; the test builds it and
# expects the static_assert's message.
add_library(time_base_narrow_wide OBJECT EXCLUDE_FROM_ALL ${CMAKE_CURRENT_SOURCE_DIR}/compile_fail/time_base_narrow_wide.cpp)
target_include_directories(time_base_narrow_wide PRIVATE ${PROJECT_SOURCE_DIR}/sdk/include)
target_compile_features(time_base_narrow_wide PRIVATE cxx_std_20)
add_test(NAME compile_fail.time_base.narrow_wide
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target time_base_narrow_wide --config $<CONFIG>)
set_tests_properties(compile_fail.time_base.narrow_wide PROPERTIES
    PASS_REGULAR_EXPRESSION "Wide has fewer significand digits than T")

# ── Anatomy names no mission ───────────────────────────────────────────────
# The check, its fixture, and their registration name the mission tokens they
# look for, so they live in tests/tools/, which the check does not scan.
add_subdirectory(tools)
