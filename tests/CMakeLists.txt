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

# ── Mobility profile (mobility/) ───────────────────────────────────────────
file(GLOB MOBILITY_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/mobility/*.cpp)
compile_all(FOLDER "Tests/mobility" LIBS branes::anatomy_mobility SOURCES ${MOBILITY_SRC})
foreach(source ${MOBILITY_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE ANATOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Simulator and glue (sim/, glue/) ───────────────────────────────────────
file(GLOB SIM_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp)
compile_all(FOLDER "Tests/sim" LIBS branes::anatomy_mobility_sim SOURCES ${SIM_SRC})
foreach(source ${SIM_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE ANATOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Scalar types ───────────────────────────────────────────────────────────
# The casts and the contracts in every type of the precision sweep
# (sim/sweep_types.hpp); these tests link Universal.
file(GLOB PRECISION_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/precision/*.cpp)
compile_all(FOLDER "Tests/precision" LIBS branes::anatomy_mobility_sim universal::universal SOURCES ${PRECISION_SRC})
foreach(source ${PRECISION_SRC})
    get_filename_component(name ${source} NAME_WE)
    target_compile_definitions(${name} PRIVATE ANATOMY_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endforeach()

# ── Every header compiles alone, against its own library ───────────────────
# One translation unit per header, linked to that header's library and nothing
# else, so a core header that includes a mobility one fails the build
# (cmake/header_check.cmake; docs/plan/split-libraries).
include(${PROJECT_SOURCE_DIR}/cmake/header_check.cmake)
anatomy_header_check(NAME headers_core ROOT ${PROJECT_SOURCE_DIR}/sdk/include LIBS branes::anatomy)
anatomy_header_check(NAME headers_yaml ROOT ${PROJECT_SOURCE_DIR}/yaml/include LIBS branes::anatomy_yaml)
anatomy_header_check(NAME headers_mobility ROOT ${PROJECT_SOURCE_DIR}/mobility/include LIBS branes::anatomy_mobility)
anatomy_header_check(NAME headers_glue ROOT ${PROJECT_SOURCE_DIR}/glue/include LIBS branes::anatomy_glue)
anatomy_header_check(NAME headers_mobility_sim ROOT ${PROJECT_SOURCE_DIR}/sim/include
    LIBS branes::anatomy_mobility_sim universal::universal)

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

# The domain flow vocabulary's invariants (flow/domain.hpp, flow/signature.hpp;
# docs/plan/flow-domains): one file per violation, each expecting the message
# of the invariant it breaks. The tests build in the one build tree, so they
# hold a lock and run one at a time, the time base's test among them.
set(flow_compile_fail
    "d1_dim|D1: every extent has a size or a bound of at least 1"
    "d1_bounded|D1: every extent has a size or a bound of at least 1"
    "d1_index_space|D1: every extent has a size or a bound of at least 1"
    "d2_rank|D2: a structure other than dense needs a square domain of rank 2"
    "d2_square|D2: a structure other than dense needs a square domain of rank 2"
    "d3_element|D3: the element type is an element type"
    "r1_empty|R1: a record has at least one field"
    "r1_duplicate|R1: a record's field names are non-empty and distinct"
    "s1_duplicate|S1: the port and state names are non-empty and distinct"
    "s2_no_output|S2: a signature has at least one out or inout port, or a state domain")
foreach(entry IN LISTS flow_compile_fail)
    string(REPLACE "|" ";" entry "${entry}")
    list(GET entry 0 case)
    list(GET entry 1 message)
    add_library(flow_${case} OBJECT EXCLUDE_FROM_ALL ${CMAKE_CURRENT_SOURCE_DIR}/compile_fail/flow_${case}.cpp)
    target_include_directories(flow_${case} PRIVATE ${PROJECT_SOURCE_DIR}/sdk/include)
    target_compile_features(flow_${case} PRIVATE cxx_std_20)
    add_test(NAME compile_fail.flow.${case}
        COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target flow_${case} --config $<CONFIG>)
    set_tests_properties(compile_fail.flow.${case} PROPERTIES
        PASS_REGULAR_EXPRESSION "${message}" RESOURCE_LOCK anatomy_build_tree)
endforeach()
set_tests_properties(compile_fail.time_base.narrow_wide PROPERTIES RESOURCE_LOCK anatomy_build_tree)

# ── Anatomy names no mission ───────────────────────────────────────────────
# The check, its fixture, and their registration name the mission tokens they
# look for, so they live in tests/tools/, which the check does not scan.
add_subdirectory(tools)
