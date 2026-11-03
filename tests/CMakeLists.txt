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

# The field mission's graph baselines (docs/plan/flow-graph, 2b): its
# operators, bodies and research graph, registered, and a header per baseline
# file naming what fills its graph (cmake/flow.cmake), for the CPU stack's
# test (sim/flow_stack.cpp).
include(${PROJECT_SOURCE_DIR}/cmake/flow.cmake)
set(field_dir ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/flow/field)
set(field_graph flow/field/field_graph.hpp)
flow_register_operator(field GroundTruthCore HEADER branes/anatomy/mobility/slots/ground_truth_core.hpp
    TYPE branes::anatomy::GroundTruthCore BINDING field::EstimatorBinding)
flow_register_operator(field GoalPlanner HEADER ${field_graph} TYPE field::GoalPlanner BINDING field::PlannerBinding)
flow_register_body(field GoalSmooth HEADER ${field_graph} TYPE field::GoalSmooth)
flow_register_graph(field Direct HEADER ${field_graph} TYPE field::DirectGraph)
foreach(null IN ITEMS NullController NullSafetyFilter NullDaaTracker NullFcBridge)
    string(REPLACE "Null" "" composite ${null})
    flow_register_operator(field ${null} HEADER branes/anatomy/mobility/slots/neutral_nulls.hpp
        TYPE branes::anatomy::${null} BINDING branes::anatomy::flow::${composite}Binding)
endforeach()
flow_register_operator(field CascadedPid HEADER branes/anatomy/glue/cascaded_pid.hpp
    TYPE branes::anatomy::glue::CascadedPid BINDING branes::anatomy::flow::ControllerBinding)
foreach(baseline IN ITEMS field-null field-pid field-body field-research)
    flow_add_baseline(field ${baseline} FILE ${field_dir}/${baseline}.yaml TEMPLATE ${field_dir}/baseline.hpp.in
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/generated/flow/field/${baseline}.hpp)
endforeach()
target_include_directories(flow_stack PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fixtures ${CMAKE_CURRENT_BINARY_DIR}/generated)

# ── The apps (apps/) ───────────────────────────────────────────────────────
# log_compare's exit codes and messages, on logs write_logs writes.
add_executable(log_compare_write_logs ${CMAKE_CURRENT_SOURCE_DIR}/apps/write_logs.cpp)
target_link_libraries(log_compare_write_logs PRIVATE branes::anatomy)
anatomy_set_warnings(log_compare_write_logs)
set_target_properties(log_compare_write_logs PROPERTIES FOLDER "Tests/apps")
add_test(NAME apps.log_compare
    COMMAND ${CMAKE_COMMAND}
        -DWRITE_LOGS=$<TARGET_FILE:log_compare_write_logs>
        -DLOG_COMPARE=$<TARGET_FILE:anatomy_log_compare>
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/apps_log_compare
        -P ${CMAKE_CURRENT_SOURCE_DIR}/apps/log_compare_test.cmake)

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
# docs/plan/flow-domains), and the graph invariants the compiler checks
# (flow/builder.hpp; docs/plan/flow-graph): one file per violation, each
# expecting the message of the invariant it breaks. The tests build in the one build tree, so they
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
    "s2_no_output|S2: a signature has at least one out or inout port, or a state domain"
    "g1_operator|G1: an operator's name matches"
    "g2_handler|G2: a firing names a handler its node declares"
    "g2_port|G2: a handler's in ports are in or inout ports of its signature"
    "g2_body|G2: a body fills its composite: it has its binding"
    "g3_arity|G3: every in port of the handler has one source"
    "g3_time|G3: a handler that takes a time is fired at"
    "g3_no_time|G3: a handler that takes no time is not fired at an input"
    "g4_edge|G4: every edge joins two ports of the same domain"
    "g4_projection|field_t: the record has no field of that name"
    "g4_member|a projection of a message names its member"
    "g2_call|G2: a handler's call\\(\\) takes its operator"
    "stack_message|the CPU stack runs a handler whose ports carry message types"
    "g4_output|G4: every edge joins two ports of the same domain"
    "g4_body|G4: a body's handler returns its composite handler's out ports")
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
