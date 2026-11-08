# workbench/ — patterns with closed-form oracles, independent of the missions
# (docs/adr/0005, step 1; docs/plan/workbench).
#
# `branes::anatomy_workbench` is an interface library over the core: the
# driver, its sources and trace codecs, and the patterns, each its operators,
# graph, hand-wired run and oracle. It is not installed or exported, so no
# mission links a reference operator (ADR 0005, decision 1). workbench_run
# runs a pattern and prints its oracle's report.

add_library(branes_anatomy_workbench INTERFACE)
add_library(branes::anatomy_workbench ALIAS branes_anatomy_workbench)
target_include_directories(branes_anatomy_workbench INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(branes_anatomy_workbench INTERFACE branes::anatomy)

# The target is anatomy_workbench_run, so it can't clash with a mission's, and the binary workbench_run.
add_executable(anatomy_workbench_run apps/workbench_run/main.cpp)
target_link_libraries(anatomy_workbench_run PRIVATE branes::anatomy_workbench)
anatomy_set_warnings(anatomy_workbench_run)
set_target_properties(anatomy_workbench_run PROPERTIES OUTPUT_NAME workbench_run FOLDER "Workbench")
