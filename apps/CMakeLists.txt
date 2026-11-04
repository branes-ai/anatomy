# apps/ — the tools a mission's gates run, built with the libraries.
#
# log_compare (log/compare.hpp; docs/plan/flow-graph, 2c): two replay logs,
# equal but for the header fields left out. The target is anatomy_log_compare,
# so it can't clash with a mission's targets, and the binary log_compare; a
# mission's test runs it as $<TARGET_FILE:anatomy_log_compare>.
add_executable(anatomy_log_compare log_compare/main.cpp)
target_link_libraries(anatomy_log_compare PRIVATE branes::anatomy)
anatomy_set_warnings(anatomy_log_compare)
set_target_properties(anatomy_log_compare PROPERTIES OUTPUT_NAME log_compare FOLDER "Apps")

# trace_compare (flow/trace.hpp; docs/plan/flow-testing, T4): two edge traces,
# record by record, naming the first edge they differ on. The target is
# anatomy_trace_compare, and the binary trace_compare.
add_executable(anatomy_trace_compare trace_compare/main.cpp)
target_link_libraries(anatomy_trace_compare PRIVATE branes::anatomy)
anatomy_set_warnings(anatomy_trace_compare)
set_target_properties(anatomy_trace_compare PROPERTIES OUTPUT_NAME trace_compare FOLDER "Apps")
