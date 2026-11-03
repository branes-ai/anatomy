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
