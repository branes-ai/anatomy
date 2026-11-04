# trace_compare's exit codes and messages (apps/trace_compare/): 0 for equal
# traces, 1 for traces that differ, naming the first edge, 2 for an error.
#
# Inputs: WRITE_TRACES, TRACE_COMPARE (executables), WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})
execute_process(COMMAND ${WRITE_TRACES} ${WORK_DIR} RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "write_traces failed (${rc})")
endif()

# expect(<exit code> <message> <args>...): trace_compare exits with the code,
# and its output contains the message.
function(expect code message)
    execute_process(COMMAND ${TRACE_COMPARE} ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT rc EQUAL code)
        message(FATAL_ERROR "trace_compare ${ARGN}: exited ${rc}, not ${code}:\n${out}${err}")
    endif()
    string(FIND "${out}${err}" "${message}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "trace_compare ${ARGN}: expected \"${message}\" in:\n${out}${err}")
    endif()
endfunction()

set(a ${WORK_DIR}/a.brtrace)
expect(0 "are equal, 4 records" --a ${a} --b ${a})
expect(1 "record 0: in0[0].y (n0.apply) at t=10 ns has another value" --a ${a} --b ${WORK_DIR}/value.brtrace)
expect(1 "record 3: a has more records, from in0[0].y (n0.apply)" --a ${a} --b ${WORK_DIR}/short.brtrace)
expect(2 "cannot open the trace file" --a ${a} --b ${WORK_DIR}/none.brtrace)
expect(2 "usage: --a" --a ${a})
expect(2 "option --verbose is not accepted here" --a ${a} --b ${a} --verbose x)
message(STATUS "trace_compare: passed")
