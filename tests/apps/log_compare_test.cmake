# log_compare's exit codes and messages (apps/log_compare/): 0 for equal logs,
# 1 for logs that differ, naming the difference, 2 for an error.
#
# Inputs: WRITE_LOGS, LOG_COMPARE (executables), WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})
execute_process(COMMAND ${WRITE_LOGS} ${WORK_DIR} RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "write_logs failed (${rc})")
endif()

# expect(<exit code> <message> <args>...): log_compare exits with the code, and
# its output contains the message.
function(expect code message)
    execute_process(COMMAND ${LOG_COMPARE} ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT rc EQUAL code)
        message(FATAL_ERROR "log_compare ${ARGN}: exited ${rc}, not ${code}:\n${out}${err}")
    endif()
    string(FIND "${out}${err}" "${message}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "log_compare ${ARGN}: expected \"${message}\" in:\n${out}${err}")
    endif()
endfunction()

set(a ${WORK_DIR}/a.brlog)
expect(0 "are equal, 2 records" --a ${a} --b ${a})
expect(1 "the header's composition_hash differs: 'slots' and 'graph'" --a ${a} --b ${WORK_DIR}/hash.brlog)
expect(0 "are equal, 2 records, except composition_hash"
    --a ${a} --b ${WORK_DIR}/hash.brlog --except composition_hash)
expect(1 "record 1 (channel 17 at t=10 ns) has another payload"
    --a ${a} --b ${WORK_DIR}/payload.brlog --except composition_hash)
expect(2 "'hash' is not a header field" --a ${a} --b ${a} --except world_hash,hash)
expect(2 "cannot open the log file" --a ${a} --b ${WORK_DIR}/none.brlog)
expect(2 "usage: --a" --a ${a})
expect(2 "option --verbose is not accepted here" --a ${a} --b ${a} --verbose x)
message(STATUS "log_compare: passed")
