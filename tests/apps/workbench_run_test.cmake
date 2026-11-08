# workbench_run's exit codes (workbench/apps/workbench_run/): 0 when every
# check of the oracle passes, 1 when one fails, 2 for a usage error; and the
# files it writes when asked: a trace that trace_compare reads, the graph's
# canonical form and its flowchart.
#
# Inputs: WORKBENCH_RUN, TRACE_COMPARE (executables), WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

# expect(<exit code> <message> <args>...): the tool exits with the code, and its output contains the message.
function(expect code message)
    execute_process(COMMAND ${WORKBENCH_RUN} ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT rc EQUAL code)
        message(FATAL_ERROR "workbench_run ${ARGN}: exited ${rc}, not ${code}:\n${out}${err}")
    endif()
    string(FIND "${out}${err}" "${message}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "workbench_run ${ARGN}: expected \"${message}\" in:\n${out}${err}")
    endif()
endfunction()

set(a ${WORK_DIR}/a.brtrace)
set(b ${WORK_DIR}/b.brtrace)
expect(0 "mlp: 3 checks, all passed" mlp --inputs 20 --trace ${a} --mermaid ${WORK_DIR}/g.mmd --canonical ${WORK_DIR}/g.txt)
expect(0 "all passed" mlp --inputs 20 --trace ${b})
expect(0 "all passed" mlp --seed 4 --inputs 5)
expect(1 "FAIL" mlp --inputs 20 --tolerance-scale 0)
expect(2 "usage: workbench_run" --inputs 3)
expect(2 "no pattern 'kalman'" kalman)
expect(2 "--inputs takes a positive integer" mlp --inputs 0)
expect(2 "--seed takes an unsigned integer" mlp --seed x)
expect(2 "option --verbose is not accepted here" mlp --verbose x)
expect(2 "cannot write" mlp --canonical ${WORK_DIR}/none/g.txt)

# Two runs of one seed: their traces are equal, record by record.
execute_process(COMMAND ${TRACE_COMPARE} --a ${a} --b ${b} RESULT_VARIABLE rc OUTPUT_VARIABLE out)
if(NOT rc EQUAL 0 OR NOT out MATCHES "are equal, 120 records")
    message(FATAL_ERROR "trace_compare of two runs: exited ${rc}:\n${out}")
endif()
file(READ ${WORK_DIR}/g.txt canonical)
if(NOT canonical MATCHES "instance linear1 Linear_f32_32x16 -")
    message(FATAL_ERROR "the canonical form names no Linear_f32_32x16:\n${canonical}")
endif()
file(READ ${WORK_DIR}/g.mmd chart)
if(NOT chart MATCHES "flowchart LR")
    message(FATAL_ERROR "the flowchart isn't one:\n${chart}")
endif()
message(STATUS "workbench_run: passed")
