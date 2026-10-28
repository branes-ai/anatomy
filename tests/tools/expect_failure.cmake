# expect_failure.cmake: run a CMake script that must fail, and check both that
# it exits non-zero and that its output contains every expected message.
# (PASS_REGULAR_EXPRESSION alone would ignore the exit code.)
#
# Inputs: SCRIPT (the script), ARGS (a list of -D definitions for it),
#         EXPECT (a list of messages that must all appear).

execute_process(COMMAND ${CMAKE_COMMAND} ${ARGS} -P ${SCRIPT}
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
set(text "${out}${err}")
if(rc EQUAL 0)
    message(FATAL_ERROR "expected ${SCRIPT} to fail, but it exited 0:\n${text}")
endif()
foreach(expected IN LISTS EXPECT)
    string(FIND "${text}" "${expected}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "expected the message \"${expected}\" in:\n${text}")
    endif()
endforeach()
message(STATUS "failed as expected (exit ${rc}), with every expected message")
