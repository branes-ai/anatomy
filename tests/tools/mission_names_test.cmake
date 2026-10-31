# framework.names_no_mission (docs/plan/move-from-racing-drone): anatomy's code
# names nothing of a mission. Every C++ and CMake file under the scanned
# directories is read with its comments stripped (comments may cite a mission,
# racing-drone, as an example), and must not contain any of the tokens below:
# racing-drone's namespace, include root, and task types.
#
# Inputs: SOURCE_DIR; DIRS (default: sdk yaml mobility sim glue tests cmake, relative to
# SOURCE_DIR); SKIP (a path prefix to skip, relative to SOURCE_DIR).

if(NOT DIRS)
    set(DIRS sdk yaml mobility sim glue tests cmake)
endif()
set(tokens
    "branes::racing" "branes/racing/" "namespace racing" "RACING_" "racing_"
    "CourseMap" "GateObservation" "oracle_gates" "kSlotNames" "kRacing" "GateDetector")

set(files "")
foreach(dir IN LISTS DIRS)
    file(GLOB_RECURSE found LIST_DIRECTORIES false RELATIVE ${SOURCE_DIR}
        ${SOURCE_DIR}/${dir}/*.hpp ${SOURCE_DIR}/${dir}/*.cpp ${SOURCE_DIR}/${dir}/*.cmake
        ${SOURCE_DIR}/${dir}/CMakeLists.txt)
    list(APPEND files ${found})
endforeach()
if(SKIP)
    list(FILTER files EXCLUDE REGEX "^${SKIP}")
endif()

set(failures "")
foreach(file IN LISTS files)
    file(STRINGS ${SOURCE_DIR}/${file} lines)
    set(n 0)
    foreach(line IN LISTS lines)
        math(EXPR n "${n} + 1")
        if(file MATCHES "\\.(hpp|cpp)$")
            string(REGEX REPLACE "//.*$" "" code "${line}")
        else()
            string(REGEX REPLACE "#.*$" "" code "${line}")
        endif()
        foreach(token IN LISTS tokens)
            string(FIND "${code}" "${token}" at)
            if(NOT at EQUAL -1)
                list(APPEND failures "${file}:${n}: names '${token}'")
            endif()
        endforeach()
    endforeach()
endforeach()

list(LENGTH files count)
if(failures)
    list(JOIN failures "\n  " text)
    message(FATAL_ERROR "anatomy names a mission:\n  ${text}")
endif()
message(STATUS "anatomy names no mission: ${count} files checked")
