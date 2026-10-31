# The checks of anatomy's own sources (docs/plan/move-from-racing-drone).
#
# framework.names_no_mission: anatomy's code (comments aside) names nothing of
# a mission: no racing namespace, include, or task type. It scans sdk/, yaml/,
# mobility/, sim/, glue/, tests/ and cmake/, except this directory, which holds
# the check, its fixture, and this registration, all of which name the tokens
# on purpose.
# framework.detects_mission_names checks that the check fails, with the right
# messages, on the fixture.
add_test(NAME framework.names_no_mission
    COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DSKIP=tests/tools/
            -P ${CMAKE_CURRENT_SOURCE_DIR}/mission_names_test.cmake)
add_test(NAME framework.detects_mission_names
    COMMAND ${CMAKE_COMMAND}
            -DSCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/mission_names_test.cmake
            "-DARGS=-DSOURCE_DIR=${PROJECT_SOURCE_DIR};-DDIRS=tests/tools/mission_names_fixture"
            "-DEXPECT=names_mission.hpp:4: names 'branes/racing/';names_mission.hpp:5: names 'branes::racing';names_mission.hpp:5: names 'CourseMap'"
            -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_failure.cmake)
