# sdk/ — the core library (anatomy docs/adr/0001, docs/plan/split-libraries).
#
# `branes::anatomy` carries what every kind of mission needs and names no
# vehicle: time and geometry, the camera frame and its codec (msg/), the stack
# profile (compose/), the slot construction (slots/), the baseline config, the
# replay log, and the harness (runner, replay, flight ticks, batch pool) as
# headers, and compiles the baseline loader (yaml-cpp), the log, and SHA-256.
# A roaming vehicle's contracts are branes::anatomy_mobility (mobility/).

add_library(branes_anatomy STATIC
    src/config/baseline.cpp
    src/log/log.cpp
    src/util/sha256.cpp
)
add_library(branes::anatomy ALIAS branes_anatomy)

target_include_directories(branes_anatomy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_anatomy PUBLIC cxx_std_20)
# yaml-cpp stays an implementation detail of the loaders.
target_link_libraries(branes_anatomy PRIVATE yaml-cpp::yaml-cpp PUBLIC Threads::Threads)
anatomy_set_warnings(branes_anatomy)
set_target_properties(branes_anatomy PROPERTIES FOLDER "SDK")
