# sdk/ — the framework library.
#
# `branes::anatomy` will carry the mission-neutral data contracts, slot
# concepts, configs, log, composition, and harness pieces, moved from
# racing-drone (docs/plan/move-from-racing-drone/). Until then it is the
# header-only skeleton the build and CI are set up on.

add_library(branes_anatomy INTERFACE)
add_library(branes::anatomy ALIAS branes_anatomy)

target_include_directories(branes_anatomy INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_anatomy INTERFACE cxx_std_20)
