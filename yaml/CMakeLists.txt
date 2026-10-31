# yaml/ — the YAML helpers for config loaders.
#
# `branes::anatomy_yaml` is header-only: <branes/anatomy/config/yaml.hpp>, the
# parsing and emitting helpers anatomy's loaders share (the baseline, vehicle
# and scenario files), for a mission's loaders too (its world file). It is a
# library of its own so that yaml-cpp, which its header includes, stays out of
# the core's and the profiles' public headers: only code that loads YAML links
# it.

add_library(branes_anatomy_yaml INTERFACE)
add_library(branes::anatomy_yaml ALIAS branes_anatomy_yaml)

target_include_directories(branes_anatomy_yaml INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_anatomy_yaml INTERFACE cxx_std_20)
target_link_libraries(branes_anatomy_yaml INTERFACE branes::anatomy yaml-cpp::yaml-cpp)
