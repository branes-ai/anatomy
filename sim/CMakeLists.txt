# sim/ — the closed-loop SITL library.
#
# `branes::anatomy_sim` is header-only: the reflex plant and the simulated
# flight controller, the IMU model, the camera model, the closed-loop event
# source (BasicSimulator), the SITL and batch loops. It runs in double, like
# the logs. The precision-sweep types (sweep_types.hpp) also need Universal;
# code that includes them links universal::universal.

add_library(branes_anatomy_sim INTERFACE)
add_library(branes::anatomy_sim ALIAS branes_anatomy_sim)

target_include_directories(branes_anatomy_sim INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_anatomy_sim INTERFACE cxx_std_20)
target_link_libraries(branes_anatomy_sim INTERFACE branes::anatomy branes::anatomy_glue branes::reflex)
