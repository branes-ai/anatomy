# mobility/ — the mobility profile (anatomy docs/adr/0001, docs/plan/split-libraries).
#
# `branes::anatomy_mobility` carries what a roaming vehicle adds to the core:
# the IMU, estimate, reference, command, truth, FC, DAA and region contracts
# and their codecs (msg/), TickOutput and MobilityProfile (compose/), the
# vehicle and scenario configs (config/), the vehicle's slot concepts, context
# and nulls (slots/), the synthetic sensors (harness/), and the log dump
# (log/). Its headers are under <branes/anatomy/mobility/...>; it compiles the
# vehicle and scenario loaders (yaml-cpp).

add_library(branes_anatomy_mobility STATIC
    src/config/scenario.cpp
    src/config/vehicle.cpp
)
add_library(branes::anatomy_mobility ALIAS branes_anatomy_mobility)

target_include_directories(branes_anatomy_mobility PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# The loaders share the core's private YAML helpers (sdk/src/config/parse.hpp).
target_include_directories(branes_anatomy_mobility PRIVATE ${PROJECT_SOURCE_DIR}/sdk/src)
target_compile_features(branes_anatomy_mobility PUBLIC cxx_std_20)
target_link_libraries(branes_anatomy_mobility PUBLIC branes::anatomy PRIVATE yaml-cpp::yaml-cpp)
anatomy_set_warnings(branes_anatomy_mobility)
set_target_properties(branes_anatomy_mobility PROPERTIES FOLDER "SDK")
