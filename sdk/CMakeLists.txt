# sdk/ — the framework library.
#
# `branes::anatomy` carries the data contracts (msg/), the slot concepts, their
# construction and the nulls (slots/), the stack concept (compose/), and the
# harness pieces (harness/) as headers, and compiles the config loaders
# (yaml-cpp), the replay log, and SHA-256. Everything is templated on the
# scalar type except the load-time configs and the log, which are double.

add_library(branes_anatomy STATIC
    src/config/baseline.cpp
    src/config/scenario.cpp
    src/config/vehicle.cpp
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
