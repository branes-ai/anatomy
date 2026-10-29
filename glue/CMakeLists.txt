# glue/ — adapters from platform components to the slots.
#
# `branes::anatomy_glue` is header-only: the reflex parameter mapping for a
# vehicle file, and the slot adapters around reflex components (CascadedPid).
# A mission's baselines name these implementations; linking it does not pull
# in the simulator. It builds on the mobility profile (branes::anatomy_mobility).

add_library(branes_anatomy_glue INTERFACE)
add_library(branes::anatomy_glue ALIAS branes_anatomy_glue)

target_include_directories(branes_anatomy_glue INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_anatomy_glue INTERFACE cxx_std_20)
target_link_libraries(branes_anatomy_glue INTERFACE branes::anatomy_mobility branes::reflex)
