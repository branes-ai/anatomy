# cmake/deps.cmake — FetchContent declarations for third-party deps.
#
# Permissive licenses only (MIT / BSL / BSD / Apache-2.0); see cortex ADR 0001
# and 0002. Pinned versions are intentional; bumps go via dedicated PRs so the
# release-please changelog records dependency churn separately from feature work.
#
# yaml-cpp (the config loaders), reflex (the glue and the simulator) and
# Universal (the precision-sweep types), at the same pins as racing-drone, so a
# mission that fetches both gets one copy of each.

include(FetchContent)

# Don't re-run `git fetch` on every build. Not FORCEd, so a dependency bump in an
# existing build tree can re-enable fetching with -DFETCHCONTENT_UPDATES_DISCONNECTED=OFF.
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Skip FetchContent update steps")

# ── catchorg/Catch2 v3 — testing framework ──────────────────────────
if(ANATOMY_BUILD_TESTS AND BUILD_TESTING)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.15.0
        GIT_SHALLOW    TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

# ── jbeder/yaml-cpp — vehicle / scenario / baseline files (MIT) ──
# Same pin as cortex. SYSTEM so its headers don't trip our warnings-as-errors
# targets.
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL       OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        yaml-cpp-0.9.0
    GIT_SHALLOW    TRUE
    SYSTEM
)
FetchContent_MakeAvailable(yaml-cpp)

# ── branes-ai/reflex — dynamics model, rate loop, mixer, filters (MIT) ──
# The glue's controller (glue/) and the simulator's plant and simulated flight
# controller (sim/). Pinned to a commit on reflex main, the same as
# racing-drone's; bumps go via dedicated PRs, here first. Its tests and benches
# are off because it is not the top-level project. It brings MTL5 5.20.0 with
# it. SYSTEM, like yaml-cpp: reflex is built warning-clean under its own flags,
# but a consumer's names can collide with its locals (MSVC C4459).
FetchContent_Declare(
    reflex
    GIT_REPOSITORY https://github.com/branes-ai/reflex.git
    GIT_TAG        33ad1bbea37cfe05f22286c2525d7a751da2b482  # main after reflex#33
    SYSTEM
)
FetchContent_MakeAvailable(reflex)

# ── stillwater-sc/universal — number systems for the precision sweep (MIT) ──
# The posits and cfloats of the precision-sweep types (sim/sweep_types.hpp).
# Header-only; the SDK, slots, glue and simulator never name a Universal type,
# so only code that includes sweep_types.hpp links it. Pinned
# to v5.11.0, as in reflex and cortex, and defined only if no one has yet (the
# same targets as reflex's test-only fetch). SYSTEM, as for the other deps:
# its headers are not warning-clean under our flags (stillwater-sc/universal#1265).
if(NOT TARGET universal::universal)
    FetchContent_Declare(
        universal
        GIT_REPOSITORY https://github.com/stillwater-sc/universal.git
        GIT_TAG        v5.11.0
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  _header_only_no_build
    )
    FetchContent_MakeAvailable(universal)
    # Universal's headers include both <sw/universal/...> and <universal/...>.
    add_library(universal INTERFACE)
    add_library(universal::universal ALIAS universal)
    target_include_directories(universal SYSTEM INTERFACE
        ${universal_SOURCE_DIR}/include
        ${universal_SOURCE_DIR}/include/sw)
    target_compile_features(universal INTERFACE cxx_std_20)
endif()
