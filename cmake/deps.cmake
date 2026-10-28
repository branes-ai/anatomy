# cmake/deps.cmake — FetchContent declarations for third-party deps.
#
# Permissive licenses only (MIT / BSL / BSD / Apache-2.0); see cortex ADR 0001
# and 0002. Pinned versions are intentional; bumps go via dedicated PRs so the
# release-please changelog records dependency churn separately from feature work.
#
# yaml-cpp (the config loaders), reflex (the glue and the simulator) and
# Universal (the precision-sweep types) join here with the move from
# racing-drone (docs/plan/move-from-racing-drone/), at racing-drone's pins.

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
