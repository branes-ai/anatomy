# Build & Test

`anatomy` is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

## Build and run the tests

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

The tests are built only when anatomy is the top-level project (`ANATOMY_BUILD_TESTS`), so a mission repo that fetches anatomy gets just the libraries.

On Windows, [a walkthrough in Visual Studio](visual-studio-walkthrough.md) generates the solution with the `msvc` preset, and steps through the workbench in the debugger.

## Using anatomy from a mission repo

A mission repo fetches anatomy pinned to a commit, and links its targets:

```cmake
FetchContent_Declare(
    anatomy
    GIT_REPOSITORY https://github.com/branes-ai/anatomy.git
    GIT_TAG        <commit>
    SYSTEM
)
FetchContent_MakeAvailable(anatomy)
target_link_libraries(my_mission PRIVATE branes::anatomy)
```

## The docs site

This site is generated from [`docs/`](../README.md). To preview it:

```bash
cd docs-site
npm ci
npm run dev        # regenerates the pages from docs/, then serves them
```

Next: [repository layout](repository-layout.md).
