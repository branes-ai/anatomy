# Plan: the framework moves here from racing-drone

Issue: [racing-drone#65](https://github.com/branes-ai/racing-drone/issues/65). Status: in progress. PR 2a (#1) and 2b done; 2c (racing-drone's switch) to come.

## Goal

Step 2 of racing-drone's [ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md): the mission-neutral half of racing-drone's framework moves here. Step 1 left those files mission-neutral and listed them: 56 files in racing-drone's `cmake/framework_boundary.cmake`.

- **Names:** the namespace becomes `branes::anatomy`, and the include root `<branes/anatomy/...>`.
- **Unchanged:** behavior, the log format, and the channel ids. racing-drone's logs, replays and results stay byte-identical.

## The three PRs

| PR | Repo | Content |
|---|---|---|
| **2a** | anatomy | This skeleton: CMake, presets, workflows, lint, commitlint, release-please, README, docs and docs site, with one smoke test |
| **2b** | anatomy | The move: the 56 files under the new names, their neutral tests and the split-out neutral cases of racing-drone's mixed tests, and a ctest that anatomy names nothing racing. Released as anatomy v0.1.0 |
| **2c** | racing-drone | The switch: fetch anatomy at the 2b commit, delete the moved files, and check byte-identity and log compatibility |

## Acceptance for anatomy

1. It builds and its tests pass on every preset in CI, with clang-format and commitlint clean.
2. Its sources name nothing racing (checked by a ctest, from 2b).
3. Its neutral tests pass, including the non-racing mission of `neutral_mission.cpp` (from 2b).

The acceptance for racing-drone's switch is in [racing-drone#65](https://github.com/branes-ai/racing-drone/issues/65).

## Decisions (approved with racing-drone#65)

- **History:** the files start fresh here, with a commit message naming the racing-drone commit they come from.
- **Three targets:** `branes::anatomy` (yaml-cpp only), `branes::anatomy_glue` (adds reflex) and `branes::anatomy_sim` (adds reflex, and Universal for the sweep types).
- **Names in racing-drone:** one header there brings each anatomy name it uses into `branes::racing` by an explicit using-declaration.

## PR 2a (#1)

The skeleton, laid out like reflex and racing-drone:
- the root config, copied from racing-drone (`.clang-format`, `.editorconfig`, `CMakePresets.json`, `commitlint.config.cjs`, release-please);
- the workflows: CI (clang-format, Linux GCC and Clang in Debug and Release, Windows MSVC, the docs site), commitlint, release-please, and the docs deploy;
- `cmake/` (warnings, `compile_all`, and deps with Catch2 only for now);
- `sdk/` as a header-only `branes::anatomy`, with one smoke test;
- `docs/`, and the `docs-site/` generated from it.

## PR 2b

**What moved:** the 56 files of racing-drone's `cmake/framework_boundary.cmake` (at racing-drone 9b9a525). Three of them, `cmake/compile_all.cmake`, `warnings.cmake` and `deps.cmake`, were already here from 2a; they keep anatomy's names and gain the dependencies.

| From racing-drone | To anatomy |
|---|---|
| `sdk/include/branes/racing/...` | `sdk/include/branes/anatomy/...` |
| `sdk/src/...` | `sdk/src/...` |
| `baselines/glue/include/branes/racing/glue/...` | `glue/include/branes/anatomy/glue/...` |
| `sim/include/branes/racing/sim/...` | `sim/include/branes/anatomy/sim/...` |
| `cmake/composition.cmake` | `cmake/composition.cmake` |
| `tests/sdk/neutral_harness.cpp`, `tests/sim/neutral_mission.cpp` | the same paths |

**Names:**
- `branes::racing` and its sub-namespaces became `branes::anatomy`, with the same sub-namespaces, and `<branes/racing/...>` became `<branes/anatomy/...>`.
- Comments that cited racing-drone's documents now say so ("racing-drone docs/..."), and examples of a mission say "racing-drone:".
- The coordinate frame the contracts call "course frame" is now "world frame".
- The log's bad-magic error is now "not a log (bad magic)"; it was "not a racing log (bad magic)".

**Targets:**
- `branes::anatomy`: a static library with the config loaders, the log and SHA-256. It links yaml-cpp privately, and Threads for the batch pool.
- `branes::anatomy_glue`: adds reflex.
- `branes::anatomy_sim`: adds reflex.

`ANATOMY_CMAKE_DIR` names `cmake/` for a mission that includes `composition.cmake`. yaml-cpp, reflex and Universal are pinned as in racing-drone.

**Tests:** 102 ctest entries, against 1 in 2a.
- **Moved as is:** `tests/sdk/util.cpp`, `neutral_harness.cpp`, `tests/sim/neutral_mission.cpp`, `tests/compile_fail/time_base_narrow_wide.cpp` (with its registration), and `tests/precision/precision_fixtures.hpp`.
- **Split from racing-drone's mixed tests,** with every neutral case kept:
  - `tests/sdk/codec.cpp` loses its gate parts;
  - `config.cpp` runs on a test mission: a `world:` key, a `noise_px` parameter, and three slots, through the fixtures in `tests/fixtures/`;
  - `log.cpp` uses a test mission channel (id 5);
  - `msg_contracts.cpp` loses the gate and course cases;
  - `tests/sim/sim_components.cpp` keeps the camera-model case and loses the gate projector's;
  - `sim_simulator.cpp` runs `BasicSimulator` with a test oracle that marks each frame on the test mission's channel, so the oracle event order is still tested;
  - `tests/precision/precision_casts.cpp` loses the gate observations.
- **Fixtures:** `tests/fixtures/vehicle.yaml` (a copy of racing-drone's RP-1), `world.yaml`, `scenarios/{smoke,faults,speed}.yaml` and `baseline.yaml`.
- **New checks:**
  - `framework.names_no_mission`: anatomy's code, comments aside, names none of racing's namespace, include root or task types.
  - `framework.detects_mission_names`: requires that check to fail on a fixture that does.

**Acceptance:**
1. **Build and tests:** 102 of 102 pass on `gcc-debug` and on `clang-debug`, and clang-format 18 is clean on the 65 C++ files. CI runs the full matrix, including MSVC.
2. **No mission names:** `framework.names_no_mission` passes over 73 files.
3. **Neutral tests:** they pass, including the non-racing mission of `neutral_mission.cpp`.

**Departures:**
- **Not moved:** `tests/tools/reference_log_fixture.cpp` and racing-drone's boundary-check fixtures. The first serves racing-drone's `log_dump` test, and the boundary check is racing-drone's; anatomy gets `framework.names_no_mission` instead.
- **What the check scans:** the plan described the no-racing check as a grep for "racing". It looks for racing's identifiers in code with comments stripped, because the comments cite racing-drone as an example on purpose.
- **The renamed error message** above is a behavior change. It is in an error, never in a log or a result.

