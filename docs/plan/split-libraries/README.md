---
status: complete
issue: branes-ai/anatomy#10
---
# Plan: the libraries split into core, mobility and mobility_sim

Issue: [anatomy#10](https://github.com/branes-ai/anatomy/issues/10). 2a (#11) and 2b (#13) here, released in v0.3.0, and 2c, racing-drone's switch ([racing-drone#72](https://github.com/branes-ai/racing-drone/pull/72)).

## Goal

Step 2 of [ADR 0001](../../adr/0001-core-profiles-multi-node.md) (Decision 1, order of work 5.2). Step 1 ([plan](../generalize-core/README.md)) made the Runner generic over a stack profile. The files still sit in libraries cut by dependency (yaml-cpp, reflex), not by kind of mission. This step cuts them by kind of mission:

| Target | Directory | Include root | Holds | Links |
|---|---|---|---|---|
| `branes::anatomy` (core) | `sdk/` | `<branes/anatomy/...>` | what names no vehicle | yaml-cpp |
| `branes::anatomy_mobility` (new) | `mobility/` | `<branes/anatomy/mobility/...>` | what a roaming vehicle adds | core |
| `branes::anatomy_glue` | `glue/` | `<branes/anatomy/glue/...>` (unchanged) | the reflex adapters | mobility, reflex |
| `branes::anatomy_mobility_sim` (was `anatomy_sim`) | `sim/` | `<branes/anatomy/sim/...>` (unchanged) | the closed-loop simulator, the SITL and batch loops, the sweep types | mobility, glue, reflex |

**Unchanged:**
- **Names:** every C++ name keeps its namespace (`branes::anatomy`, `::sim`, `::glue`, and the rest). This step moves files, not names.
- **Behavior, log format and channel ids:** racing-drone's outputs stay byte-identical, and old logs replay.

**What a mission changes:**
- its link lines (`anatomy_sim` becomes `anatomy_mobility_sim`; a mobile mission adds `anatomy_mobility`);
- the include paths of the headers that moved. racing-drone's are in its generated bridge headers only.

## The rule for each file

1. **No vehicle concept → core.**
2. **Mixed, with a clean seam → split along the seam.** A clean seam needs no new abstraction: it is separate types in one header, or a template that is already generic.
3. **Mixed, without a clean seam → mobility, whole.** Its neutral part is extracted when a non-mobile client needs it. That is ADR 0001's Decision 3: the client drives the generalization. Each such file is listed below.

## Where each file goes

**Core** (`sdk/include/branes/anatomy/`):
- **The rest, unchanged:**
  - `result.hpp`, `anatomy.hpp`, `util/{bytes,sha256,splitmix64}.hpp`;
  - `msg/time.hpp`, `msg/geometry.hpp`;
  - `log/log.hpp`, `compose/profile.hpp`;
  - `config/{error,baseline}.hpp` (with `src/config/baseline.cpp`);
  - `harness/{args,files,parallel,replay}.hpp`;
  - `cmake/composition.cmake`.
- **`msg/camera.hpp`** (split from `msg/sensors.hpp`): `PixelFormat`, `FrameHeader`, `CameraFrame`, `CameraFrameRecord`, and their `valid`, `to_record` and `view_of`. Nothing in a frame names a body.
- **`msg/camera_codec.hpp`** (split from `msg/codec.hpp`): the frame codec, and the codec helpers it uses.
- **`harness/runner.hpp`:** it drops its leftover include of `compose/tick.hpp`.
- **`harness/flight.hpp`:** `run_ticks` drives any `RunnableStack` since step 1. It includes `compose/profile.hpp`, not `tick.hpp`.
- **`slots/slot_context.hpp`, its core half:**
  - `ContextBuilt`;
  - `make_slot`, generic over the context type: `template <class S, class Ctx> make_slot(const Ctx&)`. The call sites are unchanged;
  - `parameter_in`.

**Mobility** (`mobility/include/branes/anatomy/mobility/`):
- **`msg/imu.hpp`** (the IMU half of `msg/sensors.hpp`).
- **Moved whole:** `msg/{control,estimate,truth,fc,daa,intruders,region}.hpp`.
- **`msg/codec.hpp`:** the IMU, truth, state, reference and command codecs. It includes the core frame codec.
- **`compose/tick.hpp`:** `TickOutput`, `MobilityProfile`, `MobileStack`.
- **`config/vehicle.hpp`**, with `src/config/vehicle.cpp`.
- **`slots/context.hpp`** (the mobility half of `slot_context.hpp`): `MissionWorld` (it requires a start pose), `BasicSlotContext`, and `TruthConsumer`.
- **`slots/{neutral,neutral_nulls,scripted,ground_truth_core}.hpp`.**
- **`harness/synthetic.hpp`.**
- **Mixed, moved whole until a client needs the split:**
  - `config/scenario.hpp`, with `src/config/scenario.cpp`. Its schema mechanism (`ScenarioSchema`, the mission's `sim:` keys) is neutral. Its vehicle file, physics and FC settings, randomization and faults are not;
  - `log/dump.hpp`. Its CSV machinery and `ChannelFormat` are neutral. Its fixed formats for IMU, truth, state, reference and command are not.

**Glue:** `glue/` is unchanged. It now links mobility.

**Mobility sim:** `sim/` is unchanged, including `sim/sweep_types.hpp`. Its only client is the closed-loop precision sweep.

**The private YAML helpers** (`sdk/src/config/parse.hpp`) stay in core's `src/`. Mobility's two sources include them through a private include directory, which is named in `mobility/CMakeLists.txt`.

## Departures from ADR 0001's table, recorded in the ADR (2b)

1. **The reflex glue is its own target** (`anatomy_glue`, on mobility), not part of `mobility_sim`. A flight stack uses `CascadedPid` without the simulator, and racing-drone's flight builds link the glue alone today.
2. **The flight harness is in core.** Since step 1, `run_ticks` drives any stack.
3. **The scalar-type sweep stays in `mobility_sim`.** Its only client runs the closed loop. It moves to core when a non-mobile client sweeps scalar types.
4. **The scenario and the log dump are in mobility, whole** (rule 3).
5. **`log::Channel` keeps every enumerator in core,** including the mobility profile's channels (imu, camera, state, reference, command). The enum is the registry of channel ids; the ids never change meaning. Payload meaning belongs to the profile.

This also settles step 1's open question. A profile that ships in anatomy registers its channels in the enum. A mission's channels stay outside it, named to the log reader, as racing-drone's `oracle_gates` are today.

## Checking the layering

- **Each library's include directory holds only that library's headers.** A core header that includes a mobility header fails to compile wherever core is built alone.
- **A new build-time check, `anatomy_header_check()` (`cmake/`).** For every header of every library, it generates one translation unit that includes only that header. It compiles each unit against only that library's link dependencies. The units are an object library in `all`, so the check:
  - catches a layering break even in a header that no test includes;
  - proves every header is self-contained.
- **The tests follow the libraries:**
  - `tests/sdk/` (core) links `branes::anatomy` only;
  - a new `tests/mobility/` links `branes::anatomy_mobility`;
  - `tests/sim/` and `tests/precision/` link `anatomy_mobility_sim`.

  Mixed test files are split by `TEST_CASE`, unchanged: `config.cpp`, `codec.cpp`, `neutral_harness.cpp` (its dump case), and `profile.cpp` (its two mobility `static_assert`s).

## The PRs

| PR | Repo | Content |
|---|---|---|
| **2a** | anatomy | The splits in place, with no target change: <ul><li>`msg/sensors.hpp` into `msg/camera.hpp` and `msg/imu.hpp`</li><li>`msg/codec.hpp` into the camera codec and the rest</li><li>`slot_context.hpp` into its core and mobility halves, with `make_slot` generic over the context</li><li>the runner's and flight harness's includes</li><li>this plan; step 1's plan marked done</li></ul> |
| **2b** | anatomy | The libraries: <ul><li>`mobility/` and its target, and the files moved there</li><li>`anatomy_sim` renamed `anatomy_mobility_sim`; glue relinked</li><li>`anatomy_header_check()`; the tests regrouped</li><li>ADR 0001 amended with the departures; the README and the repository-layout page</li></ul> Breaking (`feat!`); release-please proposes v0.3.0 |
| **2c** | racing-drone | <ul><li>Pin v0.3.0; switch the link lines</li><li>`tools/gen_anatomy_bridges.py` learns the mobility library, and the bridges are regenerated: a new `anatomy_mobility.hpp`, which racing's headers include where they use mobility names</li><li>Check byte-identity and log compatibility</li></ul> |

2a and 2b are released together as v0.3.0. Between them, anatomy's main is consistent, but no consumer pins it.

## Acceptance (stated before any test is run)

**anatomy (2a and 2b):**
1. The ctest tests are the same 106, by name, before and after (`ctest -N`). Only their files and includes change.
2. After 2b, `anatomy_header_check()` compiles every header of every library alone.
3. After 2b, no core header includes a mobility, glue or sim header. The header check builds core without them.
4. CI is green on every preset (gcc and clang, Debug and Release, MSVC), with clang-format 18 and commitlint clean, and `framework.names_no_mission` still passes.

**racing-drone (2c):**

5. Its 132 tests pass, on gcc and clang.
6. All 39 harness outputs are byte-identical to the pre-split baseline (main f85ff73), compared with `tools/compare_outputs.sh`.
7. Log compatibility: the f85ff73 logs replay and verify.

## Risks

- **Two include roots with one prefix.** `branes/anatomy/` (core) and `branes/anatomy/mobility/` sit in different directories under one prefix. A header put in the wrong one is caught by the header check, not by the file layout.
- **Generic `make_slot`.** It now takes any context type. A slot whose `from_context` takes a different context than the one it is called with is default-constructed, as it would be today with any mismatch. The stacks build their slots through one context type, so nothing changes. A test pins it: `make_slot` with a `BasicSlotContext` still builds `CascadedPid` through `from_context`.
- **Size.** 2b moves about 25 files. It is mechanical, and the header check and the byte-identity check guard it.
