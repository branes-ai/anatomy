# Plan: the framework moves here from racing-drone

Issue: [racing-drone#65](https://github.com/branes-ai/racing-drone/issues/65). Status: in progress (PR 2a, the skeleton).

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

## PR 2a

The skeleton, laid out like reflex and racing-drone:
- the root config, copied from racing-drone (`.clang-format`, `.editorconfig`, `CMakePresets.json`, `commitlint.config.cjs`, release-please);
- the workflows: CI (clang-format, Linux GCC and Clang in Debug and Release, Windows MSVC, the docs site), commitlint, release-please, and the docs deploy;
- `cmake/` (warnings, `compile_all`, and deps with Catch2 only for now);
- `sdk/` as a header-only `branes::anatomy`, with one smoke test;
- `docs/`, and the `docs-site/` generated from it.
