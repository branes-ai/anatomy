# anatomy

The Branes.AI mission framework: the body plan every mission composes its vehicle with.

The platform repos are the nervous system: `cortex` perceives and plans, `reflex` controls and keeps the vehicle safe, and `synapse` runs learned models. `anatomy` is the body they are assembled into. It holds the mechanism every mission repo (`racing-drone`, `scout-drone`, ...) builds on:
- the data contracts;
- the slot concepts and their composition;
- the configs;
- the replay log;
- the harness;
- the closed-loop simulator.

It holds no algorithms and no mission.

The framework moved here from racing-drone ([ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md); [plan](docs/plan/move-from-racing-drone/README.md)). It builds three libraries:
- `branes::anatomy`: the contracts, slots, composition, configs, log and harness;
- `branes::anatomy_glue`: the adapters from reflex components to the slots;
- `branes::anatomy_sim`: the closed-loop simulator on the reflex dynamics.

The documentation in [`docs/`](docs/README.md) is published at <https://branes-ai.github.io/anatomy/>.

## Building and testing

`anatomy` is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

## License

MIT; see [LICENSE](LICENSE).
