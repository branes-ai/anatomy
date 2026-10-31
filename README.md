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

## Where anatomy fits

```text
+--------------+ +-------------+
| racing-drone | | scout-drone | ...                                     Mission
+--------------+ +-------------+
        |               |
        v               v
+---------------------------------------------------------------------+
|                            Branes Anatomy                           |  Mission
| data contracts, slots and composition, configs, replay log,         |  framework
| harness, closed-loop simulator                                      |
+---------------------------------------------------------------------+
           |                       |                       |
           v                       v                       v
+---------------------+ +---------------------+ +---------------------+
|    Branes Cortex    | |   Branes Synapse    | |    Branes Reflex    |  Platform
| perception (VIO,    | | tensor graph engine | | stabilization, MPC, |
| SLAM), VLA, world   | | for GGUF models,    | | state propagation,  |
| model, planning     | | static memory plan  | | fault recovery      |
+---------------------+ +---------------------+ +---------------------+
           |                       |                       |
           v                       v                       v
+---------------------------------+ +---------------------------------+
|              MTL5               | |            Universal            |  Math
| high-performance linear algebra | |       computer arithmetic       |
+---------------------------------+ +---------------------------------+
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
+-----------+ +-------------------+ +-------------------+ +-----------+
|    CPU    | |     CPU + GPU     | |     CPU + KPU     | |    KPU    |  Hardware
+-----------+ +-------------------+ +-------------------+ +-----------+
```

The Branes.AI stack has five tiers. This repo is the second:

- **Mission**: one repo per vehicle and mission. `racing-drone` and `scout-drone` are the two that stand up the architecture; more follow (`interceptor-drone`, `isr-drone`, `nano-drone`, `counter-uas`). A mission repo owns its task: its world, the slots that see it, its stack and scorer, and its data. Algorithms do not live there.
- **Mission framework**: [`anatomy`](https://github.com/branes-ai/anatomy), the body plan every mission composes its vehicle with: the data contracts, the slot concepts and their composition, configs, the replay log, the harness, and the closed-loop simulator. It holds no algorithms and no mission.
- **Platform**: three sister repos, [`cortex`](https://github.com/branes-ai/cortex), [`synapse`](https://github.com/branes-ai/synapse), and [`reflex`](https://github.com/branes-ai/reflex), that provide the functionality and services a mission is built from. None of them sits above the others.
- **Math**: [MTL5](https://github.com/stillwater-sc/mtl5) provides high-performance linear algebra and [Universal](https://github.com/stillwater-sc/universal) provides computer arithmetic (posits, intervals, mixed precision, hardware-aligned types).
- **Hardware**: the configurations the stack targets: CPU, CPU + GPU, CPU + KPU, and KPU. Today the stack runs on CPU; GPU and KPU support are planned, and the KPU is a stub until the silicon ships.

Arrows point from a tier to the tier below it that it builds on. They show how the tiers relate, not the dependencies of individual repos: each repo's build declares the libraries it actually uses. A mission also uses platform components directly: it composes them into anatomy's slots. Anatomy itself builds on the platform where its simulator needs it (the `reflex` dynamics).

## The libraries

The framework moved here from racing-drone ([ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md); [plan](docs/plan/move-from-racing-drone/README.md)). It builds a core and the mobility profile ([ADR 0001](docs/adr/0001-core-profiles-multi-node.md)):
- `branes::anatomy` (core): what every kind of mission needs. It holds time and geometry, the camera frame, the stack profile, slot construction, the baseline config, the replay log, and the harness;
- `branes::anatomy_yaml`: the YAML helpers for config loaders, anatomy's and a mission's (its world file). Its header is `<branes/anatomy/config/yaml.hpp>`, and it is the only library whose headers include yaml-cpp;
- `branes::anatomy_mobility`: what a roaming vehicle adds. It holds the IMU, estimate, reference, command and truth contracts, `MobilityProfile`, the vehicle and scenario configs, the vehicle's slots, and the log dump. Its headers are under `<branes/anatomy/mobility/...>`;
- `branes::anatomy_glue`: the adapters from reflex components to the slots;
- `branes::anatomy_mobility_sim`: the closed-loop simulator on the reflex dynamics.

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
