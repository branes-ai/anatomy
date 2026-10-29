# Changelog

## [0.3.2](https://github.com/branes-ai/anatomy/compare/v0.3.1...v0.3.2) (2026-10-29)


### Bug Fixes

* **docs-site:** close fences, check github anchors, keep intraword underscores ([#18](https://github.com/branes-ai/anatomy/issues/18)) ([d0780d1](https://github.com/branes-ai/anatomy/commit/d0780d1103bba375e1001fdcca121b8b722e24ac))


### Documentation

* say the framework has arrived, and what came after ([#16](https://github.com/branes-ai/anatomy/issues/16)) ([8d41856](https://github.com/branes-ai/anatomy/commit/8d418568bc3d9ad0ed3bdede1b153c799f0c02f3))

## [0.3.1](https://github.com/branes-ai/anatomy/compare/v0.3.0...v0.3.1) (2026-10-29)


### Documentation

* publish the plans and adrs on the architecture site ([#14](https://github.com/branes-ai/anatomy/issues/14)) ([87b09e0](https://github.com/branes-ai/anatomy/commit/87b09e0e05d0c7e8df1cfc92a9d2040c2cfae3eb)), closes [#10](https://github.com/branes-ai/anatomy/issues/10)

## [0.3.0](https://github.com/branes-ai/anatomy/compare/v0.2.0...v0.3.0) (2026-10-29)


### ⚠ BREAKING CHANGES

* the mobility headers moved to <branes/anatomy/mobility/...>, and a mobile mission links branes::anatomy_mobility; branes::anatomy_sim is now branes::anatomy_mobility_sim.
* msg/sensors.hpp is gone: include msg/camera.hpp or msg/imu.hpp. BasicSlotContext, MissionWorld and TruthConsumer are in slots/context.hpp. The harness headers no longer bring in TickOutput or MobilityProfile; include compose/tick.hpp.

### Features

* split the libraries into core, mobility and mobility_sim ([#13](https://github.com/branes-ai/anatomy/issues/13)) ([d522f0a](https://github.com/branes-ai/anatomy/commit/d522f0ac8bfeb3f8756dbcf2e1b33739f52fbae5))


### Code Refactoring

* split the mixed headers along their core and mobility seams ([#11](https://github.com/branes-ai/anatomy/issues/11)) ([3dfabf4](https://github.com/branes-ai/anatomy/commit/3dfabf4b2e185894e80afd6c2d0256c983ea5155))

## [0.2.0](https://github.com/branes-ai/anatomy/compare/v0.1.1...v0.2.0) (2026-10-29)


### ⚠ BREAKING CHANGES

* RunnableStack requires the stack to name its profile; a mobile mission's stack adds `using profile = MobilityProfile;`.

### Features

* the core's outputs, truth and events take the stack profile's types ([#8](https://github.com/branes-ai/anatomy/issues/8)) ([0df2948](https://github.com/branes-ai/anatomy/commit/0df2948a58261b87aa53099374071503592f95d3))

## [0.1.1](https://github.com/branes-ai/anatomy/compare/v0.1.0...v0.1.1) (2026-10-28)


### Documentation

* **adr:** 0001, a core with mobility and sensing profiles, and a multi-node layer ([#5](https://github.com/branes-ai/anatomy/issues/5)) ([763f9e2](https://github.com/branes-ai/anatomy/commit/763f9e245ae640c7c53942d3a0a58c3f1b169dfe))
* the five-tier stack diagram, with anatomy and scout-drone ([#4](https://github.com/branes-ai/anatomy/issues/4)) ([07420aa](https://github.com/branes-ai/anatomy/commit/07420aa37dcaa5575ef7448abbe649482fad3651))

## 0.1.0 (2026-10-28)


### Features

* the mission framework, moved here from racing-drone ([#3](https://github.com/branes-ai/anatomy/issues/3)) ([788e1af](https://github.com/branes-ai/anatomy/commit/788e1af44c6dbec491512010731a119d5b681514))


### Build System

* the repository skeleton, laid out like reflex and racing-drone ([#1](https://github.com/branes-ai/anatomy/issues/1)) ([8a756ee](https://github.com/branes-ai/anatomy/commit/8a756eed1394acd92c4c507cc873fe867f15dae5))
