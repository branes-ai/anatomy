# Changelog

## [0.5.0](https://github.com/branes-ai/anatomy/compare/v0.4.0...v0.5.0) (2026-10-31)


### Features

* derive a run's random streams in the core, for a mission's own ([#24](https://github.com/branes-ai/anatomy/issues/24)) ([124823f](https://github.com/branes-ai/anatomy/commit/124823f370dd11144f916a17d03ce7e16c336d72))
* export the YAML helpers for a mission's config loaders ([#23](https://github.com/branes-ai/anatomy/issues/23)) ([8651efd](https://github.com/branes-ai/anatomy/commit/8651efd032906fa325bb6a4b8d9e6dd8d8f5bc55))


### Continuous Integration

* give each run on main a concurrency group of its own ([#21](https://github.com/branes-ai/anatomy/issues/21)) ([2bca9c5](https://github.com/branes-ai/anatomy/commit/2bca9c534c254e02148a279713efeab75c5fe1e9))

## [0.4.0](https://github.com/branes-ai/anatomy/compare/v0.3.2...v0.4.0) (2026-10-30)


### Features

* **docs-site:** build the site with docs-kit v0.1.0 ([#19](https://github.com/branes-ai/anatomy/issues/19)) ([0dd6886](https://github.com/branes-ai/anatomy/commit/0dd6886bbc2f27291b1badb735b5f4a49f086ded))

## [0.3.2](https://github.com/branes-ai/anatomy/compare/v0.3.1...v0.3.2) (2026-10-29)


### Bug Fixes

* **docs-site:** close fences, check github anchors, keep intraword underscores ([#18](https://github.com/branes-ai/anatomy/issues/18)) ([52428c3](https://github.com/branes-ai/anatomy/commit/52428c379dd1ea44eba78316e661c2516da3eac3))


### Documentation

* say the framework has arrived, and what came after ([#16](https://github.com/branes-ai/anatomy/issues/16)) ([6fb62b8](https://github.com/branes-ai/anatomy/commit/6fb62b889ca794a285678cfb780cffdefd9bf94e))

## [0.3.1](https://github.com/branes-ai/anatomy/compare/v0.3.0...v0.3.1) (2026-10-29)


### Documentation

* publish the plans and adrs on the architecture site ([#14](https://github.com/branes-ai/anatomy/issues/14)) ([031f860](https://github.com/branes-ai/anatomy/commit/031f8607666f4c60cf53f22685b2bf14a1b54748)), closes [#10](https://github.com/branes-ai/anatomy/issues/10)

## [0.3.0](https://github.com/branes-ai/anatomy/compare/v0.2.0...v0.3.0) (2026-10-29)


### ⚠ BREAKING CHANGES

* the mobility headers moved to <branes/anatomy/mobility/...>, and a mobile mission links branes::anatomy_mobility; branes::anatomy_sim is now branes::anatomy_mobility_sim.
* msg/sensors.hpp is gone: include msg/camera.hpp or msg/imu.hpp. BasicSlotContext, MissionWorld and TruthConsumer are in slots/context.hpp. The harness headers no longer bring in TickOutput or MobilityProfile; include compose/tick.hpp.

### Features

* split the libraries into core, mobility and mobility_sim ([#13](https://github.com/branes-ai/anatomy/issues/13)) ([f81db13](https://github.com/branes-ai/anatomy/commit/f81db13708c604958994d936b5674ef183f40cc6))


### Code Refactoring

* split the mixed headers along their core and mobility seams ([#11](https://github.com/branes-ai/anatomy/issues/11)) ([fe7f3f7](https://github.com/branes-ai/anatomy/commit/fe7f3f7520256b19fdde3fa48558f275a95ff32e))

## [0.2.0](https://github.com/branes-ai/anatomy/compare/v0.1.1...v0.2.0) (2026-10-29)


### ⚠ BREAKING CHANGES

* RunnableStack requires the stack to name its profile; a mobile mission's stack adds `using profile = MobilityProfile;`.

### Features

* the core's outputs, truth and events take the stack profile's types ([#8](https://github.com/branes-ai/anatomy/issues/8)) ([5a32168](https://github.com/branes-ai/anatomy/commit/5a32168cbeee442cd41f34451bdd2979ffbd0e24))

## [0.1.1](https://github.com/branes-ai/anatomy/compare/v0.1.0...v0.1.1) (2026-10-28)


### Documentation

* **adr:** 0001, a core with mobility and sensing profiles, and a multi-node layer ([#5](https://github.com/branes-ai/anatomy/issues/5)) ([6189b7f](https://github.com/branes-ai/anatomy/commit/6189b7fe7fa99b14993ccc76d5997c27b3ac354c))
* the five-tier stack diagram, with anatomy and scout-drone ([#4](https://github.com/branes-ai/anatomy/issues/4)) ([f281cd8](https://github.com/branes-ai/anatomy/commit/f281cd851fe32a752782695041d34cc6ea10b6e3))

## 0.1.0 (2026-10-28)


### Features

* the mission framework, moved here from racing-drone ([#3](https://github.com/branes-ai/anatomy/issues/3)) ([2c6cbde](https://github.com/branes-ai/anatomy/commit/2c6cbde67620e109d9e0b656e95a7c8fe89832ca))


### Build System

* the repository skeleton, laid out like reflex and racing-drone ([#1](https://github.com/branes-ai/anatomy/issues/1)) ([6480fd4](https://github.com/branes-ai/anatomy/commit/6480fd4df2257b486e8c31420a2c57206f55531f))
