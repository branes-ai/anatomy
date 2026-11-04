# Changelog

## [0.11.0](https://github.com/branes-ai/anatomy/compare/v0.10.4...v0.11.0) (2026-11-04)


### Features

* **flow:** trace every edge a stack writes, and compare two traces ([#73](https://github.com/branes-ai/anatomy/issues/73)) ([6bcf31d](https://github.com/branes-ai/anatomy/commit/6bcf31d2ce9c087848d32d5332d8f7071a1deb65))

## [0.10.4](https://github.com/branes-ai/anatomy/compare/v0.10.3...v0.10.4) (2026-11-04)


### Tests

* **flow:** check that meaning-preserving changes keep the records ([#71](https://github.com/branes-ai/anatomy/issues/71)) ([344fa4b](https://github.com/branes-ai/anatomy/commit/344fa4bb0ab6daf0e42e89c5f6e15362fdd381c5))

## [0.10.3](https://github.com/branes-ai/anatomy/compare/v0.10.2...v0.10.3) (2026-11-04)


### Tests

* **flow:** mutate valid graphs to break each invariant ([#69](https://github.com/branes-ai/anatomy/issues/69)) ([b95d8de](https://github.com/branes-ai/anatomy/commit/b95d8de2b70afe75f2e007a15adb3b870d34f5b5))

## [0.10.2](https://github.com/branes-ai/anatomy/compare/v0.10.1...v0.10.2) (2026-11-04)


### Tests

* **flow:** run random graphs against a reference interpreter ([#67](https://github.com/branes-ai/anatomy/issues/67)) ([356fe54](https://github.com/branes-ai/anatomy/commit/356fe544efe340b6da77a4f1f5fcfd26df3e0791))

## [0.10.1](https://github.com/branes-ai/anatomy/compare/v0.10.0...v0.10.1) (2026-11-04)


### Documentation

* **plan:** test the flow graph beyond equality with the slot stack ([#65](https://github.com/branes-ai/anatomy/issues/65)) ([65f2644](https://github.com/branes-ai/anatomy/commit/65f26449e33d31377f509a22a696961d8b425758))

## [0.10.0](https://github.com/branes-ai/anatomy/compare/v0.9.3...v0.10.0) (2026-11-04)


### Features

* **flow:** print a graph as text or as a mermaid flowchart ([#62](https://github.com/branes-ai/anatomy/issues/62)) ([382f043](https://github.com/branes-ai/anatomy/commit/382f043ff32f0ac79175ce136db4499e44629c83))

## [0.9.3](https://github.com/branes-ai/anatomy/compare/v0.9.2...v0.9.3) (2026-11-03)


### Documentation

* add the baseline file's reference and a design guide ([#60](https://github.com/branes-ai/anatomy/issues/60)) ([f6ad95d](https://github.com/branes-ai/anatomy/commit/f6ad95d2b1df373897ada9a2d5d4b4c9306c31b5))

## [0.9.2](https://github.com/branes-ai/anatomy/compare/v0.9.1...v0.9.2) (2026-11-03)


### Build System

* silence yaml-cpp's c4267 warnings on msvc ([#58](https://github.com/branes-ai/anatomy/issues/58)) ([41a0637](https://github.com/branes-ai/anatomy/commit/41a0637c07ca138d7319c08dd6e25c3d889e0695))

## [0.9.1](https://github.com/branes-ai/anatomy/compare/v0.9.0...v0.9.1) (2026-11-03)


### Bug Fixes

* **build:** fetch a moved pin in an existing build tree ([#57](https://github.com/branes-ai/anatomy/issues/57)) ([e2eb4f0](https://github.com/branes-ai/anatomy/commit/e2eb4f0b8155f638a0f98c3f976f696a93c0ef49))


### Documentation

* **adr:** mark adr 0004 step 2 complete ([#55](https://github.com/branes-ai/anatomy/issues/55)) ([5d92ce7](https://github.com/branes-ai/anatomy/commit/5d92ce767145ab9d73ce60de7aa4dd79ef1930df))

## [0.9.0](https://github.com/branes-ai/anatomy/compare/v0.8.0...v0.9.0) (2026-11-03)


### Features

* **flow:** the cpu stack, which runs the checked graph ([#52](https://github.com/branes-ai/anatomy/issues/52)) ([c2a9e5a](https://github.com/branes-ai/anatomy/commit/c2a9e5abedc697be2bff88bc8fbd27d7e88b2259))
* **log:** compare two logs, and the log_compare tool ([#54](https://github.com/branes-ai/anatomy/issues/54)) ([e5e2f2f](https://github.com/branes-ai/anatomy/commit/e5e2f2f6940ca5c2d1d58a64c4762e75b689eee9))

## [0.8.0](https://github.com/branes-ai/anatomy/compare/v0.7.2...v0.8.0) (2026-11-03)


### Features

* **flow:** the graph builder, and its checks ([#50](https://github.com/branes-ai/anatomy/issues/50)) ([874caf4](https://github.com/branes-ai/anatomy/commit/874caf4ee6a5c2a49b2465224d6f528d462036cc)), closes [#47](https://github.com/branes-ai/anatomy/issues/47)

## [0.7.2](https://github.com/branes-ai/anatomy/compare/v0.7.1...v0.7.2) (2026-11-02)


### Documentation

* **plan:** plan adr 0004 step 2, the graph and the generator ([#48](https://github.com/branes-ai/anatomy/issues/48)) ([4b86d98](https://github.com/branes-ai/anatomy/commit/4b86d98d08b21bf55a0bd15f0699c60cd0080b30)), closes [#47](https://github.com/branes-ai/anatomy/issues/47)

## [0.7.1](https://github.com/branes-ai/anatomy/compare/v0.7.0...v0.7.1) (2026-11-02)


### Documentation

* **adr:** mark adr 0004 step 1 complete ([#45](https://github.com/branes-ai/anatomy/issues/45)) ([ab316bd](https://github.com/branes-ai/anatomy/commit/ab316bdd929b946138487eebd4969c1e6b2f0fca))

## [0.7.0](https://github.com/branes-ai/anatomy/compare/v0.6.6...v0.7.0) (2026-11-02)


### Features

* **flow:** domain types and operator signatures ([#43](https://github.com/branes-ai/anatomy/issues/43)) ([a85b77e](https://github.com/branes-ai/anatomy/commit/a85b77ed012b406d9c194f95b5998e05ad8feeeb)), closes [#40](https://github.com/branes-ai/anatomy/issues/40)

## [0.6.6](https://github.com/branes-ai/anatomy/compare/v0.6.5...v0.6.6) (2026-11-02)


### Documentation

* **adr:** accept adr 0004, and plan its step 1 ([#41](https://github.com/branes-ai/anatomy/issues/41)) ([95d893b](https://github.com/branes-ai/anatomy/commit/95d893be5acc5b60f1dfecfe0228acee6add04d5))

## [0.6.5](https://github.com/branes-ai/anatomy/compare/v0.6.4...v0.6.5) (2026-11-02)


### Documentation

* **adr:** scoreboard baselines vary inside composites, research ones the graph ([#38](https://github.com/branes-ai/anatomy/issues/38)) ([a0f900c](https://github.com/branes-ai/anatomy/commit/a0f900c5032064644e1aa7fb6e00a3307b18eaf3))

## [0.6.4](https://github.com/branes-ai/anatomy/compare/v0.6.3...v0.6.4) (2026-11-02)


### Documentation

* the domain flow computational model, and adr 0004 to adopt it ([#36](https://github.com/branes-ai/anatomy/issues/36)) ([c08dd04](https://github.com/branes-ai/anatomy/commit/c08dd04cd9f8864dcdacdd5193fb4580ec4376ba))

## [0.6.3](https://github.com/branes-ai/anatomy/compare/v0.6.2...v0.6.3) (2026-11-01)


### Build System

* **deps:** reflex v0.10.0, which brings the trace writer's scene ([#34](https://github.com/branes-ai/anatomy/issues/34)) ([72e7fe6](https://github.com/branes-ai/anatomy/commit/72e7fe6aecf9209fe38ed38eac70213d07ec9282))

## [0.6.2](https://github.com/branes-ai/anatomy/compare/v0.6.1...v0.6.2) (2026-11-01)


### Build System

* docs-kit v0.6.0 for the docs site ([#31](https://github.com/branes-ai/anatomy/issues/31)) ([7b5f847](https://github.com/branes-ai/anatomy/commit/7b5f84792b7ce4988306769153e50d078a67a05d))


### Documentation

* **adr:** add the third-party data rule to the license ADR ([#32](https://github.com/branes-ai/anatomy/issues/32)) ([12a7aa1](https://github.com/branes-ai/anatomy/commit/12a7aa1a807e7e79fbc9e4f43ba76e7be30cbafa))

## [0.6.1](https://github.com/branes-ai/anatomy/compare/v0.6.0...v0.6.1) (2026-11-01)


### Documentation

* record the license policy as ADRs 0002 and 0003 ([#29](https://github.com/branes-ai/anatomy/issues/29)) ([4790f76](https://github.com/branes-ai/anatomy/commit/4790f764041692c7507c7859d0818384d85d5f37))

## [0.6.0](https://github.com/branes-ai/anatomy/compare/v0.5.0...v0.6.0) (2026-10-31)


### Features

* run a stack whose profile adds inputs in the mobile loops ([#26](https://github.com/branes-ai/anatomy/issues/26)) ([39c0c89](https://github.com/branes-ai/anatomy/commit/39c0c895c65cfb266a5f9c7b084e4865bbcdeedb))


### Build System

* docs-kit v0.4.1 for the docs site ([#27](https://github.com/branes-ai/anatomy/issues/27)) ([d3013c4](https://github.com/branes-ai/anatomy/commit/d3013c4c81c51ad7fcc3df87a3b264a08f434384))

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
