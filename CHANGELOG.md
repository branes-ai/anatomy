# Changelog

## [0.10.4](https://github.com/branes-ai/anatomy/compare/v0.10.3...v0.10.4) (2026-11-04)


### Tests

* **flow:** check that meaning-preserving changes keep the records ([#71](https://github.com/branes-ai/anatomy/issues/71)) ([8ed76d0](https://github.com/branes-ai/anatomy/commit/8ed76d0733130d3c26855896084d1c70372052dc))

## [0.10.3](https://github.com/branes-ai/anatomy/compare/v0.10.2...v0.10.3) (2026-11-04)


### Tests

* **flow:** mutate valid graphs to break each invariant ([#69](https://github.com/branes-ai/anatomy/issues/69)) ([2fe2448](https://github.com/branes-ai/anatomy/commit/2fe2448ea75582d95c9a951381a2d8b1a203c7f7))

## [0.10.2](https://github.com/branes-ai/anatomy/compare/v0.10.1...v0.10.2) (2026-11-04)


### Tests

* **flow:** run random graphs against a reference interpreter ([#67](https://github.com/branes-ai/anatomy/issues/67)) ([ff22176](https://github.com/branes-ai/anatomy/commit/ff221768b6fb5a0fd01833f6bc04ac9081b2f0a4))

## [0.10.1](https://github.com/branes-ai/anatomy/compare/v0.10.0...v0.10.1) (2026-11-04)


### Documentation

* **plan:** test the flow graph beyond equality with the slot stack ([#65](https://github.com/branes-ai/anatomy/issues/65)) ([aa9386f](https://github.com/branes-ai/anatomy/commit/aa9386f97bb56cad42d0aca85b5bfd9d24756866))

## [0.10.0](https://github.com/branes-ai/anatomy/compare/v0.9.3...v0.10.0) (2026-11-04)


### Features

* **flow:** print a graph as text or as a mermaid flowchart ([#62](https://github.com/branes-ai/anatomy/issues/62)) ([ca08773](https://github.com/branes-ai/anatomy/commit/ca08773938c1edaf77a153265009cd03b57f0482))

## [0.9.3](https://github.com/branes-ai/anatomy/compare/v0.9.2...v0.9.3) (2026-11-03)


### Documentation

* add the baseline file's reference and a design guide ([#60](https://github.com/branes-ai/anatomy/issues/60)) ([e69ea6e](https://github.com/branes-ai/anatomy/commit/e69ea6e832ff20f46aa92ffeac641ff6f67dec36))

## [0.9.2](https://github.com/branes-ai/anatomy/compare/v0.9.1...v0.9.2) (2026-11-03)


### Build System

* silence yaml-cpp's c4267 warnings on msvc ([#58](https://github.com/branes-ai/anatomy/issues/58)) ([9cc0409](https://github.com/branes-ai/anatomy/commit/9cc0409d07f8cd080bbf6974696fcb825598adf5))

## [0.9.1](https://github.com/branes-ai/anatomy/compare/v0.9.0...v0.9.1) (2026-11-03)


### Bug Fixes

* **build:** fetch a moved pin in an existing build tree ([#57](https://github.com/branes-ai/anatomy/issues/57)) ([5011384](https://github.com/branes-ai/anatomy/commit/501138493e2d68e264f7f03f2395f8f0637923b2))


### Documentation

* **adr:** mark adr 0004 step 2 complete ([#55](https://github.com/branes-ai/anatomy/issues/55)) ([c3757c2](https://github.com/branes-ai/anatomy/commit/c3757c282aa36c59db1d42b16ed0154ab41af1cc))

## [0.9.0](https://github.com/branes-ai/anatomy/compare/v0.8.0...v0.9.0) (2026-11-03)


### Features

* **flow:** the cpu stack, which runs the checked graph ([#52](https://github.com/branes-ai/anatomy/issues/52)) ([91a289d](https://github.com/branes-ai/anatomy/commit/91a289d9b91b42f55dfd3be7e6530269f42edcd2))
* **log:** compare two logs, and the log_compare tool ([#54](https://github.com/branes-ai/anatomy/issues/54)) ([605c7ba](https://github.com/branes-ai/anatomy/commit/605c7bae25752a3361f9f182b56144add4a86a62))

## [0.8.0](https://github.com/branes-ai/anatomy/compare/v0.7.2...v0.8.0) (2026-11-03)


### Features

* **flow:** the graph builder, and its checks ([#50](https://github.com/branes-ai/anatomy/issues/50)) ([1297f16](https://github.com/branes-ai/anatomy/commit/1297f165cad252adabf2f5adfb2c05d6c7ad5931)), closes [#47](https://github.com/branes-ai/anatomy/issues/47)

## [0.7.2](https://github.com/branes-ai/anatomy/compare/v0.7.1...v0.7.2) (2026-11-02)


### Documentation

* **plan:** plan adr 0004 step 2, the graph and the generator ([#48](https://github.com/branes-ai/anatomy/issues/48)) ([adfbf56](https://github.com/branes-ai/anatomy/commit/adfbf5641c5093c21a76e32579073eb8d8efcfef)), closes [#47](https://github.com/branes-ai/anatomy/issues/47)

## [0.7.1](https://github.com/branes-ai/anatomy/compare/v0.7.0...v0.7.1) (2026-11-02)


### Documentation

* **adr:** mark adr 0004 step 1 complete ([#45](https://github.com/branes-ai/anatomy/issues/45)) ([369c48a](https://github.com/branes-ai/anatomy/commit/369c48a9ebc812f95ae9ea76d21e85ef75aab4cc))

## [0.7.0](https://github.com/branes-ai/anatomy/compare/v0.6.6...v0.7.0) (2026-11-02)


### Features

* **flow:** domain types and operator signatures ([#43](https://github.com/branes-ai/anatomy/issues/43)) ([cc70981](https://github.com/branes-ai/anatomy/commit/cc7098133c90e022fd85331c80f7c14dfec001fb)), closes [#40](https://github.com/branes-ai/anatomy/issues/40)

## [0.6.6](https://github.com/branes-ai/anatomy/compare/v0.6.5...v0.6.6) (2026-11-02)


### Documentation

* **adr:** accept adr 0004, and plan its step 1 ([#41](https://github.com/branes-ai/anatomy/issues/41)) ([c41b32a](https://github.com/branes-ai/anatomy/commit/c41b32a2a04ccc360f72457f0e572301ae18aff0))

## [0.6.5](https://github.com/branes-ai/anatomy/compare/v0.6.4...v0.6.5) (2026-11-02)


### Documentation

* **adr:** scoreboard baselines vary inside composites, research ones the graph ([#38](https://github.com/branes-ai/anatomy/issues/38)) ([b1c8fd4](https://github.com/branes-ai/anatomy/commit/b1c8fd403b4f29b53d31a8dbf5ae81180cd0a397))

## [0.6.4](https://github.com/branes-ai/anatomy/compare/v0.6.3...v0.6.4) (2026-11-02)


### Documentation

* the domain flow computational model, and adr 0004 to adopt it ([#36](https://github.com/branes-ai/anatomy/issues/36)) ([9402c30](https://github.com/branes-ai/anatomy/commit/9402c307aa899ea4bb7dc0d76a6de456df70d6f6))

## [0.6.3](https://github.com/branes-ai/anatomy/compare/v0.6.2...v0.6.3) (2026-11-01)


### Build System

* **deps:** reflex v0.10.0, which brings the trace writer's scene ([#34](https://github.com/branes-ai/anatomy/issues/34)) ([8a40a09](https://github.com/branes-ai/anatomy/commit/8a40a09b30524209b111b742ac9eb0706b04f1fa))

## [0.6.2](https://github.com/branes-ai/anatomy/compare/v0.6.1...v0.6.2) (2026-11-01)


### Build System

* docs-kit v0.6.0 for the docs site ([#31](https://github.com/branes-ai/anatomy/issues/31)) ([20af284](https://github.com/branes-ai/anatomy/commit/20af28405161e1cec4586d5004747867b4288023))


### Documentation

* **adr:** add the third-party data rule to the license ADR ([#32](https://github.com/branes-ai/anatomy/issues/32)) ([acae074](https://github.com/branes-ai/anatomy/commit/acae0749f262c5ed1a335ddeac30f2bd40cb336c))

## [0.6.1](https://github.com/branes-ai/anatomy/compare/v0.6.0...v0.6.1) (2026-11-01)


### Documentation

* record the license policy as ADRs 0002 and 0003 ([#29](https://github.com/branes-ai/anatomy/issues/29)) ([5c3ff93](https://github.com/branes-ai/anatomy/commit/5c3ff93e888c68d4122cca8d3e81d6349a375e2e))

## [0.6.0](https://github.com/branes-ai/anatomy/compare/v0.5.0...v0.6.0) (2026-10-31)


### Features

* run a stack whose profile adds inputs in the mobile loops ([#26](https://github.com/branes-ai/anatomy/issues/26)) ([d359987](https://github.com/branes-ai/anatomy/commit/d359987f2da5693a42a613d54de171b0b54f3649))


### Build System

* docs-kit v0.4.1 for the docs site ([#27](https://github.com/branes-ai/anatomy/issues/27)) ([7cb9b62](https://github.com/branes-ai/anatomy/commit/7cb9b621b6ee301e7dca3b01dd40a5e328e51af8))

## [0.5.0](https://github.com/branes-ai/anatomy/compare/v0.4.0...v0.5.0) (2026-10-31)


### Features

* derive a run's random streams in the core, for a mission's own ([#24](https://github.com/branes-ai/anatomy/issues/24)) ([66ab63e](https://github.com/branes-ai/anatomy/commit/66ab63ef01a4635edf3326682d5970d9695dd5d8))
* export the YAML helpers for a mission's config loaders ([#23](https://github.com/branes-ai/anatomy/issues/23)) ([62b7dbf](https://github.com/branes-ai/anatomy/commit/62b7dbf049f1361779cb394b0fc3db4f9b2a5d20))


### Continuous Integration

* give each run on main a concurrency group of its own ([#21](https://github.com/branes-ai/anatomy/issues/21)) ([9f3a875](https://github.com/branes-ai/anatomy/commit/9f3a875313c0cbb8cafa7b0c0b610799748228c6))

## [0.4.0](https://github.com/branes-ai/anatomy/compare/v0.3.2...v0.4.0) (2026-10-30)


### Features

* **docs-site:** build the site with docs-kit v0.1.0 ([#19](https://github.com/branes-ai/anatomy/issues/19)) ([7bdff34](https://github.com/branes-ai/anatomy/commit/7bdff34ad6ea5ca2d4d0dd8e6404e15ead2c62c9))

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
