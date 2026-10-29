# Introduction

`anatomy` is the Branes.AI mission framework: the mechanism every mission composes its vehicle with.

## Why a mission framework

The Branes.AI platform has one repo per mission: `racing-drone`, `scout-drone`, `nano-drone`, `isr-drone`, `interceptor-drone`, `counter-uas`. Each composes components from the platform repos (`cortex`, `synapse`, `reflex`) into a vehicle that performs its mission.

The way a mission composes, runs, logs, replays, simulates and scores its vehicle is the same for every mission. That framework was first built inside racing-drone. [ADR 0008](https://branes-ai.github.io/architecture/racing-drone/adr/0008-anatomy-mission-framework/) moved it here, so every mission builds on one copy rather than its own.

## What anatomy holds, and what a mission supplies

| anatomy | A mission |
|---|---|
| The data contracts: time, geometry, sensors, the state estimate, control, DAA | Its world (racing: the course of gates) |
| The mission-neutral slot concepts: controller, safety filter, DAA tracker, FC bridge | The slots that see its task (racing: estimator, gate detector, planner) |
| Slot construction and the baseline generator | Its slot list, and its stack |
| The configs: vehicle, scenario, baseline | Its world file and its scenario parameters |
| The replay log, with a channel table missions extend | Its own channels (racing: `oracle_gates`) |
| The harness: the runner, replay, the batch pool | Its harness programs |
| The closed-loop simulator on the reflex dynamics | Its oracle sensors and its scorer |

Anatomy holds no algorithms: those are in the platform repos. It holds no mission either: those are in the mission repos.

## How it arrived

The framework moved here from racing-drone in three PRs: the skeleton ([#1](https://github.com/branes-ai/anatomy/pull/1)) and the move itself ([#3](https://github.com/branes-ai/anatomy/pull/3)), released as v0.1.0, and racing-drone's switch to consume it ([racing-drone#66](https://github.com/branes-ai/racing-drone/pull/66)). Racing-drone's outputs stayed byte-identical throughout; the [plan](../plan/move-from-racing-drone/README.md) has the details.

Since then, [ADR 0001](../adr/0001-core-profiles-multi-node.md) has made the core generic over a stack profile ([plan](../plan/generalize-core/README.md), v0.2.0) and split it by kind of mission ([plan](../plan/split-libraries/README.md), v0.3.0): `branes::anatomy` for every mission, `branes::anatomy_mobility` for a mobile vehicle, and `branes::anatomy_mobility_sim` for its simulator. Racing-drone builds on v0.3.0, byte-identical again. The next client is scout-drone, step 3 of ADR 0008.

Next: [build and test](build-and-test.md).
