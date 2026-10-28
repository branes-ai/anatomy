# Introduction

`anatomy` is the Branes.AI mission framework: the mechanism every mission composes its vehicle with.

## Why a mission framework

The Branes.AI platform has one repo per mission: `racing-drone`, `scout-drone`, `nano-drone`, `isr-drone`, `interceptor-drone`, `counter-uas`. Each composes components from the platform repos (`cortex`, `synapse`, `reflex`) into a vehicle that performs its mission.

The way a mission composes, runs, logs, replays, simulates and scores its vehicle is the same for every mission. That framework was first built inside racing-drone. [ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md) moves it here, so every mission builds on one copy rather than its own.

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

## How it arrives

The framework moves here from racing-drone in three PRs: this skeleton, the move itself, and racing-drone's switch to consume it. The [plan](../plan/move-from-racing-drone/README.md) has the details, and racing-drone's outputs stay byte-identical throughout.

Next: [build and test](build-and-test.md).
