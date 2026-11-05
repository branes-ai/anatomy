# Architecture decision records

Format follows cortex `docs/adr/template.md`: Status / Date / Related, Context, Decision, Consequences, and a closing *Current state* line.

Anatomy was created by [racing-drone ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md), which moved the mission framework out of racing-drone. Decisions about anatomy itself are recorded here.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-core-profiles-multi-node.md) | A mission-neutral core with mobility and sensing profiles, and a multi-node layer in the core | Accepted |
| [0002](0002-third-party-license-compatibility.md) | Third-party license compatibility (MIT; permissive dependencies only) | Accepted (amended) |
| [0003](0003-no-gpl-clean-room-policy.md) | No GPL code for any purpose; clean-room from papers | Accepted |
| [0004](0004-domain-flow-composition.md) | Missions compose operators over domains in a dependence graph, not slots | Accepted |
| [0005](0005-application-tier.md) | An application tier over the checked graph, developed on graph patterns (MLP, Kalman filter, DSP pipeline) in a workbench before the missions | Proposed |
