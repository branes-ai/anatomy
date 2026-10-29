# Repository Layout

```text
anatomy/
├── cmake/        # deps, warnings, test registration (compile_all), composition (a mission's baselines)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── glue/         # branes::anatomy_glue: adapters from reflex components to the slots
├── mobility/     # branes::anatomy_mobility: a roaming vehicle's contracts, profile, configs, slots, dump
├── sdk/          # branes::anatomy (core): time, geometry, camera, profile, slots, baseline, log, harness, util
├── sim/          # branes::anatomy_mobility_sim: the closed-loop simulator on the reflex dynamics
└── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
```

The core's headers are under `<branes/anatomy/...>` and the mobility profile's under `<branes/anatomy/mobility/...>` ([ADR 0001](../adr/0001-core-profiles-multi-node.md)). Every header compiles alone against its own library: the `headers_*` object libraries in `tests/` check it as part of the default build when the tests are enabled.

The [documentation map](../README.md) lists every document.
