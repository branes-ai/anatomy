# Repository Layout

```text
anatomy/
├── cmake/        # deps, warnings, test registration (compile_all), composition (a mission's baselines)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── glue/         # branes::anatomy_glue: adapters from reflex components to the slots
├── sdk/          # branes::anatomy: contracts (msg/), slots, compose, config, log, harness, util
├── sim/          # branes::anatomy_sim: the closed-loop simulator on the reflex dynamics
└── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
```

The [documentation map](../README.md) lists every document.
