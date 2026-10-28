# Repository Layout

```text
anatomy/
├── cmake/        # deps, warnings, test registration (compile_all)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── sdk/          # branes::anatomy: the framework library (sdk/include/branes/anatomy/)
└── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp
```

With the move from racing-drone ([plan](../plan/move-from-racing-drone/README.md)), `sdk/` fills with the contracts, slots, configs, log and harness, and two more libraries join:
- `glue/`: the reflex adapters, as `branes::anatomy_glue`;
- `sim/`: the closed-loop simulator, as `branes::anatomy_sim`.

The [documentation map](../README.md) lists every document.
