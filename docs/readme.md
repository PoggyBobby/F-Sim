# docs

## The guide

| Document | What's in it |
|----------|--------------|
| [What this is](guide/what-this-is.md) | Why an s-diff, what the sim covers and deliberately doesn't, how the pieces fit, why the numbers can be trusted |
| [Known defects](guide/known-defects.md) | The bugs that are live in `main` right now, and which results they invalidate |
| [Getting started](guide/getting-started.md) | Setup, first run, how to read a run directory, what each maneuver measures |
| [Parameters](guide/parameters.md) | The YAML parameter spine, the provenance vocabulary, how to change a number correctly |
| [Controller](guide/controller.md) | The two update entry points, the limits chain, `--perfect-state` as a debugging tool |
| [SIL](guide/sil.md) | Running the real VCU firmware in the loop: host build, patches, mA → torque |
| [Conventions](guide/conventions.md) | ISO 8855 axes, wheel order, the 8-state vector and its index constants |
| [Interop with VehicleSim](guide/interop-vehiclesim.md) | Scope map, convention and unit mismatches, how to compare a result honestly |
| [File map](guide/file-map.md) | One line per module — what owns what |
| [Glossary](guide/glossary.md) | s-diff, TV, slip ratio/angle, friction ellipse, APPS/BPS, SIL, VCU, TTC |

## Reference

| Document | What it is | Authoritative on |
|----------|------------|------------------|
| [BREAKDOWN.md](BREAKDOWN.md) | Full technical breakdown — equations, parameter tables, test catalog | Physics and parameter provenance |
| [problems.txt](problems.txt) | Textbook cross-check and code audit | The defect list (see §C) |
| [datasheets/](datasheets/) | Vendor PDFs, project report, generated parameter spreadsheet | — |
