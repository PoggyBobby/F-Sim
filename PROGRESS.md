# Restructure progress

Live checklist. Delete this file before the PR.

- [x] **1. Branch + history merge** — `restructure` off `origin/main`, merged local
      `main` skeleton (README / .gitignore / .gitmodules resolved). *committed `446b9c3`*
- [x] **2. File moves** — pure `git mv` into `model/`, `docs/`, `controllers/`, `sil/`
      + package `__init__.py`. *committed `40942f9`*
- [x] **3. `model/config.py`** — YAML loader: units, derived formulas, `cfg`, `cfg.meta()`
- [x] **4. YAML params** — 13 files, ~68 entries, each with value/unit/symbol/status/what/need/how/why
- [x] **5. `model/params.py`** — dataclasses built from `cfg`
- [x] **6. Split `sensors.py`** — readings / driver / suite + 5 per-sensor modules
- [ ] **7. Imports** — absolute package paths across the tree
- [ ] **8. `runlog.py`** — `cfg.meta()` instead of comment-scraping; repoint `CODE_FILES`
- [ ] **9. `param_sheet.py`** — generate from YAML; output to `docs/datasheets/`
- [ ] **10. Readmes** — blanks per dir, maneuvers table, root README rewrite
- [ ] **11. Verify** — value parity (76 constants), `verify.py` 78/78, full sim run
- [ ] **12. Push + PR**

## Baselines captured

- All 76 pre-move constant values → `scratchpad/baseline_car_data.json`
- `verify.py` **78 checks passed, 0 failed** on the pre-move tree
- `param_sheet.py`'s metadata table (61 rows) → `scratchpad/param_meta.json`

## Suggested commit boundaries

| Commit | Steps |
|---|---|
| A | 3 + 4 — the loader and the YAML data |
| B | 5 — params.py |
| C | 6 — the sensor split |
| D | 7 — imports |
| E | 8 + 9 — runlog + param_sheet |
| F | 10 — readmes and the root README |
