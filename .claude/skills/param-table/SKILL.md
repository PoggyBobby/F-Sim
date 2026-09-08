---
name: param-table
description: Keep docs/guide/parameters.md in sync with the params.yaml files. Use whenever a params.yaml is added, edited or deleted, whenever git diff/git status shows one changed, before committing parameter changes, or when asked to update/check the parameter table or docs/guide/parameters.md.
---

# Parameter table sync

`docs/guide/parameters.md` carries a generated table (parameter_name, file_name,
parameter_type) built from every `params.yaml` by `param_table.py`. The table is
data, not prose: it lives between two HTML markers and is rewritten wholesale.
Everything outside the markers is hand-written and must never be touched by this
skill.

## 1. Did the parameter data move?

```bash
git status --porcelain -- '*params.yaml'                 # uncommitted edits
git diff --name-only -- '*params.yaml'                   # unstaged
git diff --name-only --cached -- '*params.yaml'          # staged
git diff --name-only HEAD~1 -- '*params.yaml'            # last commit
```

Any hit means the table may be stale. Adding or deleting a `params.yaml`
counts — the loader discovers files by walking `model/`, `controllers/`, `sil/`.

## 2. Check

```bash
.venv/bin/python param_table.py --check
```

Exit 0 = in sync. Exit 1 = stale, and it prints the row count in the doc vs the
parameter count in the YAML files. A `status:` edit alone makes it stale too —
`parameter_type` is that field.

## 3. Regenerate

```bash
.venv/bin/python param_table.py
git diff -- docs/guide/parameters.md
```

Read the diff before moving on. Expected shapes:

- new/removed rows → a parameter was added or deleted
- a `parameter_type` change → someone edited a `status:` tag (the point of the
  provenance system; confirm the new tag is one of the fixed vocabulary in
  `model/config.py`'s `STATUS_TAGS`)
- a `file_name` change → a parameter moved between components
- prose outside the markers changed → **something is wrong, revert it**

If the generator raises `ConfigError`, the YAML is malformed (missing
`namespace:`/`unit:`/`what:`, unknown unit, duplicate namespace, unresolvable
`derived:` path). Fix the YAML — never work around it by hand-editing the table.

## 4. Then

The same edit invalidates two other generated artifacts:

```bash
.venv/bin/python param_sheet.py     # docs/datasheets/FSAE-Sim Parameters.xlsx
.venv/bin/python verify.py          # 80 checks; must pass before any run
```

And if vehicle or tire numbers moved meaningfully, the controller gains in
`controllers/python/params.yaml` need retuning.

## Rules

- Never hand-edit the rows between the markers.
- Never change the prose outside the markers as part of a sync.
- Never commit a parameter change with a stale table — run `--check` first.
