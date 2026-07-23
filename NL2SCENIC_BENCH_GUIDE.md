# Running the nl2scenic-bench dataset (arbitrary-named .scenic files)

This guide covers the tooling added to run SafeBench/ChatScene's Scenic
pipeline over `.scenic` files that are **not** named `scenario_NNN.scenic` -
specifically the `nl2scenic-bench` dataset:

```
safebench/scenario/scenic_data/nl2scenic-bench/results/scenic/
  image-only/*.scenic   (50 files)
  text-image/*.scenic   (50 files)
  text-only/*.scenic    (50 files)
  text-video/*.scenic   (50 files)
  video-only/*.scenic   (50 files)
```

Filenames are descriptive/source-derived (e.g. `CARLA_Leaderboard_10.scenic`,
`C-ICAP_Daytime - fallen shared bicycle and skewed vehicle.scenic`, some with
spaces) rather than the `scenario_NNN.scenic` pattern the rest of the pipeline
expects. This complements [`ROUTE_DRIVEN_SCENIC_GUIDE.md`](ROUTE_DRIVEN_SCENIC_GUIDE.md),
which still applies for bench directories that already use `scenario_NNN.scenic`
naming (`ChatScene`, `UN_R152/157/171`, `NHTSA_*`, `CARLA_Leaderboard`, etc.) -
those work completely unmodified, with zero manifest files involved.

---

## 1) The mechanism: a per-bench-directory scenario-id manifest

`safebench/util/scenario_id_manifest.py` implements `scenario_id_manifest.json`,
a small JSON file that lives **inside a bench directory**, mapping numeric
`scenario_id` -> `.scenic` filename:

```json
{
  "version": 1,
  "bench_id": "image-only",
  "id_to_file": {
    "1": "AnimalCrashWithoutPriorVehicleManeuver.scenic",
    "2": "C-ICAP_Construction area ahead.scenic",
    "...": "..."
  }
}
```

Assignment rule: a file literally named `scenario_NNN.scenic` keeps its
embedded id; every other file gets the next free integer, assigned in sorted
(alphabetical) order. This is deterministic and idempotent - rerunning the
builder over an unchanged directory reproduces the same manifest, and adding
a new file later only assigns it a new id without shifting any existing ones.

Every consumer (`generate_scenic_route_pickle.py`, `scenic_parse()` in
`safebench/scenario/tools/scenario_utils.py`, and `run_eval_batch.py`'s
scenario discovery) only falls back to this manifest **after** the legacy
`scenario_NNN.scenic` pattern-matching fails to find anything - so bench
directories that already use that naming convention are unaffected and never
touch the manifest.

---

## 2) Build the manifest (once per modality, before generation)

```bash
python scripts/build_scenario_id_manifest.py \
  --scenic-dir safebench/scenario/scenic_data/nl2scenic-bench/results/scenic/image-only
```

- Prints the `scenario_id -> filename` mapping and writes
  `scenario_id_manifest.json` into that directory.
- `--dry-run` previews the mapping without writing anything.
- Safe to rerun: existing assignments are preserved; only genuinely new
  `.scenic` files get new ids.

Repeat for each modality (`image-only`, `text-image`, `text-only`,
`text-video`, `video-only`). This step has already been run once for all 5
directories in this repo - rerun it only if `.scenic` files are added/removed.

---

## 3) Generate the route pickle (per modality)

Same script as the rest of the pipeline
([`generate_scenic_route_pickle.py`](scripts/generate_scenic_route_pickle.py)),
pointed directly at a modality directory so `scenic_route.pickle` and
`scenic_route_index.json` are written **inside** it, alongside the `.scenic`
files and the manifest:

```bash
python scripts/generate_scenic_route_pickle.py \
  --scenic-dir safebench/scenario/scenic_data/nl2scenic-bench/results/scenic/image-only \
  --glob '*.scenic' \
  --out-pickle safebench/scenario/scenic_data/nl2scenic-bench/results/scenic/image-only/scenic_route.pickle \
  --bench-id image-only \
  --key-mode numeric_only \
  --max-scene-attempts 200 \
  --warmup-ticks 20 \
  --max-record-seconds 30 \
  --record-video
```

- `--key-mode numeric_only` keeps the pickle key self-contained
  (`scenario_id_{id}_route_id_{route}`) so lookup never depends on the index
  file or a bench_id match - simplest for a pickle that's scoped to one
  modality directory anyway.
- If a file's `scenario_id` can't be inferred from its name, the script
  automatically loads (or builds, if missing) that directory's manifest, so
  running this directly - without step 2 - still works.
- Repeat with `--scenic-dir .../text-image`, `.../text-only`, `.../text-video`,
  `.../video-only` (and matching `--out-pickle`/`--bench-id`) for the other 4
  modalities.
- See [`ROUTE_DRIVEN_SCENIC_GUIDE.md` §2](ROUTE_DRIVEN_SCENIC_GUIDE.md#2-route-pickle-generation-generate_scenic_route_picklepy)
  for the full flag reference (`--record-video`, `--max-record-seconds`,
  `--no-resume`, etc.) - all of it applies unchanged here.

---

## 4) Scenario-config YAMLs (one per modality)

Five YAMLs already exist under `safebench/scenario/config/`:

- `eval_scenic_nl2scenic_image-only.yaml`
- `eval_scenic_nl2scenic_text-image.yaml`
- `eval_scenic_nl2scenic_text-only.yaml`
- `eval_scenic_nl2scenic_text-video.yaml`
- `eval_scenic_nl2scenic_video-only.yaml`

Each sets `route_dir` to the modality directory itself and `scenic_dir` to its
parent, so `scenic_dir/bench_id` resolves back to the same directory -
`scenic_route.pickle`, `scenic_route_index.json`, and
`scenario_id_manifest.json` all co-locate with the `.scenic` files. Example
(`eval_scenic_nl2scenic_image-only.yaml`):

```yaml
policy_type: 'scenic'
scenario_category: 'scenic'

route_dir: 'safebench/scenario/scenic_data/nl2scenic-bench/results/scenic/image-only'
scenic_dir: 'safebench/scenario/scenic_data/nl2scenic-bench/results/scenic'
bench_id: 'image-only'
scenic_mode2d: true
sample_num: 50
opt_step: 10
select_num: 2

method: 'scenic'
scenario_id: 1
route_id: [0]

ego_action_dim: 2
ego_state_dim: 4
ego_action_limit: 1.0
```

The other four are identical except `route_dir`'s trailing segment and
`bench_id`. `scenario_id`'s literal value in the YAML is irrelevant for batch
runs - `run_eval_batch.py`/`run_eval_nl2scenic_bench.py` override it per run.

---

## 5) Run one modality with `run_eval_batch.py`

Standard [`run_eval_batch.py`](scripts/run_eval_batch.py) workflow (see
[`ROUTE_DRIVEN_SCENIC_GUIDE.md` §4-batch](ROUTE_DRIVEN_SCENIC_GUIDE.md#4-batch-batch-train_scenario--eval-run_eval_batchpy)
for full flag reference) - it now discovers scenario ids via the manifest
whenever no `scenario_NNN.scenic` files are found:

```bash
# Preview discovery without running CARLA
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic_nl2scenic_image-only.yaml \
  --mode train_scenario \
  --dry_run

# Step 1: train_scenario (OPT selection), writes scenario_<id>.json per scenario
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic_nl2scenic_image-only.yaml \
  --mode train_scenario \
  --test_policy ppo \
  --route_id 0 \
  --port 2005 --tm_port 8005 --device cpu

# Step 2: eval (reads each scenario_<id>.json), optionally averaging metrics
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic_nl2scenic_image-only.yaml \
  --mode eval \
  --test_policy ppo \
  --route_id 0 \
  --port 2005 --tm_port 8005 --device cpu \
  --average
```

Swap `--scenario_cfg` for the other 4 YAMLs to run each modality individually.
`--scenario_ids`/`--scenario_range` work as usual to run a subset.

---

## 6) Run all 5 modalities in one command with `run_eval_nl2scenic_bench.py`

[`scripts/run_eval_nl2scenic_bench.py`](scripts/run_eval_nl2scenic_bench.py)
loops `train_scenario` then `eval` across all 5 modality YAMLs, invoking
`run_eval_batch.py` once per (modality, mode):

```bash
# Preview all 10 (5 modalities x 2 modes) subprocess commands, no CARLA needed
python scripts/run_eval_nl2scenic_bench.py --dry_run

# Real run: train_scenario + eval, all 5 modalities, averaging each modality's results
python scripts/run_eval_nl2scenic_bench.py \
  --test_policy ppo \
  --route_id 0 \
  --port 2005 --tm_port 8005 --device cpu \
  --average
```

Useful flags:

| Flag | Purpose |
|------|---------|
| `--modalities` | Subset to run, e.g. `--modalities image-only text-only` (default: all 5) |
| `--modes` | Subset/order of modes, e.g. `--modes eval` to skip train_scenario (default: `train_scenario eval`) |
| `--agent_cfg` | Forwarded to `run_eval_batch.py` (default `adv_scenic.yaml`) |
| `--test_policy` | Forwarded (default `ppo` - use a matching `pretrain_dir` checkpoint) |
| `--test_epoch` | Forwarded, for per-scenario adversarial checkpoints |
| `--scenario_ids` / `--scenario_range` | Forwarded, to run a subset within each modality |
| `--average` | Applied only on the `eval` mode calls |
| `--continue_on_error` / `--no-continue_on_error` | Default `True` - keep going if one (modality, mode) fails |
| `--dry_run` | Print all subprocess commands without running CARLA |

Any additional unrecognized flags are forwarded straight through to
`run_eval_batch.py` (and from there to `run_eval.py`).

---

## 7) Where outputs go

Same bench-id-insertion behavior as the rest of the pipeline
([`ROUTE_DRIVEN_SCENIC_GUIDE.md` §4.4](ROUTE_DRIVEN_SCENIC_GUIDE.md#44-where-outputs-go)) -
since each YAML sets `bench_id` to the modality name, results land under:

```text
log/adv_train/<mode>/<policy>/<agent_cfg>_epoch<N>/eval_scenic_nl2scenic_image-only/image-only/
  scenario_1/.../eval_results/OPT_<behavior-name>_ROUTE-0_results.pkl
  scenario_2/.../eval_results/OPT_<behavior-name>_ROUTE-0_results.pkl
  ...
  average_eval_results.json          # if --average was passed
```

`<behavior-name>` is the original `.scenic` file's stem (e.g.
`CARLA_Leaderboard_10`, or a name with spaces) - `scripts/average_eval_results.py`'s
matching pattern was generalized from `OPT_scenario_(\d+)_ROUTE-0_results.pkl`
to `OPT_.+_ROUTE-\d+_results.pkl` to pick these up (it still matches legacy
`OPT_scenario_001_ROUTE-0_results.pkl` filenames too).

---

## 8) Backward compatibility

Nothing here changes behavior for existing `scenario_NNN.scenic` bench
directories:

- `generate_scenic_route_pickle.py`, `scenic_parse()`, and
  `run_eval_batch.py`'s discovery all try the legacy pattern-based resolution
  **first**; the manifest is only consulted when that fails to find a match.
- Directories with no `scenario_id_manifest.json` never touch the new code
  path at all.
- Verified against the existing `UN_R157` bench
  (`safebench/scenario/scenario_data/scenic_data_wenting/scenic_route-driven_C2S_sac/UN_R157`,
  pure `scenario_NNN.scenic` naming, no manifest) - `run_eval_batch.py --dry_run`
  still discovers its 12 scenarios via the original regex path, unchanged.

---

## 9) Troubleshooting

- **`No scenario_*.scenic files found in: ... (and no scenario_id_manifest.json to fall back to)`**
  - Run step 2 (`build_scenario_id_manifest.py`) for that modality directory first.
- **`Could not infer --scenario-id from filename`** (from `generate_scenic_route_pickle.py`)
  - Either pass `--scenario-id` explicitly (single-file mode only), or build the
    manifest first so the directory-batch mode can resolve it.
- **A `.scenic` file was added/removed after generating the route pickle**
  - Rerun `build_scenario_id_manifest.py` for that directory - existing ids are
    preserved, only new files get new ids. Then rerun
    `generate_scenic_route_pickle.py` (it resumes by default; use
    `--no-resume` to regenerate everything).

---

## 10) Configuring a new batch of scenic codes (a different dataset under `scenic_data/`)

Everything above is written against `nl2scenic-bench` specifically, but none of
the tooling is actually tied to that name. This section is the checklist for
pointing the same pipeline at a **new** dataset directory, e.g.:

```
safebench/scenario/scenic_data/<new-dataset>/<bench-name>/*.scenic
```

(`<bench-name>` is whatever you want the `bench_id` to be - a modality name
like `image-only`, a source name, whatever groups these files together for
`bench_id`-scoped output paths.)

### 10.1 Check the naming convention

```bash
ls safebench/scenario/scenic_data/<new-dataset>/<bench-name>/*.scenic
```

If files are already named `scenario_001.scenic`, `scenario_002.scenic`, etc.,
skip straight to §10.4 - the legacy pattern-matching handles everything and no
manifest is needed. Otherwise (arbitrary/descriptive filenames), continue below.

### 10.2 Build the scenario-id manifest

```bash
python scripts/build_scenario_id_manifest.py \
  --scenic-dir safebench/scenario/scenic_data/<new-dataset>/<bench-name>
```

Same mechanism as §1-2 above. Do this once per bench directory in the new
dataset before generating routes.

### 10.3 Check for known Scenic-authoring patterns that block route generation

Before running `generate_scenic_route_pickle.py` on a whole new batch, grep for
these - each was a real, widespread blocker in `nl2scenic-bench` and is worth
ruling out up front rather than discovering it 30 failures in:

- **`param carla_map` set via a variable instead of a literal**, e.g.
  `Town = 'Town05'` then `param carla_map = Town`. The generator's
  `CARLA_MAP_RE` only understands a literal string
  (`param carla_map = 'Town05'`), so this fails every file with
  `Missing 'param carla_map' in Scenic file`. Check with:
  ```bash
  grep -L "^param carla_map = '" safebench/scenario/scenic_data/<new-dataset>/<bench-name>/*.scenic
  ```
  If files show up, run [`scripts/fix_nl2scenic_carla_map.py`](scripts/fix_nl2scenic_carla_map.py)
  (despite the name, it's dataset-agnostic - it takes any root directory) to
  rewrite the indirection to a literal, in place:
  ```bash
  python scripts/fix_nl2scenic_carla_map.py --dry-run safebench/scenario/scenic_data/<new-dataset>
  python scripts/fix_nl2scenic_carla_map.py safebench/scenario/scenic_data/<new-dataset>
  ```
  It only ever rewrites the single `param carla_map = ...` line; everything
  else in each file is untouched. `weather` does **not** need this treatment -
  `generate_scenic_route_pickle.py` already treats a non-literal
  `param weather = ...` (e.g. `Weather(...)`, `Uniform(...)`) as optional and
  simply lets the file's own definition run unmodified (see §2.1 of
  `ROUTE_DRIVEN_SCENIC_GUIDE.md`).
- **`VerifaiRange(...)` instead of `Range(...)`** for `param` declarations.
  Fails with `ModuleNotFoundError: No module named 'verifai'` if the package
  isn't installed in the active venv. Check with:
  ```bash
  grep -l "VerifaiRange" safebench/scenario/scenic_data/<new-dataset>/<bench-name>/*.scenic
  ```
  If any show up, `pip install verifai` into the venv (confirmed safe in this
  repo's `chatscene-v3` venv - it only needed a compatible `attrs` downgrade,
  no conflicts with `scenic`/`carla`/`torch`).

### 10.4 Generate route pickles - one file per process, never `--scenic-dir` on a whole directory

Use [`scripts/generate_route_batch.sh`](scripts/generate_route_batch.sh), **not**
`generate_scenic_route_pickle.py --scenic-dir` directly. Scenic keeps
process-global state (`scenic.syntax.veneer`'s running-scenario registry, the
road-network cache) that isn't fully torn down between files run in the same
process - in practice this means only the *first* file in a `--scenic-dir`
batch ever succeeds, and everything after it fails with a generic,
content-independent `AssertionError`. `generate_route_batch.sh` runs
`--scenic-file` in a loop instead, giving each file a fresh interpreter and
fresh CARLA connection:

```bash
# Whole bench directory (safe to re-run repeatedly - --resume skips finished ones)
scripts/generate_route_batch.sh safebench/scenario/scenic_data/<new-dataset>/<bench-name>

# Or a curated list of specific files
scripts/generate_route_batch.sh \
  safebench/scenario/scenic_data/<new-dataset>/<bench-name> \
  some_scenario.scenic another_scenario.scenic
```

It auto-sources `env.scenic3.sh`, pre-checks the CARLA port is actually
listening, logs every file's full output persistently (default
`/tmp/<bench-name>_batch_loop_<timestamp>.log` - overridable with `--log`), and
prints a final summary of successes/failures. See `--help` for all flags
(`--port`, `--tm-port`, `--max-scene-attempts`, `--no-video`, etc.).

### 10.5 Scenario-config YAML for the new bench

Create `safebench/scenario/config/eval_scenic_<new-dataset>_<bench-name>.yaml`,
same shape as the `nl2scenic-bench` ones in §4:

```yaml
policy_type: 'scenic'
scenario_category: 'scenic'

route_dir: 'safebench/scenario/scenic_data/<new-dataset>/<bench-name>'
scenic_dir: 'safebench/scenario/scenic_data/<new-dataset>'
bench_id: '<bench-name>'
scenic_mode2d: true
sample_num: 50
opt_step: 10
select_num: 2

method: 'scenic'
scenario_id: 1
route_id: [0]

ego_action_dim: 2
ego_state_dim: 4
ego_action_limit: 1.0
```

`route_dir` points at the bench directory itself (where the pickle/index/
manifest live); `scenic_dir` points at its **parent** (`<new-dataset>`), so
`scenic_dir/bench_id` resolves back to the same bench directory. One YAML per
bench directory in the new dataset.

### 10.6 Route-driven conversion before `train_scenario`/`eval`

`train_scenario`/`eval` need route-driven `.scenic` files (ego pinned to
`globalParameters.spawnPt`/`yaw`, no `EgoBehavior`, adversary spawn inverted
relative to the fixed ego - see `ROUTE_DRIVEN_CONVERSION_PLAYBOOK.md`). Mirror
the pattern used for `nl2scenic-bench`:

1. Create a sibling directory: `safebench/scenario/scenic_data/<new-dataset>_route-driven/<bench-name>/`.
2. Copy in **only** the files that actually have a route (i.e. whose
   `scenario_id` is a key in that bench's `scenic_route.pickle`) - the
   intersection of `scenario_id_manifest.json`'s `id_to_file` and the
   pickle's keys.
3. Copy `scenario_id_manifest.json` alongside, **filtered** to just those
   copied ids (same ids, not renumbered - the pickle is keyed by the original
   `scenario_id`).
4. Apply the Playbook conversion to each copied file by hand.
5. Add a second YAML, `eval_scenic_<new-dataset>_<bench-name>_route_driven.yaml`,
   identical to §10.5's except `scenic_dir` points at
   `safebench/scenario/scenic_data/<new-dataset>_route-driven` instead (keep
   `route_dir` pointing at the original bench directory - the pickle doesn't
   need to move, since lookups are keyed purely by `scenario_id`, never by
   the `.scenic` file's path).

### 10.7 Run it

```bash
source env.scenic3.sh

python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic_<new-dataset>_<bench-name>_route_driven.yaml \
  --mode train_scenario \
  --test_policy sac \
  --route_id 0 --port 2005 --tm_port 8005 --device cpu

python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic_<new-dataset>_<bench-name>_route_driven.yaml \
  --mode eval \
  --test_policy sac \
  --route_id 0 --port 2005 --tm_port 8005 --device cpu \
  --average
```

Use `--scenario_ids`/`--scenario_range` to target specific scenarios (see §5).
`scripts/run_eval_nl2scenic_bench.py` is hardcoded to the 5 `nl2scenic-bench`
modality names and config naming, so it won't pick up a new dataset as-is -
either invoke `run_eval_batch.py` per bench directory as above, or copy that
wrapper and adjust its modality list/config-name lambda for the new dataset.

### 10.8 Gotchas already fixed at the framework level (apply automatically)

These live in shared `safebench/util/scenic_utils.py`, not in any per-dataset
script, so they help any new dataset without further action:

- **`SafebenchCarlaSimulation.__init__`** now cleans up Scenic's global
  simulation state (`veneer.currentSimulation`) on *any* exception during
  setup, not just Scenic's own rejection-exception types - previously, one
  scenario hitting an unrelated bug (e.g. a color-tuple `AttributeError`) on
  its first sample poisoned every subsequent resample attempt with a bogus
  `assert currentSimulation is None`, burning the rest of the retry budget on
  a masked, misleading error.
- **`ScenicSimulator.endSimulation()`** now tolerates a `RejectSimulationException`/
  `RejectionException`/`GuardViolation` raised while stopping a scenario
  that's already ending (e.g. a dynamic `require` briefly violated right as
  the scenario naturally terminates) instead of letting it abort the rest of
  `train_scenario`/`eval`'s sample loop.
- **`--device cpu` requires actually passing it** - if the machine's GPU has a
  CUDA compute capability the installed PyTorch build doesn't support (check
  for an `sm_XXX ... not compatible` warning at startup), computations
  silently run on CUDA anyway if `--device` is omitted or mistyped, producing
  invalid tensor values rather than a clear error.
