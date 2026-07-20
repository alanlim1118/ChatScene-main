# Route-driven Scenic workflow (SafeBench / ChatScene)

This guide summarizes how to:

- Generate a route pickle with `scripts/generate_scenic_route_pickle.py` (Scenic **3.x**)
- Generate a route pickle for Scenic **2.x** files with `scripts/generate_scenic_route_pickle_v2.py` — see [Section 2-v2](#2-v2-scenic-2x-route-pickle-generation-generate_scenic_route_pickle_v2py)
- Migrate Scenic `Range(...)` params to `OPT_*` naming with `scripts/migrate_opt_range_params.py`
- Run SafeBench Scenic with `scripts/run_eval.py` (train_scenario → eval), or batch over a whole bench with `scripts/run_eval_batch.py`
- Run SafeBench Scenic **2.x** with `scripts/run_eval_v2.py` + `eval_scenic_v2.yaml` — see [Section 4-v2](#4-v2-running-safebench--scenic-2x-run_eval_v2py)
- Run JSON-free Scenic eval with `scripts/run_eval_scenic.py` or batch over a whole bench with `scripts/run_eval_scenic_batch.py`
- Average evaluation metrics across runs with `scripts/average_eval_results.py`
- Organize directories / configs for **bench layout** Scenic files (e.g. `ChatScene/scenario_001.scenic`)

---

## 1) Directory layout (bench layout)

### 1.1 Route-driven Scenic files

Place route-driven Scenic files under a bench folder:

```text
safebench/scenario/scenario_data/scenic_data_wenting/
  scenic_route-driven/
    ChatScene/
      scenario_001.scenic
      scenario_002.scenic
      ...
```

### 1.2 Route pickle + index

Place the route pickle and its index together:

```text
safebench/scenario/scenario_data/route_wenting/
  scenic_route.pickle
  scenic_route_index.json
```

`scenic_route_index.json` maps:

```text
{bench_id}:{scenario_id}:{route_id}  ->  pickle_key
```

Example:

```json
{
  "ChatScene:1:0": {
    "pickle_key": "scenario_id_ChatScene__1_route_id_0",
    "scenic_file": "…/ChatScene/scenario_001.scenic",
    "town": "Town05",
    "weather": "ClearNoon"
  }
}
```

---

## 2) Route pickle generation (`generate_scenic_route_pickle.py`)

> **Scenic 2.x files:** use [Section 2-v2](#2-v2-scenic-2x-route-pickle-generation-generate_scenic_route_pickle_v2py) instead.

### 2.1 What it generates

For each Scenic file, it writes one entry into the pickle containing:

- `spawnPt`: `{x, y, z, yaw}` (Scenic coords)
- `trajectory`: **3 points** (start/mid/end) in Scenic coords (used by SafeBench to build the agent route)
- `waypoints`: polyline points in Scenic coords (passed into Scenic globals; optional for your templates)
- `lanePts`: currently written as `[]` (empty)

It also writes/updates `scenic_route_index.json` next to the pickle.

### 2.2 Single Scenic file

```bash
python scripts/generate_scenic_route_pickle.py \
  --scenic-file safebench/scenario/scenario_data/scenic_data_wenting/Self_Gemini_3flash_R1_original/UN_R152/scenario_003.scenic \
  --out-pickle safebench/scenario/scenario_data/route_wenting/scenic_route.pickle \
  --max-scene-attempts 200   --warmup-ticks 20   --record-video
```

### 2.3 Batch generation (directory)

```bash
python scripts/generate_scenic_route_pickle.py \
  --scenic-dir safebench/scenario/scenario_data/scenic_data_wenting/Self_Gemini_3flash_R1_original \
  --out-pickle safebench/scenario/scenario_data/route_wenting/scenic_route.pickle
```

Bench Generation

python scripts/generate_scenic_route_pickle.py   --scenic-dir safebench/scenario/scenario_data/scenic_data_chatscene/chatscene_R1/UN_R152   --glob '*.scenic'   --out-pickle safebench/scenario/scenario_data/route_chatscene/scenic_route.pickle   --bench-id UN_R152   --max-scene-attempts 200   --warmup-ticks 20   --record-video


### 2.4 Overwrite / rewrite existing entries

By default the script skips keys already present in the output pickle. To **rewrite** them, pass:

```bash
--no-resume
```

### 2.5 Handle CARLA/Scenic spawn failures (important)

CARLA may fail to spawn pedestrians/actors for some sampled scenes. The generator retries scene sampling:

- `--max-scene-attempts` (default 50)
- `--warmup-ticks` (default 0)

Example:

```bash
python scripts/generate_scenic_route_pickle.py \
  --scenic-file .../scenario_001.scenic \
  --out-pickle safebench/scenario/scenario_data/route_wenting/scenic_route.pickle \
  --max-scene-attempts 200 \
  --warmup-ticks 20
```

### 2.6 Record MP4 during route generation (FPV + BEV)

Enable with:

```bash
--record-video
```

Videos are saved to:

```text
safebench/scenario/scenario_data/recordings/
```

The recorder requires `ffmpeg` available on `PATH`.


### 2.6 Set time limit to recording (for scenarios that does not terminate)

Enable with:

```bash
--fixed-delta-seconds 0.1 --max-record-seconds 20
```

---

## 2-v2) Scenic 2.x route pickle generation (`generate_scenic_route_pickle_v2.py`)

Use this variant when your `.scenic` files target **Scenic 2.x** (e.g. ChatScene `chatscene_overlap_nl2scenic_route-driven`, no Scenic 3 `new` syntax). It mirrors `generate_scenic_route_pickle.py` but runs inside `scenic2-venv` via [safebench/util/scenic_utils_v2.py](safebench/util/scenic_utils_v2.py).

### 2-v2.1 When to use v1 vs v2

| | **v1** (`generate_scenic_route_pickle.py`) | **v2** (`generate_scenic_route_pickle_v2.py`) |
|---|---|---|
| Scenic version | Scenic **3.x** ([scenic_utils.py](safebench/util/scenic_utils.py)) | Scenic **2.x** ([scenic_utils_v2.py](safebench/util/scenic_utils_v2.py)) |
| Typical venv | default / Scenic 3 env | `scenic2-venv` (see [env.scenic2.sh](env.scenic2.sh)) |
| Scenic files | Scenic 3 syntax (`new Car`, etc.) | Scenic 2 syntax (e.g. overlap nl2scenic route-driven set) |
| Eval entry point | [run_eval.py](scripts/run_eval.py) + [eval_scenic.yaml](safebench/scenario/config/eval_scenic.yaml) | [run_eval_v2.py](scripts/run_eval_v2.py) + [eval_scenic_v2.yaml](safebench/scenario/config/eval_scenic_v2.yaml) |
| Runner | [ScenicRunner](safebench/scenic_runner.py) | [ScenicRunnerV2](safebench/scenic_runner_v2.py) |

CLI flags (`--scenic-file`, `--scenic-dir`, `--out-pickle`, `--bench-id`, `--no-resume`, video recording, etc.) are the same as v1.

### 2-v2.2 Pickle contents: coordinate formats (important)

Both v1 and v2 generators now write the **same pickle schema**. Ego motion is captured via `_carla_to_scenic_xyz()` → `(x, -y, z)` during recording, and all route geometry is stored in **Scenic coordinates**:

- `spawnPt`: `{x, y, z}` in Scenic coords; `yaw` in **CARLA degrees**
- `trajectory`: 3 coarse points (start / mid / end) in **Scenic coords**
- `waypoints`: dense polyline in **Scenic coords**
- `lanePts`: `[]` (empty)
- No `route_format` field

At eval time, [`ScenicDataLoader`](safebench/scenario/scenario_data_loader.py) converts Scenic → CARLA with `carla.Location(x, -y, z)` for both pipelines.

**Legacy v2 pickles** (generated before this unification) may still contain:

```json
"route_format": {
  "spawn_position": "scenic",
  "spawn_yaw": "scenic_radians",
  "trajectory": "carla",
  "waypoints": "scenic"
}
```

The loader still honors `route_format.trajectory == "carla"` for those entries. To upgrade without re-running CARLA:

```bash
python scripts/migrate_v2_route_pickle_to_scenic.py safebench/scenario/scenario_data/route_debug/scenic_route.pickle --dry-run
python scripts/migrate_v2_route_pickle_to_scenic.py safebench/scenario/scenario_data/route_debug/scenic_route.pickle
```

### 2-v2.3 Loader behavior (`ScenicDataLoader`)

At eval time, [safebench/scenario/scenario_data_loader.py](safebench/scenario/scenario_data_loader.py) converts pickle points to `carla.Location`:

| `route_format.trajectory` | Conversion |
|---|---|
| absent or `"scenic"` | `carla.Location(x, -y, z)` — Scenic → CARLA (default for v1 and current v2) |
| `"carla"` | `carla.Location(x, y, z)` — legacy v2 only; migrate or regenerate |

**Backward compatibility:** pickles without `route_format` default to `"scenic"`. Legacy v2 pickles with `"trajectory": "carla"` continue to work until migrated.

**Common bug:** Y-flipping a trajectory that is already in CARLA coords (or failing to flip Scenic coords) places the SAC route far from the ego spawn. Symptom: ego stationary, `route_completion: 0.0`, scenarios stop on timeout. Fix: ensure stored `trajectory` is in Scenic coords (regenerate with current v2 generator, or run `migrate_v2_route_pickle_to_scenic.py`).

### 2-v2.4 Example commands

Activate Scenic 2 env first:

```bash
source env.scenic2.sh   # or: source scenic_v2-venv/bin/activate + CARLA PYTHONPATH
```

Single file:

```bash
python scripts/generate_scenic_route_pickle_v2.py \
  --scenic-file safebench/scenario/scenario_data/scenic_data_chatscene/chatscene_overlap_nl2scenic_route-driven/NHTSA_PreCrash/scenario_022.scenic \
  --out-pickle safebench/scenario/scenario_data/route_v2/scenic_route.pickle \
  --bench-id NHTSA_PreCrash \
  --max-scene-attempts 200 \
  --warmup-ticks 20 \
  --record-video
```

Batch (bench directory):

```bash
python scripts/generate_scenic_route_pickle_v2.py \
  --scenic-dir safebench/scenario/scenario_data/scenic_data_chatscene/chatscene_overlap_nl2scenic_route-driven/NHTSA_PreCrash \
  --glob '*.scenic' \
  --out-pickle safebench/scenario/scenario_data/route_debug/scenic_route.pickle \
  --bench-id NHTSA_PreCrash \
  --max-scene-attempts 200 \
  --warmup-ticks 20
```

**Do not** assume legacy v2 pickles (with `route_format.trajectory == "carla"`) match the current unified schema — migrate or regenerate if eval shows a route/spawn mismatch.

### 2-v2.5 Pipeline comparison

```text
Scenic 3 pipeline (Sections 2 + 4):
  generate_scenic_route_pickle.py  →  trajectory in Scenic coords
       ↓
  run_eval.py + ScenicRunner + eval_scenic.yaml
       ↓
  ScenicDataLoader: Y-flip trajectory  →  CARLA route for SAC

Scenic 2.x pipeline (Sections 2-v2 + 4-v2):
  generate_scenic_route_pickle_v2.py  →  trajectory in Scenic coords (same schema as v1)
       ↓
  run_eval_v2.py + ScenicRunnerV2 + eval_scenic_v2.yaml
       ↓
  ScenicDataLoader: Y-flip trajectory  →  CARLA route for SAC
```

---

## 3) Making Scenic code route-driven (agent controls ego)

### 3.1 Ego is controlled by SafeBench agent

SafeBench Scenic integration skips stepping the ego’s Scenic behavior. So for route-driven evaluation:

- Spawn ego from route pickle globals:
  - `globalParameters.spawnPt`, `globalParameters.yaw`
- **Do not** define an ego Scenic behavior (or keep it but it won’t be stepped).

### 3.2 Typical route-driven snippet

```scenic
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL
```

If you need to place actors “ahead of ego”, use a valid VectorField such as `roadDirection`:

```scenic
anchor = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DISTANCE_AHEAD
```

Avoid `following egoSpawnPt.heading` (heading is a scalar and causes a Scenic type error).

### 3.3 OPT_ Range parameters (`migrate_opt_range_params.py`)

SafeBench’s OPT optimizer ([`safebench/util/scenic_utils.py`](safebench/util/scenic_utils.py) `get_params()`) only collects parameters whose name starts with `OPT` **and** whose value is a Scenic `Range(...)`. Fixed scalars (`EGO_MODEL`, `ADV_BRAKE = 1.0`, `TERMINATION_DIST = 120`, etc.) and reserved names (`map`, `carla_map`, `weather`) are left unchanged.

Use `scripts/migrate_opt_range_params.py` when you have legacy `.scenic` files with unprefixed `Range` params (e.g. `param distAhead = Range(20, 40)`) and need them renamed for `train_scenario` / `eval`.

**What it changes**

| Before | After |
|--------|-------|
| `param distAhead = Range(20, 40)` | `param OPT_DIST_AHEAD = Range(20, 40)` |
| `globalParameters.distAhead` | `globalParameters.OPT_DIST_AHEAD` |
| `param EGO_MODEL = 'vehicle...'` | unchanged (not a `Range`) |
| `param map = localPath(...)` | unchanged (excluded) |

CamelCase names use explicit mappings (`pedDist` → `OPT_PED_DIST`, `distAhead` → `OPT_DIST_AHEAD`, etc.). The script also fixes stale `globalParameters` refs when an `OPT_*` param exists but the reference was not updated.

**Commands**

Preview changes (no writes):

```bash
python scripts/migrate_opt_range_params.py --dry-run
```

Apply to the default tree (`scenic_route-driven_Chat2Scenic`):

```bash
python scripts/migrate_opt_range_params.py
```

Migrate a different directory:

```bash
python scripts/migrate_opt_range_params.py \
  safebench/scenario/scenario_data/scenic_data_wenting/scenic_route-driven_Chat2Scenic/UN_R152
```

After migration, every `Range` param in actor placement and behavior should be referenced via `globalParameters.OPT_*` (see [Section 3.2](#32-typical-route-driven-snippet)).

---

## 4) Running SafeBench (`run_eval.py`)

> **Scenic 2.x pipeline:** use [Section 4-v2](#4-v2-running-safebench--scenic-2x-run_eval_v2py) with `scenic2-venv` instead.

### 4.1 Scenario YAML (example: `eval_scenic_wenting.yaml`)

For bench layout + route_wenting:

```yaml
route_dir: 'safebench/scenario/scenario_data/route_wenting'
scenic_dir: 'safebench/scenario/scenario_data/scenic_data_wenting/scenic_route-driven'
bench_id: 'ChatScene'

scenario_id: 1
route_id: [0]
```

### 4.2 Important: file selection when `scenario_id` is specified

With `bench_id: ChatScene` and `--scenario_id N`, SafeBench will only load:

```text
scenic_dir/ChatScene/scenario_{N:03d}.scenic
```

So for scenario 2 it will load `scenario_002.scenic` only.

### 4.3 Two-step workflow (recommended)

SafeBench Scenic `eval` mode expects an OPT-selection JSON produced by `train_scenario`.

1) **train_scenario** (generates `scenario_1.json` under the bench dir)

```bash
python scripts/run_eval.py \
  --mode train_scenario \
  --scenario_cfg eval_scenic_wenting.yaml \
  --scenario_id 1 \
  --route_id 0 \
  --port 2002 \
  --tm_port 8002 \
  --device cpu
```

This writes (for bench layout):

```text
safebench/scenario/scenario_data/scenic_data_wenting/scenic_route-driven/ChatScene/scenario_1.json
```

2) **eval** (runs only selected hard scenes; supports video)

```bash
python scripts/run_eval.py \
  --mode eval \
  --scenario_cfg eval_scenic_wenting.yaml \
  --scenario_id 1 \
  --route_id 0 \
  --port 2002 \
  --tm_port 8002 \
  --device cpu \
  --save_video
```

### 4.4 Where outputs go

The run prints a log directory like:

```text
log/adv_train/<mode>/<policy>/<agent_cfg>_epoch<N>/<scenario_cfg>/<bench_id>/scenario_<id>/<exp>_seed_0/
```

The `<bench_id>` level (e.g. `ChatScene`, `UN_R152`) is inserted automatically when the scenario YAML defines `bench_id`. It sits between the `<scenario_cfg>` name and `scenario_<id>`, for every mode (`train_scenario`, `eval`, ...). Example:

```text
log/adv_train/eval/sac/adv_scenic_epochNone/eval_scenic_wenting/UN_R152/scenario_1/...
```

Inside it:

- `eval_results/*_results.pkl` and `eval_results/*_records.pkl`
- `video/<log_name>/*.mp4` (only if `--save_video`)

If `bench_id` is not set in the YAML, the level is omitted (legacy layout). Implemented in [scripts/run_eval.py](scripts/run_eval.py) (the `args.output_dir` construction).

---

## 4-batch) Batch train_scenario / eval (`run_eval_batch.py`)

Use this when you want to run [Section 4.3](#43-two-step-workflow-recommended) over **many scenario IDs** in one command. The batch script auto-discovers all `scenario_*.scenic` files under `scenic_dir/<bench_id>` (from the scenario YAML), then spawns one `run_eval.py` subprocess per scenario.

Outputs (`scenario_<id>.json`, `eval_results/*.pkl`, videos) are **identical** to running `run_eval.py` manually for each `--scenario_id`.

### 4-batch.1 When to use it

- Run `train_scenario` across an entire bench to produce OPT-selection JSON for every scenario.
- Run `eval` across an entire bench after training (reads each `scenario_<id>.json`).
- Re-run a bench after partial completion (completed routes are skipped by `ScenicRunner` via `logger.check_eval_dir`).
- Optionally average metrics across the bench in one step with `--average`.

### 4-batch.2 Commands

**train_scenario** (optimize `OPT_*` ranges; writes `scenario_<id>.json` per scenario):

```bash
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic.yaml \
  --mode train_scenario \
  --test_policy ppo \
  --route_id 0 \
  --port 2002 \
  --tm_port 8002 \
  --device cpu
```

**eval** (evaluate using saved JSON; add `--save_video` if needed):

```bash
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic.yaml \
  --mode eval \
  --test_policy ppo \
  --route_id 0 \
  --port 2002 \
  --tm_port 8002 \
  --device cpu \
  --average
```

Evaluate a subset only:

```bash
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic.yaml \
  --mode eval \
  --test_policy ppo \
  --scenario_range 1-9 \
  --route_id 0
```

Explicit scenario IDs (combinable with `--scenario_range`):

```bash
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic.yaml \
  --mode train_scenario \
  --test_policy ppo \
  --scenario_ids 1 3 9 \
  --route_id 0
```

Preview commands without running CARLA:

```bash
python scripts/run_eval_batch.py \
  --scenario_cfg eval_scenic.yaml \
  --mode train_scenario \
  --dry_run
```

### 4-batch.3 Flags

**Batch-specific** (consumed by `run_eval_batch.py`):

| Flag | Purpose |
|------|---------|
| `--scenario_ids` | Optional explicit list of scenario IDs |
| `--scenario_range` | Inclusive range, e.g. `1-9` |
| `--continue_on_error` | Default `True`; keep going if one subprocess fails |
| `--no-continue_on_error` | Stop the batch on the first failure |
| `--dry_run` | Print discovered IDs and subprocess commands only |
| `--average` | After all runs, call `average_eval_results.py` on the bench output dir |
| `--average_output` | Override path for `average_eval_results.json` |

**Forwarded to `run_eval.py`** (pass them on the same command line; unrecognized flags are forwarded verbatim):

`--agent_cfg`, `--scenario_cfg`, `--mode`, `--route_id`, `--port`, `--tm_port`, `--device`, `--seed`, `--save_video`, `--test_policy`, `--test_epoch`, `--threads`, `--max_episode_step`, `--frame_skip`, `--fixed_delta_seconds`, `--auto_ego`

`--scenario_cfg` is **required** (used for scenario discovery and passed through to each subprocess).

### 4-batch.4 How it works

1. Loads `safebench/scenario/config/<scenario_cfg>` to get `scenic_dir` and `bench_id`.
2. Scans `scenic_dir/<bench_id>/scenario_*.scenic` and extracts numeric IDs (e.g. `scenario_011.scenic` → `11`).
3. If neither `--scenario_ids` nor `--scenario_range` is given, runs **all** discovered IDs in sorted order. Overrides are intersected with discovered IDs; missing IDs are warned and skipped.
4. For each ID, runs:

   ```bash
   python scripts/run_eval.py --scenario_cfg ... --scenario_id <id> ...<forwarded args>
   ```

5. Prints a per-scenario success/failure summary at the end.
6. With `--average`, writes `average_eval_results.json` under the bench-level log directory (see [Section 4c](#4c-average-evaluation-metrics-average_eval_resultspy)).

### 4-batch.5 Where outputs go

Same tree as Section 4.4, one `scenario_<id>/` folder per subprocess:

```text
log/adv_train/<mode>/<policy>/<agent_cfg>_epoch<N>/<scenario_cfg>/<bench_id>/
  scenario_1/.../eval_results/OPT_scenario_001_ROUTE-0_results.pkl
  scenario_2/.../eval_results/OPT_scenario_002_ROUTE-0_results.pkl
  ...
  average_eval_results.json          # if --average was passed
```

For `train_scenario`, each bench folder also gets:

```text
safebench/scenario/scenario_data/.../<bench_id>/scenario_<id>.json
```

### 4-batch.6 Model / policy notes

- Default ego policy is SAC (`--test_policy` defaults to `sac`). With [adv_scenic.yaml](safebench/agent/config/adv_scenic.yaml), use **`--test_policy ppo`** and a matching `pretrain_dir` checkpoint.
- Tune optimization cost in the scenario YAML: `sample_num`, `opt_step`, `select_num` (e.g. [eval_scenic.yaml](safebench/scenario/config/eval_scenic.yaml)).
- `--average` matches `OPT_scenario_*_ROUTE-0_results.pkl`; pass **`--route_id 0`** when averaging if your YAML lists multiple routes.
- Use a valid `--scenario_id` for the bench (e.g. UN_R152 has scenarios 1, 2, 4 — not 0).

### 4-batch.7 Notes

- Subprocess isolation: a CARLA crash in one scenario does not kill the batch interpreter; the next scenario still runs (unless `--no-continue_on_error`).
- Re-running the batch is safe for partial completion; already-finished routes are skipped inside `run_eval.py`.
- Discovery follows the bench layout (`scenic_dir/<bench_id>/*.scenic`). Set `bench_id` in the scenario YAML before running.
- This script wraps `run_eval.py` (JSON-based workflow). For JSON-free eval over a bench, use [Section 4b-batch](#4b-batch-batch-json-free-eval-run_eval_scenic_batchpy) instead.

---

## 4-v2) Running SafeBench — Scenic 2.x (`run_eval_v2.py`)

[scripts/run_eval_v2.py](scripts/run_eval_v2.py) is the Scenic **2.x** counterpart of [scripts/run_eval.py](scripts/run_eval.py).

### 4-v2.1 Differences from `run_eval.py`

| | **`run_eval.py`** | **`run_eval_v2.py`** |
|---|---|---|
| Default scenario YAML | [eval_scenic.yaml](safebench/scenario/config/eval_scenic.yaml) | [eval_scenic_v2.yaml](safebench/scenario/config/eval_scenic_v2.yaml) |
| Scenic runner | [ScenicRunner](safebench/scenic_runner.py) | [ScenicRunnerV2](safebench/scenic_runner_v2.py) |
| Scenic utils | [scenic_utils.py](safebench/util/scenic_utils.py) (Scenic 3) | [scenic_utils_v2.py](safebench/util/scenic_utils_v2.py) (Scenic 2) |
| Parse helper | [scenario_utils.scenic_parse](safebench/scenario/tools/scenario_utils.py) | [scenario_utils_v2.scenic_parse](safebench/scenario/tools/scenario_utils_v2.py) |
| Modes | `train_agent`, `train_scenario`, `eval` | `train_scenario`, `eval` only |
| Python env | Scenic 3 | `scenic2-venv` ([env.scenic2.sh](env.scenic2.sh)) |
| Route pickle | v1 ([generate_scenic_route_pickle.py](scripts/generate_scenic_route_pickle.py)) | v2 ([generate_scenic_route_pickle_v2.py](scripts/generate_scenic_route_pickle_v2.py)) recommended |

Both scripts set `num_scenario = 1` for scenic policy and pass `--route_id` as a one-element list.

### 4-v2.2 Scenario YAML example (`eval_scenic_v2.yaml`)

Same keys as `eval_scenic.yaml`; point `route_dir` at a pickle produced by v2 (or any unified-format pickle):

```yaml
policy_type: 'scenic'
scenario_category: 'scenic'

route_dir: 'safebench/scenario/scenario_data/route_debug'
scenic_dir: 'safebench/scenario/scenario_data/scenic_data_chatscene/chatscene_overlap_nl2scenic_route-driven'
bench_id: 'NHTSA_PreCrash'

sample_num: 50
opt_step: 10
select_num: 2

method: 'scenic'
route_id: [0]

ego_action_dim: 2
ego_state_dim: 4
ego_action_limit: 1.0
```

See [Section 2-v2.2](#2-v2.2-pickle-contents-coordinate-formats-important) for the unified pickle schema shared by v1 and v2.

### 4-v2.3 Example eval commands

```bash
source env.scenic2.sh

# Step 1: train_scenario (OPT selection on surrogate SAC)
python scripts/run_eval_v2.py \
  --mode train_scenario \
  --scenario_cfg eval_scenic_v2.yaml \
  --scenario_id 22 \
  --route_id 0 \
  --device cuda \
  --test_policy sac

# Step 2: eval (after scenario_<id>.json exists under scenic_dir/bench_id/)
python scripts/run_eval_v2.py \
  --mode eval \
  --scenario_cfg eval_scenic_v2.yaml \
  --scenario_id 22 \
  --route_id 0 \
  --device cuda \
  --test_policy sac \
  --save_video
```

Log layout matches v1, with `eval_scenic_v2` in the path instead of `eval_scenic`:

```text
log/adv_train/<mode>/sac/adv_scenic_epochNone/eval_scenic_v2/<bench_id>/scenario_<id>/...
```

### 4-v2.4 End-to-end Scenic 2.x pipeline

```text
1. generate_scenic_route_pickle_v2.py  →  scenic_route.pickle (unified Scenic-coord schema)
2. migrate_opt_range_params.py         →  OPT_* JSON (if needed)
3. run_eval_v2.py --mode train_scenario →  scenario_<id>.json (OPT bounds + select_id)
4. run_eval_v2.py --mode eval           →  eval_results/*.pkl, optional videos
```

### 4-v2.5 Troubleshooting: ego not moving

If the ego sits still until timeout (`route_completion: 0.0`, near-zero `avg_acceleration`):

1. Confirm eval uses **`run_eval_v2.py`** with the correct `route_dir` and `bench_id`.
2. Sanity-check stored coords: `trajectory[0].y` should match `waypoints[0].y` (both Scenic). If `trajectory[0].y ≈ -waypoints[0].y`, the entry is a **legacy** v2 pickle — run `migrate_v2_route_pickle_to_scenic.py` or regenerate with `--no-resume`.
3. After migration/regeneration, CARLA spawn Y should equal `-trajectory[0].y` (Scenic → CARLA flip). A ~2×|y| meter error usually means double Y-flip.

v1 pickles (Scenic coords, no `route_format`) continue to work with the default `"scenic"` conversion path — see [Section 2-v2.3](#2-v2.3-loader-behavior-scenicdataloader).

---

## 4b) JSON-free one-time eval (`run_eval_scenic.py`)

Use this when you want to evaluate the ego agent on scenes sampled **directly from the OPT parameter ranges written in the `.scenic` code**, WITHOUT the OPT-selection JSON produced by `train_scenario`.

### 4b.1 When to use it

- You have NOT run `train_scenario` (no `scenario_<id>.json`), or
- You want to ignore the optimized/narrowed ranges and sample from the raw Scenic-code ranges, or
- You just want a quick one-shot run.

Note: the route pickle (`scenic_route.pickle`) is still required (it provides `spawnPt` / `trajectory`). Only the OPT JSON is skipped.

### 4b.2 Command

```bash
python scripts/run_eval_scenic.py \
  --scenario_cfg eval_scenic_wenting.yaml \
  --scenario_id 1 \
  --route_id 0 \
  --num-scenes 1 \
  --port 2002 \
  --tm_port 8002 \
  --device cpu \
  --save_video
```

- `--num-scenes N`: how many scenes to sample from the Scenic code ranges and run (default `1`).
- The script forces `mode=eval` internally but does NOT read any JSON.

### 4b.3 How it works

- The script sets `use_opt_json=False` in the scenario config plus `sample_num = opt_step = --num-scenes`.
- `scenic_parse` ([safebench/scenario/tools/scenario_utils.py](safebench/scenario/tools/scenario_utils.py)) skips loading the OPT JSON when `use_opt_json=False`.
- `ScenicDataLoader` ([safebench/scenario/scenario_data_loader.py](safebench/scenario/scenario_data_loader.py)) generates scenes via `train_scene()` (sampling from the code ranges) and runs all of them.
- The agent-eval loop and result/video saving are identical to `run_eval.py`.

### 4b.4 Where outputs go

Separate `eval_scenic` log tree (keeps these runs apart from regular `eval`):

```text
log/adv_train/eval_scenic/<policy>/<agent_cfg>_epoch<N>/<scenario_cfg>/<bench_id>/scenario_<id>/<exp>_seed_0/
```

Example:

```text
log/adv_train/eval_scenic/sac/adv_scenic_epochNone/eval_scenic_wenting/UN_R152/scenario_1/.../eval_results/
    OPT_scenario_001_ROUTE-0_results.pkl
    OPT_scenario_001_ROUTE-0_records.pkl
```

Plus `video/<log_name>/*.mp4` if `--save_video`.

### 4b.5 Notes

- With `--num-scenes 1`, scene generation is deterministic (`random.seed(0)`), so repeated runs sample the same scene. Increase `--num-scenes` for multiple distinct samples.
- `run_eval.py` is unchanged: `use_opt_json` defaults to `True`, so the standard JSON-based workflow (Section 4.3) behaves exactly as before.
- If the `.scenic` file defines no `OPT_*` Range params, this script and the JSON path sample the same way (from the code's non-OPT randomness); the JSON only adds `select_id`.

---

## 4b-batch) Batch JSON-free eval (`run_eval_scenic_batch.py`)

Use this when you want to run [Section 4b](#4b-json-free-one-time-eval-run_eval_scenicpy) over **many scenario IDs** in one command. The batch script auto-discovers all `scenario_*.scenic` files under `scenic_dir/<bench_id>` (from the scenario YAML), then spawns one `run_eval_scenic.py` subprocess per scenario.

Pickle files, videos, and log layout are **identical** to running `run_eval_scenic.py` manually for each `--scenario_id`.

### 4b-batch.1 When to use it

- Evaluate an entire bench (e.g. all `NHTSA_Crash` or `UN_R171` scenarios) without scripting a shell loop.
- Re-run a bench after partial completion (completed routes are skipped by `ScenicRunner` via `logger.check_eval_dir`).
- Optionally average metrics across the bench in one step with `--average`.

### 4b-batch.2 Command

Evaluate all discovered scenarios:

```bash
python scripts/run_eval_scenic_batch.py \
  --scenario_cfg eval_scenic_wenting.yaml \
  --route_id 0 \
  --num-scenes 3 \
  --port 2002 \
  --tm_port 8002 \
  --device cpu \
  --save_video \
  --average
```

Evaluate a subset only:

```bash
python scripts/run_eval_scenic_batch.py \
  --scenario_cfg eval_scenic_wenting.yaml \
  --scenario_range 1-9 \
  --route_id 0 \
  --num-scenes 1
```

Explicit scenario IDs (combinable with `--scenario_range`):

```bash
python scripts/run_eval_scenic_batch.py \
  --scenario_cfg eval_scenic_wenting.yaml \
  --scenario_ids 1 3 9 \
  --route_id 0 \
  --num-scenes 1
```

Preview commands without running CARLA:

```bash
python scripts/run_eval_scenic_batch.py \
  --scenario_cfg eval_scenic_wenting.yaml \
  --dry_run
```

### 4b-batch.3 Flags

**Batch-specific** (consumed by `run_eval_scenic_batch.py`):

| Flag | Purpose |
|------|---------|
| `--scenario_ids` | Optional explicit list of scenario IDs |
| `--scenario_range` | Inclusive range, e.g. `1-9` |
| `--continue_on_error` | Default `True`; keep going if one subprocess fails |
| `--no-continue_on_error` | Stop the batch on the first failure |
| `--dry_run` | Print discovered IDs and subprocess commands only |
| `--average` | After all runs, call `average_eval_results.py` on the bench output dir |
| `--average_output` | Override path for `average_eval_results.json` |

**Forwarded to `run_eval_scenic.py`** (pass them on the same command line; unrecognized flags are forwarded verbatim):

`--agent_cfg`, `--scenario_cfg`, `--route_id`, `--num-scenes`, `--port`, `--tm_port`, `--device`, `--seed`, `--save_video`, `--test_policy`, `--test_epoch`, `--threads`, `--max_episode_step`, `--frame_skip`, `--fixed_delta_seconds`, `--auto_ego`

`--scenario_cfg` is **required** (used for scenario discovery and passed through to each subprocess).

### 4b-batch.4 How it works

1. Loads `safebench/scenario/config/<scenario_cfg>` to get `scenic_dir` and `bench_id`.
2. Scans `scenic_dir/<bench_id>/scenario_*.scenic` and extracts numeric IDs (e.g. `scenario_011.scenic` → `11`).
3. If neither `--scenario_ids` nor `--scenario_range` is given, runs **all** discovered IDs in sorted order. Overrides are intersected with discovered IDs; missing IDs are warned and skipped.
4. For each ID, runs:

   ```bash
   python scripts/run_eval_scenic.py --scenario_cfg ... --scenario_id <id> ...<forwarded args>
   ```

5. Prints a per-scenario success/failure summary at the end.
6. With `--average`, writes `average_eval_results.json` under the bench-level log directory (see [Section 4c](#4c-average-evaluation-metrics-average_eval_resultspy)).

### 4b-batch.5 Where outputs go

Same tree as Section 4b.4, one `scenario_<id>/` folder per subprocess:

```text
log/adv_train/eval_scenic/<policy>/<agent_cfg>_epoch<N>/<scenario_cfg>/<bench_id>/
  scenario_1/.../eval_results/OPT_scenario_001_ROUTE-0_results.pkl
  scenario_2/.../eval_results/OPT_scenario_002_ROUTE-0_results.pkl
  ...
  average_eval_results.json          # if --average was passed
```

### 4b-batch.6 Model / policy notes

- Default ego policy: SAC pretrain from `pretrain_dir` in [safebench/agent/config/adv_scenic.yaml](safebench/agent/config/adv_scenic.yaml) (typically `safebench/agent/model_ckpt/safe_rl/sac/model_save/model.pt`).
- Use a different algorithm: `--test_policy td3` (or `ddpg`, `ppo`) and set `pretrain_dir` in `adv_scenic.yaml` to the matching checkpoint (e.g. `safebench/agent/model_ckpt/safe_rl/td3_0/model_save/model.pt`).
- Use per-scenario adversarial checkpoints: pass `--test_epoch -1` to load `model.sac.-001.torch` from `load_dir/scenario_<id>/` (SAC naming only).

Without `--test_epoch`, only `pretrain_dir` weights are used; `load_dir/scenario_<id>/` is printed but not loaded.

### 4b-batch.7 Notes

- Subprocess isolation: a CARLA crash in one scenario does not kill the batch interpreter; the next scenario still runs (unless `--no-continue_on_error`).
- Re-running the batch is safe for partial completion; already-finished routes are skipped inside `run_eval_scenic.py`.
- Discovery follows the bench layout (`scenic_dir/<bench_id>/*.scenic`). Set `bench_id` in the scenario YAML (e.g. [eval_scenic_wenting.yaml](safebench/scenario/config/eval_scenic_wenting.yaml)) before running.

---

## 4c) Average evaluation metrics (`average_eval_results.py`)

After running eval (Section 4.3, 4-batch, 4b, or 4b-batch), each scenario writes a results pickle under `eval_results/`:

```text
OPT_scenario_001_ROUTE-0_results.pkl
```

Each file is a dict of metrics (e.g. `collision_rate`, `route_completion`, `safety_os`, `task_os`, `comfort_os`, `final_score`).

Use `scripts/average_eval_results.py` to recursively find all matching `OPT_scenario_XXX_ROUTE-0_results.pkl` files under a directory and compute the mean of each metric.

### 4c.1 Command

```bash
python scripts/average_eval_results.py \
  log/adv_train/eval_scenic/sac/adv_scenic_epochNone/eval_scenic_wenting/NHTSA_Crash
```

Optional custom output path:

```bash
python scripts/average_eval_results.py \
  log/adv_train/eval_scenic/sac/adv_scenic_epochNone/eval_scenic_wenting/NHTSA_Crash \
  --output /path/to/my_averages.json
```

### 4c.2 What it does

- Recursively searches the given directory for files matching `OPT_scenario_XXX_ROUTE-0_results.pkl` (where `XXX` is a numeric scenario id, e.g. `001`, `008`).
- Ignores companion `*_records.pkl` files.
- Loads each results pickle and averages all numeric metrics.
- Writes JSON to `<directory>/average_eval_results.json` by default.

### 4c.3 Output format

```json
{
  "num_pickle_files": 9,
  "source_files": [
    "scenario_1/scenario_1_rl_scenic_seed_0/eval_results/OPT_scenario_001_ROUTE-0_results.pkl",
    "scenario_2/scenario_2_rl_scenic_seed_0/eval_results/OPT_scenario_002_ROUTE-0_results.pkl"
  ],
  "average_metrics": {
    "collision_rate": 0.222222,
    "route_completion": 0.509734,
    "final_score": 0.818882
  }
}
```

- `num_pickle_files`: how many result pickles were averaged.
- `source_files`: relative paths of the pickles used (for auditing).
- `average_metrics`: mean of each metric across all matched files.

Point the script at a bench-level folder (e.g. `.../NHTSA_Crash`) to average across all scenarios under it, or at a single `scenario_<id>/...` folder to average only that scenario's results.

---

## 5) Common issues

### 5.1 “Unable to spawn object unnamed Pedestrian”

This is a Scenic/CARLA spawn failure during scene instantiation. It can occur intermittently based on sampled parameters.

- Route generation script retries sampling (`--max-scene-attempts`).
- SafeBench eval may also print repeated failures before one scene succeeds.

### 5.2 No configs run in eval mode

If `eval` exits immediately, it usually means the OPT-selection JSON was not found/loaded.

Run `train_scenario` first to generate `scenario_<id>.json` under the bench folder, then run `eval`.


### Easy Start


python scripts/generate_scenic_route_pickle.py   --scenic-file safebench/scenario/scenario_data/scenic_data_wenting/Self_Gemini_3flash_R1_original/UN_R171/scenario_002.scenic   --out-pickle safebench/scenario/scenario_data/route_wenting/scenic_route.pickle  --max-scene-attempts 200   --warmup-ticks 20   --record-video  --fixed-delta-seconds 0.1 --max-record-seconds 30   --no-resume




python scripts/generate_scenic_route_pickle.py   --scenic-dir safebench/scenario/scenario_data/scenic_data_wenting/Self_Gemini_3flash_R1_original/ChatScene   --glob '*.scenic'   --out-pickle safebench/scenario/scenario_data/route_wenting/scenic_route.pickle   --bench-id ChatScene   --max-scene-attempts 200   --warmup-ticks 20   --record-video  --fixed-delta-seconds 0.1 --max-record-seconds 40