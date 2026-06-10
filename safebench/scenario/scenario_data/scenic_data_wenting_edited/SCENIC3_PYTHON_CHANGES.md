# Scenic 3.x Python Integration Changes

This document describes **Python and YAML-only** changes made to run [`scenic_data_wenting`](.) (Scenic 3.x `.scenic` files) through the ChatScene / Safebench pipeline (`scripts/run_train.py`, `train_scenario` / `train_agent` / `eval`). No `.scenic` files were modified.

**Target environment:** `conda activate chatscene` with **Scenic 3.1.1** (pip). CARLA is expected on port **2002** by default (`run_train.py --port 2002`). CARLA **0.9.15** has been used successfully alongside these changes; the edits are version-agnostic at the Python layer.

**Related references:**

- [ROUTE_DRIVEN_SCENARIO1_REFERENCE.md](ROUTE_DRIVEN_SCENARIO1_REFERENCE.md)
- [ROUTE_DRIVEN_SCENARIO2_SCENIC3_REFERENCE.md](ROUTE_DRIVEN_SCENARIO2_SCENIC3_REFERENCE.md)

---

## Summary

| Area | Change |
|------|--------|
| Scenic compile | `scenarioFromFile(..., mode2D=True)` + global params `use2DMap`, `render`, `address`, `timestep` |
| Simulation lifecycle | New `SafebenchCarlaSimulation` — setup-only spawn for co-simulation with Safebench `world.tick()` |
| OPT parameter I/O | Only `Range`-like `OPT_*` params are optimized; fixed scalars (e.g. `OPT_STOP_THRESHOLD = 0.8`) are skipped |
| Ego / adversaries | Resolve ego via `simulation.ego` / `scene.egoObject`; adversaries = non-ego objects with a behavior |
| Trajectory | Fallbacks from `waypoints` and `egoTrajectory.points` in scene params |
| Config | `scenic_mode2d: true` in wenting YAML configs |

---

## Architecture (co-simulation)

Scenic 2 ran the simulation loop inside a manually stepped generator. Scenic 3’s stock `Simulation.__init__` runs the **full** loop and calls `destroy()` before returning. Safebench still needs actors to stay alive while the RL stack calls `world.tick()` each step.

```mermaid
flowchart LR
    parse[scenic_parse] --> extra[extra_params + scenic_mode2d]
    extra --> init[ScenicSimulator]
    init --> gen[generateScene]
    gen --> setSim[setSimulation via SafebenchCarlaSimulation]
    setSim --> run[runSimulation generator]
    run --> sb[Safebench env.step / world.tick]
    sb --> end[endSimulation]
```

**Scene validation** (`ScenicDataLoader.generate_scene`): `setSimulation(..., max_steps=1)` spawns actors once to reject invalid scenes, then `endSimulation()` tears down before the next sample.

**Episode run** (`ScenicRunner.run_scenes`): `setSimulation()` with default `max_steps` (from parser `--time`, default 10000), then `runSimulation()` steps adversary behaviors; Safebench advances CARLA separately.

---

## Files changed

### 1. [`safebench/util/scenic_utils.py`](../../../util/scenic_utils.py)

Main Scenic 3 adapter.

#### New helpers

- **`_is_opt_range(param)`** — Returns true when a global param has `.low` and `.high` (Scenic `Range`). Used to exclude fixed `OPT_*` scalars from optimization and JSON export.
- **`merge_scenic_global_params(user_params, fixed_delta_seconds)`** — Merges Safebench route/CARLA settings with Scenic 3 defaults:
  - `use2DMap: True` (required for `.xodr` maps in Scenic 3.1)
  - `render: 0`
  - `address: '127.0.0.1'`
  - `timestep: fixed_delta_seconds` (default 0.1)

#### New class: `SafebenchCarlaSimulation`

Subclass of Scenic 3 `CarlaSimulation` that:

- Accepts required Scenic 3 kwargs: `maxSteps`, `name`, `timestep`, `verbosity`
- Runs `veneer.beginSimulation` → `setup()` → `dynamicScenario._start()` → `updateObjects()`
- Does **not** call `Simulation._run()` or auto-`destroy()` on success (actors remain for Safebench)
- On failure: `_safebench_teardown()` destroys actors and ends veneer state

#### `ScenicSimulator` updates

| Method | Change |
|--------|--------|
| `__init__` | `scenarioFromFile(..., mode2D=self.mode2D)`; `merge_scenic_global_params`; imports from `scenic.core.simulators` / `scenic.core.dynamics` |
| `get_params` | Only `OPT_*` keys that pass `_is_opt_range` |
| `save_params` / `load_params` / `update_params` | Same range-only filter; `load_params` ignores entries without `low`/`high` |
| `setSimulation(scene, max_steps=None, name=None)` | Builds `SafebenchCarlaSimulation` instead of `simulator.createSimulation()`; passes `maxSteps` and `name`; catches `RejectSimulationException`, `RejectionException`, `GuardViolation` |
| `runSimulation` | Uses `_safebench_max_steps`; does not re-call `veneer.beginSimulation` / `_start` (already done in `setSimulation`) |
| `endSimulation` | Stops scenarios, records final exprs, `destroy()`, clears veneer |
| `_ego_object` | `simulation.ego` or `scene.egoObject` |

**Removed:** Duplicate local `TerminationType` / `SimulationResult` classes; use Scenic 3 imports.

**Removed:** Extra `self.simulation.setup()` after create (Scenic 3 setup runs inside `SafebenchCarlaSimulation.__init__`).

---

### 2. [`safebench/scenic_runner.py`](../../../scenic_runner.py)

`_init_scenic(config)`:

```python
mode2D = getattr(config, 'scenic_mode2d', True)
self.scenic = ScenicSimulator(
    config.scenic_file,
    config.extra_params,
    mode2D=mode2D,
    fixed_delta_seconds=self.fixed_delta_seconds,
)
```

---

### 3. [`safebench/scenic_runner_dynamic.py`](../../../scenic_runner_dynamic.py)

Same `_init_scenic` wiring as `scenic_runner.py` for dynamic scenic runs.

---

### 4. [`safebench/scenario/tools/scenario_utils.py`](../../tools/scenario_utils.py)

#### New: `_scenic_base_extra_params(config)`

Builds the dict merged into every `ScenarioConfig.extra_params`:

| Key | Value |
|-----|--------|
| `port` | `config['port']` |
| `traffic_manager_port` | `config['tm_port']` |
| `use2DMap` | `True` |
| `render` | `0` |
| `address` | `'127.0.0.1'` |
| `timestep` | `config.get('fixed_delta_seconds', 0.1)` |

#### `scenic_parse` and `dynamic_scenic_parse`

- Set `parsed_config.scenic_mode2d = config.get('scenic_mode2d', True)`
- Set `parsed_config.extra_params = _scenic_base_extra_params(config)` (replaces port-only dict)

---

### 5. [`safebench/scenario/scenario_definition/scenic_scenario.py`](../../scenario_definition/scenic_scenario.py)

#### New: `_resolve_ego_object()`

Resolution order:

1. `simulation.ego` (Scenic 3)
2. `simulation.scene.egoObject`
3. First object with `isCar` or type name `Car`
4. `simulation.objects[0]` (fallback)

#### `_update_route_and_ego()`

- Ego from `_resolve_ego_object()` instead of `simulation.ego.carlaActor` only
- **Adversaries:** every `simulation.objects` entry that is not ego and has `behavior is not None` (replaces `'Adv' in str(other_actor.behavior)`)
- Fixed actor pool registration: `adv_actor.id` used for adversary pool keys (was incorrectly using ego `actor.id`)

---

### 6. [`safebench/scenario/scenario_data_loader.py`](../../scenario_data_loader.py)

#### `generate_scene`

Validation probe uses short simulation:

```python
self.scenic.setSimulation(
    scene, max_steps=1, name=f'validate_{opt_time}_{len(scenes)}'
)
```

#### `sampler` trajectory fallbacks

After `egoTrajectoryPts`, also tries:

- `scene.params['waypoints']` (route-driven wenting files)
- `scene.params['egoTrajectory'].points` if present

---

### 7. YAML configs (not Python, but required for runs)

| File | Addition / note |
|------|------------------|
| [`train_scenario_scenic_wenting.yaml`](../../config/train_scenario_scenic_wenting.yaml) | `scenic_mode2d: true`; `scenic_dir` → `scenic_data_wenting/` |
| [`train_agent_scenic_wenting.yaml`](../../config/train_agent_scenic_wenting.yaml) | `scenic_mode2d: true` |
| [`eval_scenic_wenting.yaml`](../../config/eval_scenic_wenting.yaml) | `scenic_mode2d: true` |

---

## Bugs addressed by these changes

| Error | Cause | Fix |
|-------|--------|-----|
| `SpecifierError: property "heading" cannot be directly specified` | Scenic 3 3D mode | `mode2D=True` at compile |
| `TypeError: missing 'maxSteps' and 'name'` | Scenic 3 `Simulation` API | `SafebenchCarlaSimulation` + `setSimulation` kwargs |
| Full sim runs on spawn; actors destroyed | Scenic 3 `Simulation.__init__` | `SafebenchCarlaSimulation` (setup only) |
| `AttributeError: 'float' object has no attribute 'low'` | Fixed `OPT_*` scalars in wenting `.scenic` | `_is_opt_range` in param get/save/load/update |
| `KeyError: 'town'` when compiling without routes | Missing Safebench `extra_params` | `_scenic_base_extra_params` + route pickle fields |

---

## OPT parameters: range vs fixed

Wenting scenarios mix:

```scenic
param OPT_DISTANCE_AHEAD = Range(20, 30)   # optimized — saved in scenario_N.json
param OPT_STOP_THRESHOLD = 0.8             # fixed — not in opt_params / save_params
```

Only **Range-like** `OPT_*` keys appear in `scenario_1.json` `opt_time_*` entries after `train_scenario`. Fixed constants remain in the Scenic file and sampled scenes as-is.

---

## How to run

```bash
conda activate chatscene
cd /path/to/ChatScene-main

# Start CARLA (match --port, default 2002)
./CarlaUE4.sh -prefernvidia -RenderOffScreen -carla-port=2002

python scripts/run_train.py \
  --agent_cfg=adv_scenic.yaml \
  --scenario_cfg=train_scenario_scenic_wenting.yaml \
  --mode train_scenario \
  --scenario_id 1
```

Optional flags: `--port`, `--tm_port`, `--device cpu`, `--fixed_delta_seconds 0.1`.

---

## Known limitations (unchanged by Python work)

1. **`.scenic` syntax:** Some files in other scenarios still use Scenic 2 monitor syntax (`monitor Name:` without `()`). Not fixed by `mode2D`.
2. **`behavior_2_opt.scenic` (scenario_1):** May fail at runtime/sampling (`undefined intersection` in a `require`) — needs a `.scenic` edit if you want that behavior to run.
3. **Bundled repo `ChatScene-main/Scenic`:** Still Scenic 2.1.0b4; runtime must use pip Scenic 3 in `chatscene`, not the bundled tree.
4. **Dual Scenic 2 + 3:** Not supported; only `scenic_data_wenting` path was targeted.

---

## File checklist

| Path | Modified |
|------|----------|
| `safebench/util/scenic_utils.py` | Yes |
| `safebench/scenic_runner.py` | Yes |
| `safebench/scenic_runner_dynamic.py` | Yes |
| `safebench/scenario/tools/scenario_utils.py` | Yes |
| `safebench/scenario/scenario_definition/scenic_scenario.py` | Yes |
| `safebench/scenario/scenario_data_loader.py` | Yes |
| `safebench/scenario/config/train_scenario_scenic_wenting.yaml` | Yes |
| `safebench/scenario/config/train_agent_scenic_wenting.yaml` | Yes |
| `safebench/scenario/config/eval_scenic_wenting.yaml` | Yes |
| `scripts/run_train.py` | No (uses existing `--port` / `--tm_port`) |
| `scenic_data_wenting/**/*.scenic` | No |

---

*Last updated to reflect integration work for Scenic 3.1.1 + `scenic_data_wenting`.*
