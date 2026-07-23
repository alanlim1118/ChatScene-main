---
name: route-driven-scenic-conversion
description: Convert an adversarial Scenic (.scenic) scenario — where Scenic scripts both the ego and the adversary — into a route-driven scenario where SafeBench/an RL agent drives the ego along a fixed route and Scenic only scripts the adversary/pedestrian/prop. Use whenever asked to make .scenic file(s) "route-driven", to rewrite scenarios under a `scenic_route-driven` (or similarly named) directory, or to port scenarios from an original/adversarial bench into the route-driven pipeline described in ROUTE_DRIVEN_SCENIC_GUIDE.md.
---

# Route-Driven Scenic Conversion

Converts an original adversarial Scenic scenario into a route-driven one: pin the
ego to a fixed spawn point supplied by a route pickle, delete the ego's own
Scenic behavior, and re-anchor the adversary so it keeps the same relative
geometry it had in the original. Everything else — adversary/pedestrian/prop
behaviors, monitors, `require`s, `terminate` clauses, blueprints — is preserved
byte-for-byte wherever possible.

If the repo has `ROUTE_DRIVEN_CONVERSION_PLAYBOOK.md` at its root, read it first —
this skill is the condensed, battle-tested operating procedure distilled from
that playbook plus lessons learned converting 100 real files in this repo
across two benches. If the two ever disagree, this file reflects what
actually worked; update the playbook to match rather than trusting it
blindly.

## 0) Before touching any file

1. **Find the original (unconverted) scenario.** The target file under
   `scenic_route-driven/<bench>/` (or similar) usually already exists as an
   exact copy of the original — diff it against its source directory (e.g.
   `.../scenic/<bench>/` or `Self_Gemini_3flash_R1_original/<bench>/`) to
   confirm it hasn't been converted yet. If diff is empty, it needs conversion.
2. **Find the route pickle** that was generated *from* this original file
   (`scenic_route.pickle` + `scenic_route_index.json`, usually sitting next to
   the original `.scenic` files, or in a sibling `route_*` directory per
   `ROUTE_DRIVEN_SCENIC_GUIDE.md` §1–2). This pickle stores a `spawnPt` sampled
   by actually *running* the original scenario, so the fixed ego position you
   pin to is guaranteed to satisfy the original's `require`s — you don't need
   to re-derive or second-guess it.
3. **Set up an offline smoke-test harness** (see §7) before converting more
   than one or two files — you want fast, cheap feedback per file, not a
   review pass at the end.

## 1) Mental model

| | Original scenario | Route-driven scenario |
|---|---|---|
| Ego position | sampled (`Uniform(*network.lanes)`, `on lane.centerline`, or relative to adversary) | **fixed**: `globalParameters.spawnPt` + `globalParameters.yaw` |
| Ego behavior | a Scenic `EgoBehavior` | **none** — SafeBench/RL steps the ego; any ego Scenic behavior is ignored |
| Adversary | placed relative to ego, own behavior | **unchanged in intent** — re-anchored so it's placed relative to the now-fixed ego |
| Termination, monitors, requires | as written | **copied verbatim** |

The whole job: *pin the ego, invert the spawn chain so the adversary keeps the
same relative geometry, delete the ego behavior, touch nothing else.*

## 2) Hard constraints

1. **Minimal diff.** Only change: (a) the ego spawn source, (b) removal of the
   ego Scenic behavior + ego-only params, (c) the adversary anchor when it was
   expressed relative to a previously-sampled ego.
2. **Preserve the adversary wholesale.** Keep every adversary/pedestrian/prop
   `behavior` block, `monitor` block, `require monitor`, `regionContainedIn`,
   blueprint, and distance/speed range — even ones that look unrelated to the
   ego at first glance.
3. **Never touch termination.** Copy every `terminate when` / `terminate
   after` line byte-for-byte, including awkward forms like `distance to
   egoSpawnPt` (no `from ego`). The **only** sanctioned diff: a bare
   `terminate` statement that lived *inside* the now-deleted `EgoBehavior`
   block disappears along with the block — that's not a scenario-level clause.
4. **Keep every `require`, even ones that mention `egoSpawnPt` or
   `intersection`.** It is tempting to think "the ego position is fixed now,
   so this require about `distance from egoSpawnPt to intersection` is
   pointless" — it is not pointless, it's still a `require` and rule #2 in the
   playbook says preserve requires verbatim. It will hold true anyway (the
   route pickle's spawn point was sampled from a scene that satisfied it), so
   preserving it costs nothing and keeps the diff honest.
   **Mistake seen in practice:** dropping the `distance from egoSpawnPt to
   intersection` and heading-difference `require`s because they "belong to"
   the ego. They don't — they're scenario-level geometric constraints. Keep
   all of them.
5. **Spawn-chain inversion is geometry-preserving.** If the original said "ego
   is 20m behind the adversary", the route-driven file must say "adversary is
   20m ahead of the ego" using the same distance/range.

## 3) The recipe, per file

### 3.1 Pin the ego

```scenic
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)   # see §4 — do not use `facing yaw`
```

Then derive lane/intersection context from the fixed point instead of
sampling it:

| Original ego sampling | Route-driven replacement |
|---|---|
| `Uniform(*network.lanes)` / section scans | `egoInitLane = network.laneAt(egoSpawnPt.position)` |
| section-level work (`_laneToLeft`, `_laneToRight`, projections) | `egoLaneSec = network.laneSectionAt(egoSpawnPt)` |
| `Uniform(*intersection.incomingLanes)` for ego | `egoInitLane = network.laneAt(egoSpawnPt.position)`, then filter `egoInitLane.maneuvers` |
| `intersection = Uniform(*filter(..., network.intersections))` then `egoManeuver = Uniform(*filter(TYPE, intersection.maneuvers))` | `egoManeuver = Uniform(*filter(lambda m: m.type is TYPE and m.intersection is not None and <same intersection predicate>, egoInitLane.maneuvers))` then `intersection = egoManeuver.intersection` (every `Maneuver` has an `.intersection` attribute — use it instead of re-sampling the intersection; the `m.intersection is not None` guard is mandatory — see §6) |

Delete the loop/list that built the candidate set the original `Uniform(...)`
sampled from (e.g. `laneSecsWithLeftLane = [...]` plus its `require
len(...) > 0`) — it existed only to constrain that sampling, which no longer
happens. Don't leave it behind as dead code.

### 3.2 Ego actor

```scenic
ego = new Car at egoSpawnPt,
    with blueprint MODEL          # keep the original blueprint / model constant
    # no `with behavior EgoBehavior(...)`
```

- `new Car at <point>` takes its heading from the road, not from the point, so
  the ego always aligns to the lane regardless of `egoSpawnPt`'s `facing`
  value — this is *why* the yaw bug (§4) stays hidden until an adversary uses
  the point's heading.
- Preserve `with regionContainedIn <original ego section>` if the original had
  one.
- Match the original's own style for whether `with rolename 'hero'` is
  present — don't add it if the original bench never used it (minimal diff).

### 3.3 Invert the adversary spawn

| Original pattern | Inverted (fixed ego) |
|---|---|
| `egoSpawnPt = ... behind advSpawnPt by D` | `advSpawnPt = following egoInitLane.orientation from egoSpawnPt for D` (or `ahead of egoSpawnPt by D` only after §4 fix) |
| `X = following roadDirection from Y for Range(-b, -a)` (X behind Y) | invert to `Y = following roadDirection from X for Range(a, b)` (X ahead of Y) when X is now derived from the fixed ego |
| `advSpawnPt in adjLane.centerline` + `project(egoSpawnPt)` | derive `adjLane`/`adjSection` from `laneSectionAt(egoSpawnPt)` (`_laneToLeft`/`_laneToRight`); keep the same `project` + offset chain |
| intersection conflict: `advSpawnPt in advInitLane.centerline` | keep the adv spawn line unchanged; just derive `advManeuver` from `egoInitLane.maneuvers` → `egoManeuver.conflictingManeuvers` / `.reverseManeuvers` instead of a globally-sampled `intersection` |

**Prefer a VectorField over a scalar heading** for placement:
`following roadDirection from egoSpawnPt for D` or
`following egoInitLane.orientation from egoSpawnPt for D` or
`adjSection.centerline.project(egoSpawnPt.position)`. These are independent of
`egoSpawnPt`'s heading and immune to the yaw pitfall. Only reach for
`egoSpawnPt.heading` / `ahead of egoSpawnPt` / `behind egoSpawnPt` when you
actually need the point's orientation, and then make sure §4 is applied at the
source — every *downstream* use of `egoSpawnPt.heading` is then automatically
correct, because the fix lives in one place (the `facing (-(yaw + 90) deg)`
assignment). You do not need to touch each individual `.heading` reference.

### 3.4 Always remove

- the `behavior EgoBehavior(...)` (or similarly-named ego-only behavior, e.g.
  `EgoDriveAndBrake`) definition,
- `with behavior EgoBehavior(...)` on the ego actor,
- ego-only params/constants that are referenced **only** inside the removed
  behavior (`EGO_SPEED`, `OPT_EGO_SPEED`, `OPT_SHARP_STEER`,
  `LOSS_CONTROL_STEER`, `egoTrajectory`, `egoTrajectoryLine`, etc.) — but
  **grep first**: if an adversary/pedestrian behavior also reads that
  variable (e.g. a pedestrian's `stop_reference` set to `egoTrajectoryLine`),
  it is not ego-only — keep it.

If an **adversary** param referenced a removed ego param (e.g. `param
OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.1, 1.2, 1.3)`),
substitute the ego param's own literal range in place, so the adversary's
distribution is numerically unchanged:
`param OPT_ADV_SPEED = Range(1, 5) * Uniform(1.1, 1.2, 1.3)` (where
`OPT_EGO_SPEED` had been `Range(1, 5)`).

### 3.5 Never change

- every `terminate when` / `terminate after` line (verbatim, per §2 rule 3),
- adversary/pedestrian/prop `behavior` blocks, `monitor` blocks, `require
  monitor`,
- **every** distance/speed `require` band (per §2 rule 4) — only rename the
  lane variable when it now comes from the fixed spawn point instead of a
  sampled one.

## 4) The yaw/heading pitfall — read this before writing any `facing`

**Root cause:** the route pickle stores `spawnPt['yaw']` as a **CARLA yaw in
degrees** (e.g. `-179.73`). Scenic's `facing` expects a **heading in
radians**. `facing yaw` silently interprets the degree value as radians and
produces a garbage heading.

- The ego itself doesn't care — `new Car at <point>` aligns to the road
  regardless of the point's `facing` value.
- It bites the moment an adversary is placed or oriented using
  `egoSpawnPt.heading` (`behind egoSpawnPt by D`, `with heading
  egoSpawnPt.heading + 180 deg`, `offsetRotated(egoSpawnPt.heading, ...)`,
  etc.) — the adversary then spawns tens of degrees off-axis, often off-lane,
  and can trip a naive `terminate when distance > X` almost immediately.

**Correct conversion**, from Scenic's own `carlaToScenicHeading`
(`scenic/simulators/carla/utils/utils.py`):

```text
scenic_heading_radians = normalizeAngle(-radians(carla_yaw_degrees + 90))
```

Write it once, at the ego spawn point definition:

```scenic
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
```

(`X deg` is Scenic's degrees→radians unit operator, so `-(yaw + 90) deg` ==
`-radians(yaw + 90)`; `facing` normalizes the angle.) Do this **once** and
every later `egoSpawnPt.heading` reference downstream is correct for free —
do not go hunt down and "fix" each individual `.heading` use.

**Sanity check before shipping:** grep for every place the point's own
heading is used:

```bash
rg -n 'egoSpawnPt\.heading|behind egoSpawnPt|ahead of egoSpawnPt|left of egoSpawnPt|right of egoSpawnPt|facing egoSpawnPt' <target-dir>
```

Confirm the file's `egoSpawnPt = ...` assignment uses the corrected form.
Files that anchor adversaries only via `roadDirection`, `lane.orientation`, or
`.centerline.project(...)` are unaffected by this pitfall entirely and need no
heading change — the smoke test's `rel` output (§7) is the final check either
way.

## 5) A known non-yaw pitfall: concrete vs. lazy-Distribution `+`

Watch for expressions that concatenate `PolylineRegion`s built from a mix of
a **now-concrete** lane (derived from the fixed `egoSpawnPt` via
`network.laneAt(...)`) and a **still-lazy** `Uniform(...)`-sampled maneuver:

```scenic
# BAD after conversion — egoInitLane is now concrete, egoManeuver is a Distribution:
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline
# fails at sample time with: AttributeError: 'PolylineRegion' object has no attribute '__radd__'
```

Fix by keeping every term in the chain rooted in the *same* lazy object,
mirroring how the original expressed it (the original's `egoInitLane` was
itself `egoManeuver.startLane`, i.e. already lazy):

```scenic
# GOOD — every term stays lazy, consistent with egoManeuver being a Distribution:
egoTrajectoryLine = egoManeuver.startLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline
```

Rule of thumb: if any term in a `PolylineRegion +` chain traces back to a
`Uniform(...)`/`Range(...)` pick, make sure **all** terms do (via
`<sampled-maneuver>.startLane...` rather than a separately-computed concrete
lane). This only surfaces once you've made the ego lane concrete — the
original file never hit it because everything upstream of `intersection =
Uniform(...)` was already lazy.

## 6) A third pitfall: `egoInitLane.maneuvers` picking a merge with no real intersection

**Root cause:** `Lane.maneuvers` includes every reachable next-lane connection,
not just ones that cross a real intersection — plain lane-to-lane merges are
maneuvers too, and Scenic's own `Maneuver` docstring says `intersection` is
`None` for these ("`None` for lane mergers"). The *original* scenario never
sees this because it samples from `intersection.maneuvers` — a set already
scoped to a validated real intersection. Once you convert that to
`egoInitLane.maneuvers` (§3.1's last table row), the candidate set is
whatever maneuvers exist on the lane the *fixed* point happens to sit on,
which can include a same-type merge maneuver that was never a real option in
the original.

**Symptom:** `intersection = egoManeuver.intersection` silently becomes
`None`, and later either:
- a `require`/`distance to intersection` expression crashes with
  `AttributeError` (no defensive filter), or
- `Uniform(*filter(...))` raises `InvalidScenarioError: tried to make
  discrete distribution over empty domain!` (defensive filter present, and
  the lane genuinely has zero matching real-intersection maneuvers).

**Fix — always required, cheap, never wrong:** filter for
`m.intersection is not None` *and* mirror whatever predicate the original
used to select the intersection (`is4Way`, `is3Way`, `isSignalized`, or a
combination) directly in the maneuver filter:

```scenic
egoManeuver = Uniform(*filter(lambda m:
    m.type is ManeuverType.STRAIGHT and
    m.intersection is not None and
    m.intersection.is4Way and m.intersection.isSignalized,
    egoInitLane.maneuvers))
intersection = egoManeuver.intersection
```

Apply this every single time you write an `egoInitLane.maneuvers` filter —
don't wait to see the crash first.

**What the filter does *not* fix.** If it results in an empty domain, that
means the fixed `egoSpawnPt` genuinely sits on a lane with zero matching
real-intersection maneuvers — a deeper problem than a missing guard. This
happens specifically when the *original* scenario placed `egoSpawnPt` itself
via an offset/chain from another actor (`following ... from leadSpawnPt for
D`, a spawn-chain-inversion case per §3.3) rather than directly
`new OrientedPoint in egoInitLane.centerline`:

- **Safe pattern** (the common case): original has
  `egoSpawnPt = new OrientedPoint in egoInitLane.centerline`. The recorded
  position is *by definition* on that lane, so
  `network.laneAt(egoSpawnPt.position)` reliably returns the same lane object
  every time.
- **Risky pattern** (spawn-chain inversion only): original derives
  `egoSpawnPt` from another point via `following roadDirection/orientation
  from X for D`. The pickled position (recorded after the route-pickle
  generator's warmup ticks — see `ROUTE_DRIVEN_SCENIC_GUIDE.md` §2.5) can
  drift onto a *different*, often preceding, lane segment than the one the
  maneuver-derivation logic expects. No maneuver-filter fix can repair this —
  the position itself doesn't match what the original's derivation would
  have produced. It can also surface one level down: the maneuver filter
  *does* find a candidate, but it's the only one and its geometry fails an
  angle/distance `require` that held for whatever maneuver the original
  actually used, so `generate()` exhausts `maxIterations` with a plain
  `RejectionException`.

**When you hit this:** apply the `m.intersection is not None` guard (always,
regardless), and if the failure persists — empty domain, or a
`RejectionException` after thousands of iterations — treat it as a
pickle-specific edge case, not a bug in your conversion. Don't rabbit-hole
rewriting the whole spatial-relations block to force a fix. A quick way to
confirm this diagnosis before giving up on a file:

```python
import scenic.domains.driving.roads as roads
from scenic.core.vectors import Vector
net = roads.Network.fromFile("path/to/assets/maps/CARLA/TownXX.xodr", useCache=False)
lane = net.laneAt(Vector(x, y))  # x, y from the route pickle's spawnPt
for m in lane.maneuvers:
    print(m.type, "intersection:", m.intersection.uid if m.intersection else None)
```

If that confirms the lane has no (or only a geometrically-wrong) matching
maneuver, log it as a known-limitation file in your summary and move on —
this was observed on real files (a merge-lane case and two angle-mismatch
cases) and is a property of that specific pickle, not something a better
Scenic rewrite fixes.

## 7) Offline smoke test (no live CARLA needed)

`scenarioFromFile(...).generate()` compiles the file, loads the map, and
samples a scene (running all `require`s and placement math) without
connecting to a CARLA server. Catches syntax errors, bad lane/maneuver
derivations, unsatisfiable requires, and heading/placement bugs.

Find a Python env with a matching Scenic + CARLA `.egg`/wheel already
installed (check for a project-local venv, e.g. `chatscene-v3/`, before
assuming you need to set one up):

```bash
source <venv>/bin/activate
python3 -c "from scenic import scenarioFromFile; import carla; print('ok')"
```

Reusable harness — mirrors exactly how the runner injects route params
(`safebench/scenario/tools/scenario_utils.py` → `extra_params` /
`safebench/util/scenic_utils.py` → `merge_scenic_global_params`). Adjust the
`ORIG_ROOT`/`RD_ROOT` and manifest-lookup logic to match the actual directory
layout you're working in (numeric `scenario_NNN.scenic` benches don't need the
manifest step at all — see `NL2SCENIC_BENCH_GUIDE.md` §1 for when a
`scenario_id_manifest.json` is involved):

```python
import json, os, pickle, math
from scenic import scenarioFromFile
from safebench.util.scenic_utils import merge_scenic_global_params

ORIG_ROOT = "path/to/original/<bench-parent>"   # where the route pickle lives
RD_ROOT   = "path/to/scenic_route-driven"

def term_lines(p):
    with open(p, encoding="utf-8") as f:
        return [l.rstrip("\n") for l in f if l.strip().startswith("terminate")]

def check(bench, filename):
    idx = json.load(open(os.path.join(ORIG_ROOT, bench, "scenic_route_index.json")))
    data_full = pickle.load(open(os.path.join(ORIG_ROOT, bench, "scenic_route.pickle"), "rb"))
    # if filenames aren't scenario_NNN.scenic, resolve scenario_id via the manifest
    # that lives next to the ORIGINAL files (it has every file, not just the
    # already-converted subset that may exist under scenic_route-driven):
    manifest = json.load(open(os.path.join(ORIG_ROOT, bench, "scenario_id_manifest.json")))
    file_to_id = {v: int(k) for k, v in manifest["id_to_file"].items()}
    sid = file_to_id[filename]
    entry = idx[f"{bench}:{sid}:0"]
    data = data_full[entry["pickle_key"]]
    sp = data["spawnPt"]
    params = merge_scenic_global_params({
        "spawnPt": (sp["x"], sp["y"]), "z": sp["z"], "yaw": sp["yaw"],
        "town": data["town"], "weather": data["weather"],
        "waypoints": data["waypoints"], "lanePts": data["lanePts"],
    }, 0.1)
    path = os.path.join(RD_ROOT, bench, filename)
    try:
        scn = scenarioFromFile(path, params=params, model="scenic.simulators.carla.model", mode2D=True)
        scene, n = scn.generate(maxIterations=5000)
        ego = scene.objects[0]
        line = f"[OK] {bench}/{filename}: {len(scene.objects)} objs, {n} iters, ego_h={math.degrees(ego.heading):.1f}"
        if len(scene.objects) > 1:
            adv = scene.objects[1]
            rel = (math.degrees(adv.heading - ego.heading) + 180) % 360 - 180
            line += f", adv_h={math.degrees(adv.heading):.1f}, rel={rel:+.1f}"
        print(line)
    except Exception as e:
        print(f"[FAIL] {bench}/{filename}: {type(e).__name__}: {e}")

    t_rd, t_orig = term_lines(path), term_lines(os.path.join(ORIG_ROOT, bench, filename))
    if t_rd != t_orig:
        print(f"  [TERM DIFF] orig={t_orig} rd={t_rd}")  # only OK if the diff is a bare `terminate`
                                                            # that lived inside the deleted EgoBehavior
```

**How to read it:**
- `[OK] ... N objs` → ego + adversary/prop/pedestrian all placed; requires
  satisfiable. `iters` in the tens/hundreds is fine (rejection sampling still
  works); a `[FAIL]` or iters pinned at `maxIterations` means something is
  unsatisfiable or mis-derived.
- `rel ≈ 0°` for a same-direction ahead/behind adversary; `rel ≈ ±180°` for
  oncoming; `rel ≈ ±90°` for a left/right crossing adversary — sanity-check
  this against the scenario's description. A wildly different `rel` than
  expected is the classic symptom of the yaw pitfall (§4) or a swapped
  ahead/behind sign during spawn-chain inversion (§3.3).
- `[TERM DIFF]` is only acceptable when the removed lines are a bare
  `terminate` that lived inside the deleted `EgoBehavior`'s `interrupt`
  clause — anything else is a real regression, go re-diff that file against
  the original's `terminate` lines by hand.

Run this over every converted file, not just the interesting-looking ones —
several genuine bugs (wrong requires dropped, the lazy-Distribution `+`
issue) only showed up because every file got smoke-tested individually.

## 8) Pre-flight checklist (per file)

- [ ] `EgoSpawnPt`/`yaw` pulled from `globalParameters`; ego pinned via `new
      OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)`.
- [ ] Ego Scenic behavior + its exclusively-used params removed; adversary
      params that referenced a removed ego param got an equivalent literal
      substitution.
- [ ] Dead sampling loops (`laneSecsWithXxx = [...]` + their `require
      len(...) > 0`) removed along with the `Uniform(...)` they fed.
- [ ] Adversary placement uses a VectorField or the corrected-heading
      `egoSpawnPt.heading`.
- [ ] `rg 'egoSpawnPt\.heading|(behind|ahead of|left of|right of) egoSpawnPt|facing egoSpawnPt'`
      → every hit's source assignment uses the corrected heading form.
- [ ] Every `egoInitLane.maneuvers` (or similar) filter includes
      `m.intersection is not None` plus the original's intersection predicate
      (§6) — not just the maneuver type.
- [ ] Adversary/pedestrian/prop behaviors, monitors, `require`s (**all** of
      them, including ones mentioning `egoSpawnPt`/`intersection`),
      `regionContainedIn`, blueprints untouched.
- [ ] `terminate when`/`terminate after` lines byte-identical to the
      original; termination audit (§7) shows only the sanctioned diff, if any.
- [ ] Smoke test: `[OK]`, plausible `rel`, no `[FAIL]`.

## 9) Bench pattern cheat-sheet

| Pattern | Inversion |
|---|---|
| Ego-only / passive | spawn + `laneAt`; no adversary |
| Ego-hazard (steer/depart/reverse) | just remove `EgoBehavior`; keep terminate verbatim (semantic shift accepted) |
| Same-lane lead/follow ahead | `following roadDirection` / `egoInitLane.orientation from egoSpawnPt for Range(...)` |
| ⚠ Same-lane via `ahead of`/`behind egoSpawnPt` | keep, but only with the corrected heading (§4) |
| Adjacent lane / cut-in | `laneSectionAt` + `_laneToLeft`/`_laneToRight` + `.centerline.project(egoSpawnPt.position)` + offset |
| ⚠ Adjacent lane with `facing egoSpawnPt.heading` | corrected heading (§4), or `facing adjSection.orientation` |
| Oncoming lane | project onto adjacent/oncoming lane, keep the backward `Range` offset |
| Intersection conflict | `egoInitLane.maneuvers`, filtered with `m.intersection is not None` + the original predicate (§6), → `egoManeuver.intersection` for the intersection var, → `conflictingManeuvers`/`reverseManeuvers` for the adversary; adv stays `in advInitLane.centerline`; keep TL monitors |
| Intersection ped/prop | ped on `endLane.centerline[0]` / prop on `connectingLane.centerline` from the derived maneuver |
| Lead cut-out / reveal chain | preserve the multi-hop chain (lead → revealed obstacle), anchored from `egoSpawnPt` via a road field |
| Highway on/off-ramp maneuver | filter `egoInitLane.maneuvers` for the ramp maneuver type directly, instead of scanning all `network.lanes` for one |
| Crossing pedestrian (perpendicular) | ⚠ `facing (egoSpawnPt.heading - 90 deg)` needs the corrected heading; anchor the crossing point via `roadDirection`/lane geometry, not a bare heading offset |

## 10) After conversion

- Do **not** regenerate the route pickle/index — they point at the originals
  and are exactly what makes the fixed `spawnPt` valid.
- If the task covers a whole bench directory, list every `.scenic` file under
  it and diff each one against its original before assuming it still needs
  conversion — some may already be done, and copying an already-converted
  file over itself is a no-op but wastes a smoke-test cycle.
