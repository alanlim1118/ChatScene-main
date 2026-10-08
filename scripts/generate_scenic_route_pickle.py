#!/usr/bin/env python3

import argparse
import fnmatch
import json
import math
import os
import os.path as osp
import pickle
import re
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, TYPE_CHECKING

# Allow running as `python scripts/...py` without installing the package.
_REPO_ROOT = osp.abspath(osp.join(osp.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

if TYPE_CHECKING:
    # Only for type checking; runtime imports are lazy to keep `--help` lightweight.
    import carla  # noqa: F401
    from safebench.util.scenic_utils import ScenicSimulator  # noqa: F401
    from safebench.agent.osc_ego_recorder import EgoVideoRecorder  # noqa: F401

from safebench.util import scenario_id_manifest
from safebench.util import scenario_dir as scenario_layout
import scenario_dir_route_gen


SCENARIO_RE = re.compile(r"scenario_(\d+)\.scenic$")
CARLA_MAP_RE = re.compile(r"^\s*param\s+carla_map\s*=\s*'([^']+)'\s*$", re.MULTILINE)
WEATHER_RE = re.compile(r"^\s*param\s+weather\s*=\s*'([^']+)'\s*$", re.MULTILINE)


def _parse_scenario_num_from_filename(path: str) -> Optional[int]:
    m = SCENARIO_RE.search(osp.basename(path))
    if not m:
        return None
    return int(m.group(1))


def _parse_required_param(text: str, regex: re.Pattern, name: str, scenic_file: str) -> str:
    m = regex.search(text)
    if not m:
        raise ValueError(f"Missing `{name}` in Scenic file: {scenic_file}")
    return m.group(1)


def _parse_optional_param(text: str, regex: re.Pattern) -> Optional[str]:
    """Like _parse_required_param, but returns None instead of raising when
    the param isn't a plain string literal (e.g. `param weather = Weather(...)`
    or `param weather = Uniform(*OPTIONS)`). Callers should simply omit the
    corresponding override key rather than passing None through to Scenic, so
    the file's own (non-literal) definition is used unmodified.
    """
    m = regex.search(text)
    return m.group(1) if m else None


def _bench_id_from_path(scenic_file: str, scenic_dir: Optional[str]) -> str:
    # Heuristic: for files like .../Self_Gemini_.../<BenchName>/scenario_001.scenic
    if scenic_dir:
        rel = osp.relpath(scenic_file, scenic_dir)
        parts = rel.split(os.sep)
        if len(parts) >= 2:
            return parts[0]
    return osp.basename(osp.dirname(scenic_file))


def _euclid_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _carla_to_scenic_xyz(loc) -> Tuple[float, float, float]:
    # Safebench later converts Scenic->CARLA by y := -y. So we must write Scenic coords here.
    return (float(loc.x), float(-loc.y), float(loc.z))


def _carla_to_scenic_xy(loc) -> Tuple[float, float]:
    return (float(loc.x), float(-loc.y))


def _downsample_by_dist_xyz(points_xyz: Sequence[Tuple[float, float, float]], min_dist: float) -> List[Tuple[float, float, float]]:
    if min_dist <= 0 or len(points_xyz) <= 1:
        return list(points_xyz)
    kept: List[Tuple[float, float, float]] = [points_xyz[0]]
    last_xy = (points_xyz[0][0], points_xyz[0][1])
    for p in points_xyz[1:]:
        xy = (p[0], p[1])
        if _euclid_xy(xy, last_xy) >= min_dist:
            kept.append(p)
            last_xy = xy
    if kept[-1] != points_xyz[-1]:
        kept.append(points_xyz[-1])
    return kept


@dataclass(frozen=True)
class RouteRecord:
    key: str
    bench_id: str
    scenario_id: int
    route_id: int
    scenic_file: str
    town: str
    weather: Optional[str]
    spawnPt: Dict[str, float]
    trajectory: List[Tuple[float, float, float]]
    waypoints: List[Tuple[float, float, float]]
    lanePts: List[Tuple[float, float]]
    # Directory the FPV/BEV recorder wrote to for the successful attempt (None if not recording).
    video_dir: Optional[str] = None


def _make_pickle_key(key_mode: str, bench_id: str, scenario_id: int, route_id: int) -> str:
    if key_mode == "bench_and_scenario":
        return f"scenario_id_{bench_id}__{scenario_id}_route_id_{route_id}"
    if key_mode == "numeric_only":
        return f"scenario_id_{scenario_id}_route_id_{route_id}"
    raise ValueError(f"Unknown --key-mode: {key_mode}")


def _run_scenic_and_record_ego(
    simulator,
    scene,
    *,
    max_steps: int,
    min_waypoint_dist: float,
    record_video: bool,
    recordings_dir: str,
    video_prefix: str,
    video_fps: int,
    video_width: int,
    video_height: int,
    bev_height: float,
    max_record_seconds: float = 0.0,
) -> Tuple[Dict[str, float], List[Tuple[float, float, float]], Optional[str]]:
    if not simulator.setSimulation(scene, max_steps=max_steps):
        raise RuntimeError("Failed to create Scenic CARLA simulation")

    sim = simulator.simulation
    world = sim.world

    # Optional simulation-time cap (seconds) for both recording and waypoint capture.
    max_record_steps = (
        int(round(max_record_seconds / float(sim.timestep)))
        if max_record_seconds and max_record_seconds > 0
        else 0
    )

    ego_obj = simulator._ego_object()
    ego_actor = getattr(ego_obj, "carlaActor", None)
    if ego_actor is None:
        raise RuntimeError("Could not resolve Scenic ego CARLA actor (ego_obj.carlaActor missing)")

    # Ensure sync mode + fixed delta (idempotent if already set externally)
    settings = world.get_settings()
    if not settings.synchronous_mode:
        settings.synchronous_mode = True
    if settings.fixed_delta_seconds is None:
        settings.fixed_delta_seconds = float(sim.timestep)
    world.apply_settings(settings)

    # Record initial pose as spawn point
    t0 = ego_actor.get_transform()
    spawn_xyz = _carla_to_scenic_xyz(t0.location)
    spawn_yaw = float(t0.rotation.yaw)
    spawnPt = {"x": spawn_xyz[0], "y": spawn_xyz[1], "z": spawn_xyz[2], "yaw": spawn_yaw}

    traj_xyz: List[Tuple[float, float, float]] = []

    from scenic.core.dynamics.actions import _EndSimulationAction, _EndScenarioAction  # imported lazily
    from scenic.core.simulators import TerminationType, SimulationResult  # imported lazily

    dynamicScenario = sim.scene.dynamicScenario
    terminationReason = None
    terminationType = None

    recorder = None
    if record_video:
        from safebench.agent.osc_ego_recorder import EgoVideoRecorder
        import carla as carla_mod

        recorder = EgoVideoRecorder(
            carla_mod,
            world,
            ego_actor,
            outdir=recordings_dir,
            prefix=video_prefix,
            fps=video_fps,
            width=video_width,
            height=video_height,
            bev_height=bev_height,
        )
        recorder.start()

    # Main tick loop: include ego behavior (unlike Safebench ScenicRunner which skips ego).
    try:
        while True:
            # Step Scenic scenario logic/monitors.
            terminationReason = dynamicScenario._step()
            terminationType = TerminationType.scenarioComplete
            sim.recordCurrentState()

            newReason = dynamicScenario._runMonitors()
            if newReason is not None:
                terminationReason = newReason
                terminationType = TerminationType.terminatedByMonitor

            if terminationReason is not None:
                break

            terminationReason = dynamicScenario._checkSimulationTerminationConditions()
            if terminationReason is not None:
                terminationType = TerminationType.simulationTerminationCondition
                break

            if max_steps and sim.currentTime >= max_steps:
                terminationReason = f"reached time limit ({max_steps} steps)"
                terminationType = TerminationType.timeLimit
                break

            allActions = {}
            for agent in sim.scheduleForAgents():
                if not getattr(agent, "behavior", None):
                    continue
                actions = agent.behavior._step()
                if isinstance(actions, (_EndSimulationAction, _EndScenarioAction)):
                    # _EndScenarioAction fires on a bare `terminate` (as opposed
                    # to `terminate simulation` -> _EndSimulationAction) inside a
                    # behavior. For these single-top-level-scenario files there
                    # is no parent scenario to fall back to, so treat it the
                    # same as ending the simulation - otherwise it isn't a
                    # list/tuple of per-agent actions and executeActions() below
                    # crashes with "object is not iterable".
                    terminationReason = str(actions)
                    terminationType = TerminationType.terminatedByBehavior
                    break
                if isinstance(actions, tuple) and len(actions) == 1 and isinstance(actions[0], (list, tuple)):
                    actions = tuple(actions[0])
                allActions[agent] = actions

            if terminationReason is not None:
                break

            sim.actionSequence.append(allActions)
            sim.executeActions(allActions)
            world.tick()
            sim.updateObjects()
            sim.currentTime += 1

            if recorder is not None:
                recorder.update_bev()

            tr = ego_actor.get_transform()
            traj_xyz.append(_carla_to_scenic_xyz(tr.location))

            if max_record_steps and sim.currentTime >= max_record_steps:
                terminationReason = f"reached record time limit ({max_record_seconds:.1f}s)"
                terminationType = TerminationType.timeLimit
                break

            sim.result = SimulationResult(
                sim.trajectory,
                sim.actionSequence,
                terminationType,
                terminationReason,
                sim.records,
            )
    finally:
        if recorder is not None:
            recorder.stop()

    traj_xyz = _downsample_by_dist_xyz(traj_xyz, min_waypoint_dist)
    return spawnPt, traj_xyz, (recorder.scene_dir if recorder is not None else None)


def _record_to_pickle_entry(record: RouteRecord) -> Dict:
    return {
        "town": record.town,
        "weather": record.weather,
        "spawnPt": record.spawnPt,
        "trajectory": record.trajectory,
        "waypoints": record.waypoints,
        "lanePts": record.lanePts,
    }


def _load_pickle(path: str) -> Dict:
    if not osp.exists(path):
        return {}
    with open(path, "rb") as f:
        obj = pickle.load(f)
    if not isinstance(obj, dict):
        raise TypeError(f"Expected dict in pickle {path}, got {type(obj)}")
    return obj


def _dump_pickle(path: str, data: Dict) -> None:
    os.makedirs(osp.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)


def _load_json(path: str) -> Dict:
    if not osp.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _dump_json(path: str, data: Dict) -> None:
    os.makedirs(osp.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)


def _iter_scenic_files(scenic_dir: str, glob_pat: str) -> List[str]:
    matches: List[str] = []
    for root, _, files in os.walk(scenic_dir):
        for fn in files:
            rel = osp.relpath(osp.join(root, fn), scenic_dir)
            if fnmatch.fnmatch(rel, glob_pat) or fnmatch.fnmatch(fn, glob_pat):
                if fn.endswith(".scenic"):
                    matches.append(osp.join(root, fn))
    return sorted(matches)


def generate_one(
    scenic_file: str,
    *,
    scenic_dir: Optional[str],
    port: int,
    tm_port: int,
    fixed_delta_seconds: float,
    max_steps: int,
    mode2d: bool,
    route_id: int,
    key_mode: str,
    bench_id: Optional[str],
    scenario_id: Optional[int],
    min_waypoint_dist: float,
    record_video: bool,
    recordings_dir: str,
    video_fps: int,
    video_width: int,
    video_height: int,
    bev_height: float,
    max_scene_attempts: int,
    warmup_ticks: int,
    carla_timeout: float = 60.0,
    max_record_seconds: float = 0.0,
) -> RouteRecord:
    try:
        from safebench.util.scenic_utils import ScenicSimulator
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            "Missing runtime dependency for Scenic route generation. "
            "This script requires the same env as running Safebench Scenic (notably `torch`, `scenic`, `carla`)."
        ) from e

    with open(scenic_file, "r", encoding="utf-8") as f:
        scenic_text = f.read()

    # Accepts both `param carla_map = 'Town05'` and the indirect
    # `Town = 'Town05'` / `param carla_map = Town` form.
    town = scenario_layout.parse_carla_map(scenic_text)
    if town is None:
        raise ValueError(f"Missing `param carla_map` in Scenic file: {scenic_file}")
    # `weather` is optional: many Scenic files define it via a Weather(...)/
    # WeatherConditions(...) constructor or Uniform(...) rather than a plain
    # string literal. When it isn't a literal, we don't override it below -
    # the file's own (non-literal) definition is used unmodified by Scenic.
    weather = _parse_optional_param(scenic_text, WEATHER_RE)

    if scenario_id is None:
        scenario_id = _parse_scenario_num_from_filename(scenic_file)
    if scenario_id is None:
        # Fallback for bench dirs whose .scenic files aren't scenario_NNN.scenic:
        # consult a pre-built scenario_id_manifest.json in the file's directory
        # (see scripts/build_scenario_id_manifest.py). Read-only here - this does
        # not assign new ids, only resolves ids someone already assigned.
        scenario_id = scenario_id_manifest.id_for_file(
            osp.dirname(scenic_file), osp.basename(scenic_file)
        )
    if scenario_id is None:
        raise ValueError(
            f"Could not infer --scenario-id from filename: {scenic_file}. "
            "Pass --scenario-id explicitly, or build a manifest first with "
            "scripts/build_scenario_id_manifest.py."
        )

    if bench_id is None:
        bench_id = _bench_id_from_path(scenic_file, scenic_dir)

    params = {
        "port": int(port),
        "traffic_manager_port": int(tm_port),
        "address": "127.0.0.1",
        "timestep": float(fixed_delta_seconds),
        "use2DMap": bool(mode2d),
        "render": 0,
        "town": town,
        "timeout": float(carla_timeout),
    }
    if weather is not None:
        params["weather"] = weather

    simulator = ScenicSimulator(scenic_file, params, mode2D=mode2d, fixed_delta_seconds=fixed_delta_seconds)
    original_name = osp.splitext(osp.basename(scenic_file))[0]
    if isinstance(scenario_id, int):
        video_prefix = f"{bench_id}__{original_name}__scenario_{scenario_id:03d}__route_{route_id}"
    else:
        video_prefix = f"{original_name}__route_{route_id}"

    if warmup_ticks > 0:
        # Optional stabilization before spawning (navmesh/streaming can lag right after map load).
        import carla as carla_mod

        world = simulator.simulator.client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = float(fixed_delta_seconds)
        world.apply_settings(settings)
        for _ in range(int(warmup_ticks)):
            world.tick()

    last_error: Optional[BaseException] = None
    spawnPt = None
    traj_xyz: Optional[List[Tuple[float, float, float]]] = None
    recorded_video_dir: Optional[str] = None

    for attempt in range(1, max_scene_attempts + 1):
        try:
            scene, _ = simulator.generateScene()
            spawnPt, traj_xyz, recorded_video_dir = _run_scenic_and_record_ego(
                simulator,
                scene,
                max_steps=max_steps,
                min_waypoint_dist=min_waypoint_dist,
                record_video=record_video,
                recordings_dir=recordings_dir,
                video_prefix=video_prefix,
                video_fps=video_fps,
                video_width=video_width,
                video_height=video_height,
                bev_height=bev_height,
                max_record_seconds=max_record_seconds,
            )
            break
        except Exception as e:
            # Scenic/CARLA spawn failures are common (e.g., invalid pedestrian spawn).
            # We resample scenes until setSimulation succeeds.
            last_error = e
            try:
                simulator.endSimulation()
            except Exception:
                pass
            if attempt >= max_scene_attempts:
                raise RuntimeError(
                    f"Failed to create a runnable Scenic simulation after {max_scene_attempts} attempts "
                    f"for {scenic_file}. Last error: {repr(last_error)}"
                ) from last_error
            continue
    else:
        raise RuntimeError(f"Unreachable: attempts loop exhausted for {scenic_file}")

    assert spawnPt is not None and traj_xyz is not None
    simulator.endSimulation()
    simulator.destroy()

    if len(traj_xyz) < 2:
        raise RuntimeError(f"Ego trajectory too short ({len(traj_xyz)}) for {scenic_file}")

    waypoints = traj_xyz
    # Use a coarse route for Safebench route construction.
    trajectory = [
        traj_xyz[0],
        traj_xyz[len(traj_xyz) // 2],
        traj_xyz[-1],
    ]

    # Leave lanePts empty for now (some route-driven Scenic templates use it).
    lanePts: List[Tuple[float, float]] = []

    key = _make_pickle_key(key_mode, bench_id, scenario_id, route_id)
    return RouteRecord(
        key=key,
        bench_id=bench_id,
        scenario_id=scenario_id,
        route_id=route_id,
        scenic_file=scenic_file,
        town=town,
        weather=weather,
        spawnPt=spawnPt,
        trajectory=trajectory,
        waypoints=waypoints,
        lanePts=lanePts,
        video_dir=recorded_video_dir,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenic-file", type=str, default=None)
    parser.add_argument("--scenic-dir", type=str, default=None)
    scenario_dir_route_gen.add_arguments(parser)
    parser.add_argument("--glob", type=str, default="**/*.scenic")

    parser.add_argument("--scenario-id", type=int, default=None)
    parser.add_argument("--bench-id", type=str, default=None)
    parser.add_argument("--route-id", type=int, default=0)
    parser.add_argument("--key-mode", type=str, default="bench_and_scenario", choices=["bench_and_scenario", "numeric_only"])

    parser.add_argument("--out-pickle", type=str, default="safebench/scenario/scenario_data/route_wenting/scenic_route.pickle")
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.1)
    parser.add_argument("--max-steps", type=int, default=10000)
    parser.add_argument(
        "--max-record-seconds",
        type=float,
        default=0.0,
        help="Stop the scenario (video + waypoint capture) after this many simulation seconds (0 = disabled).",
    )
    parser.add_argument("--min-waypoint-dist", type=float, default=1.0)
    parser.add_argument("--mode2d", action="store_true", default=True)
    parser.add_argument("--no-mode2d", action="store_false", dest="mode2d")

    parser.add_argument("--port", type=int, default=2002)
    parser.add_argument("--tm-port", type=int, default=8002)
    parser.add_argument(
        "--carla-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for CARLA RPC calls (notably client.load_world), e.g. town switches. "
        "Scenic's default (10s) is often too short for heavier towns.",
    )

    # Default: on with --scenario-dir, off otherwise.
    parser.add_argument("--record-video", action="store_true", default=None)
    parser.add_argument("--no-record-video", action="store_false", dest="record_video")
    parser.add_argument(
        "--recordings-dir",
        type=str,
        default="safebench/scenario/scenario_data/recordings",
        help="Directory to write MP4 recordings (FPV + BEV).",
    )
    parser.add_argument("--video-fps", type=int, default=15)
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--bev-height", type=float, default=50.0)

    parser.add_argument(
        "--max-scene-attempts",
        type=int,
        default=50,
        help="Max number of Scenic scene resampling attempts if CARLA actor spawning fails.",
    )
    parser.add_argument(
        "--warmup-ticks",
        type=int,
        default=0,
        help="Optional CARLA world ticks before sampling scenes (can improve spawn reliability).",
    )

    parser.add_argument("--resume", action="store_true", default=True)
    parser.add_argument("--no-resume", action="store_false", dest="resume")
    parser.add_argument("--fail-fast", action="store_true", default=False)
    parser.add_argument(
        "--validate-pickle",
        action="store_true",
        default=False,
        help="Validate existing --out-pickle schema and exit (no CARLA/Scenic execution).",
    )

    args = parser.parse_args()

    if args.validate_pickle:
        data = _load_pickle(args.out_pickle)
        required = {"trajectory", "town", "weather", "waypoints", "lanePts", "spawnPt"}
        for k, v in data.items():
            if not isinstance(v, dict):
                raise SystemExit(f"Bad entry type for key {k}: {type(v)}")
            missing = required.difference(v.keys())
            if missing:
                raise SystemExit(f"Entry {k} missing keys: {sorted(missing)}")
            sp = v["spawnPt"]
            if not isinstance(sp, dict) or any(x not in sp for x in ("x", "y", "z", "yaw")):
                raise SystemExit(f"Entry {k} has invalid spawnPt: {sp}")
        return

    if sum(x is not None for x in (args.scenic_file, args.scenic_dir, args.scenario_dir)) != 1:
        raise SystemExit("Provide exactly one of --scenic-file, --scenic-dir or --scenario-dir")

    if args.scenario_dir is not None:
        if args.record_video is None:
            args.record_video = True
        sys.exit(scenario_dir_route_gen.run(args, __file__, lambda scenic_file, **kw: generate_one(
            scenic_file,
            scenic_dir=None,
            port=args.port,
            tm_port=args.tm_port,
            fixed_delta_seconds=args.fixed_delta_seconds,
            max_steps=args.max_steps,
            mode2d=args.mode2d,
            route_id=args.route_id,
            key_mode=args.key_mode,
            min_waypoint_dist=args.min_waypoint_dist,
            video_fps=args.video_fps,
            video_width=args.video_width,
            video_height=args.video_height,
            bev_height=args.bev_height,
            max_scene_attempts=args.max_scene_attempts,
            warmup_ticks=args.warmup_ticks,
            carla_timeout=args.carla_timeout,
            max_record_seconds=args.max_record_seconds,
            **kw,
        )))
    args.record_video = bool(args.record_video)

    out_pickle = args.out_pickle
    out_index = osp.join(osp.dirname(out_pickle), "scenic_route_index.json")
    data = _load_pickle(out_pickle)
    index = _load_json(out_index)

    scenic_files: List[str]
    scenic_root: Optional[str] = None
    if args.scenic_file:
        scenic_files = [args.scenic_file]
        scenic_root = None
    else:
        scenic_root = args.scenic_dir
        scenic_files = _iter_scenic_files(args.scenic_dir, args.glob)
        if not scenic_files:
            raise SystemExit(f"No Scenic files found under {args.scenic_dir} with glob {args.glob}")

    # Bench dirs (one per distinct parent directory of a discovered .scenic
    # file) whose scenario_id manifest we've already built/loaded this run -
    # avoids rebuilding it once per file when a directory has many files.
    built_manifest_dirs: Dict[str, Dict] = {}

    errors: List[Dict] = []
    for sf in scenic_files:
        try:
            inferred_scenario = _parse_scenario_num_from_filename(sf)
            if inferred_scenario is None and args.scenario_id is None:
                # Bench dir with non-scenario_NNN.scenic naming: resolve via
                # its scenario_id manifest, so both --scenic-dir batches and
                # --scenic-file single runs skip already-completed scenarios
                # via --resume the same way generate_one() resolves ids.
                bench_dir = osp.dirname(sf)
                if scenic_root is not None:
                    # Directory-batch mode may build/extend the manifest
                    # (idempotent, preserves any existing assignments) so this
                    # run - and later eval-time lookups - agree on ids.
                    if bench_dir not in built_manifest_dirs:
                        built_manifest_dirs[bench_dir] = scenario_id_manifest.load_or_build(bench_dir)
                inferred_scenario = scenario_id_manifest.id_for_file(bench_dir, osp.basename(sf))
            scenario_id = args.scenario_id if args.scenario_id is not None else inferred_scenario
            bench_id = args.bench_id if args.bench_id is not None else _bench_id_from_path(sf, scenic_root)
            key = _make_pickle_key(args.key_mode, bench_id, scenario_id or -1, args.route_id)

            if args.resume and key in data:
                continue

            rec = generate_one(
                sf,
                scenic_dir=scenic_root,
                port=args.port,
                tm_port=args.tm_port,
                fixed_delta_seconds=args.fixed_delta_seconds,
                max_steps=args.max_steps,
                mode2d=args.mode2d,
                route_id=args.route_id,
                key_mode=args.key_mode,
                bench_id=bench_id,
                scenario_id=scenario_id,
                min_waypoint_dist=args.min_waypoint_dist,
                record_video=args.record_video,
                recordings_dir=args.recordings_dir,
                video_fps=args.video_fps,
                video_width=args.video_width,
                video_height=args.video_height,
                bev_height=args.bev_height,
                max_scene_attempts=args.max_scene_attempts,
                warmup_ticks=args.warmup_ticks,
                carla_timeout=args.carla_timeout,
                max_record_seconds=args.max_record_seconds,
            )
            data[rec.key] = _record_to_pickle_entry(rec)
            idx_key = f"{rec.bench_id}:{rec.scenario_id}:{rec.route_id}"
            index[idx_key] = {
                "pickle_key": rec.key,
                "scenic_file": rec.scenic_file,
                "town": rec.town,
                "weather": rec.weather,
            }
            _dump_pickle(out_pickle, data)
            _dump_json(out_index, index)
        except Exception as e:
            err = {"scenic_file": sf, "error": repr(e)}
            errors.append(err)
            if args.fail_fast:
                raise

    if errors:
        err_path = osp.join(osp.dirname(out_pickle), "scenic_route_errors.json")
        _dump_json(err_path, {"errors": errors})


if __name__ == "__main__":
    main()
