"""--scenario-dir mode shared by generate_scenic_route_pickle.py (Scenic 3) and
generate_scenic_route_pickle_v2.py (Scenic 2).

Each scenario is a self-contained <dataset>/<id>/ directory (see
safebench/util/scenario_dir.py); this writes <id>/route.pickle and
<id>/video/<id>_{fpv,bev}.mp4 next to <id>/<id>.scenic, or <id>/route_error.txt
on failure. The Scenic-version-specific part is the `generate` callable each
script passes in:

    generate(scenic_file, *, bench_id, scenario_id, record_video, recordings_dir) -> record

where `record` has town / weather / spawnPt / trajectory / waypoints / lanePts
and video_dir (the recorder's directory for the successful attempt, or None).
"""

import argparse
import os
import os.path as osp
import shutil
import subprocess
import sys
import tempfile
import traceback
from datetime import datetime
from typing import Callable, Dict, List, Sequence

_REPO_ROOT = osp.abspath(osp.join(osp.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from safebench.util import scenario_dir as scenario_layout


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--scenario-dir",
        type=str,
        default=None,
        help="Per-scenario layout: a <id>/ directory holding <id>.scenic, or a dataset directory of them. "
        "Writes <id>/route.pickle and <id>/video/ next to the Scenic file (no --out-pickle/index/manifest).",
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=None,
        help="With --scenario-dir on a dataset directory: only these scenario ids (directory names).",
    )
    parser.add_argument(
        "--in-process",
        action="store_true",
        default=False,
        help="With --scenario-dir: run all scenarios in this process instead of one subprocess each.",
    )


def _replace_flag_value(argv: Sequence[str], flag: str, value: str) -> List[str]:
    out: List[str] = []
    skip = False
    for a in argv:
        if skip:
            skip = False
            continue
        if a == flag:
            out.extend([flag, value])
            skip = True
        elif a.startswith(flag + "="):
            out.append(f"{flag}={value}")
        else:
            out.append(a)
    return out


def _strip_multi_flag(argv: Sequence[str], flag: str) -> List[str]:
    """Remove `flag` and the values following it (up to the next --option)."""
    out: List[str] = []
    dropping = False
    for a in argv:
        if a == flag:
            dropping = True
            continue
        if dropping and not a.startswith("-"):
            continue
        dropping = False
        out.append(a)
    return out


def _collect_videos(staged_scene_dir: str, scenario_dir: str, sid: str) -> List[str]:
    """Move the recorder's FPV/BEV mp4s to <scenario_dir>/video/<sid>_{fpv,bev}.mp4."""
    dest_dir = scenario_layout.video_dir(scenario_dir)
    os.makedirs(dest_dir, exist_ok=True)
    moved = []
    for fn in os.listdir(staged_scene_dir):
        for view in ("FPV", "BEV"):
            if f"__{view}__" in fn and fn.endswith(".mp4"):
                dest = osp.join(dest_dir, f"{sid}_{view.lower()}.mp4")
                os.replace(osp.join(staged_scene_dir, fn), dest)
                moved.append(dest)
    return moved


def generate_scenario_dir(scenario_dir: str, args: argparse.Namespace, generate: Callable) -> str:
    """Generate <scenario_dir>/route.pickle (+ videos). Returns 'ok', 'skipped' or 'failed'."""
    sid = scenario_layout.scenario_id(scenario_dir)
    scenic_file = scenario_layout.source_scenic_file(scenario_dir)

    if args.resume and scenario_layout.has_route(scenario_dir):
        print(f"[skip] {sid}: route.pickle already exists (use --no-resume to regenerate)")
        return "skipped"
    if scenario_layout.is_empty_scenic(scenic_file):
        print(f"[skip] {sid}: {osp.basename(scenic_file)} is empty")
        return "skipped"

    staging = None
    if args.record_video:
        os.makedirs(scenario_layout.video_dir(scenario_dir), exist_ok=True)
        # Every scene attempt that gets as far as recording leaves a directory
        # here; only the successful one is kept.
        staging = tempfile.mkdtemp(prefix=".staging_", dir=scenario_layout.video_dir(scenario_dir))

    print(f"[route] {sid}: {scenic_file}", flush=True)
    try:
        rec = generate(
            scenic_file,
            bench_id=scenario_layout.dataset_name(scenario_dir),
            scenario_id=sid,
            record_video=bool(args.record_video),
            recordings_dir=staging or args.recordings_dir,
        )
        videos = _collect_videos(rec.video_dir, scenario_dir, sid) if rec.video_dir else []
    except Exception:
        tb = traceback.format_exc()
        with open(scenario_layout.route_error_path(scenario_dir), "w", encoding="utf-8") as f:
            f.write(f"{datetime.now().isoformat(timespec='seconds')}  {scenic_file}\n\n{tb}")
        print(f"[fail] {sid}: see {scenario_layout.route_error_path(scenario_dir)}\n{tb}", flush=True)
        return "failed"
    finally:
        if staging is not None:
            shutil.rmtree(staging, ignore_errors=True)
            try:
                os.rmdir(scenario_layout.video_dir(scenario_dir))  # only succeeds if nothing was recorded
            except OSError:
                pass

    entry = {k: getattr(rec, k) for k in scenario_layout.ROUTE_KEYS}
    entry.update({
        "scenario_id": sid,
        "scenic_file": osp.basename(scenic_file),
        "generated_at": datetime.now().isoformat(timespec="seconds"),
    })
    path = scenario_layout.save_route(scenario_dir, entry)
    if osp.exists(scenario_layout.route_error_path(scenario_dir)):
        os.remove(scenario_layout.route_error_path(scenario_dir))
    print(f"[ok] {sid}: {path}" + "".join(f"\n      {v}" for v in videos), flush=True)
    return "ok"


def run(args: argparse.Namespace, script_path: str, generate: Callable) -> int:
    """Entry point for --scenario-dir. Returns a process exit code."""
    scenario_dirs = scenario_layout.discover(args.scenario_dir)
    if args.scenarios:
        wanted = set(args.scenarios)
        missing = sorted(wanted - {scenario_layout.scenario_id(d) for d in scenario_dirs})
        if missing:
            print(f"Warning: requested scenarios not found under {args.scenario_dir}: {missing}")
        scenario_dirs = [d for d in scenario_dirs if scenario_layout.scenario_id(d) in wanted]
    if not scenario_dirs:
        raise SystemExit(
            f"No scenario directories under {args.scenario_dir} "
            "(expected <dir>/<id>/<id>.scenic, or a single <id>/ directory)"
        )

    if len(scenario_dirs) == 1 or args.in_process:
        statuses = [generate_scenario_dir(d, args, generate) for d in scenario_dirs]
        return 1 if "failed" in statuses else 0

    # Scenic keeps process-global state that isn't fully torn down between
    # files, so in one process only the first file reliably succeeds (see
    # scripts/generate_route_batch.sh). Run each scenario in a fresh interpreter.
    results: Dict[str, List[str]] = {"ok": [], "skipped": [], "failed": []}
    base_argv = sys.argv[1:]
    for i, d in enumerate(scenario_dirs, start=1):
        sid = scenario_layout.scenario_id(d)
        if args.resume and scenario_layout.has_route(d):
            results["skipped"].append(sid)
            continue
        if scenario_layout.is_empty_scenic(scenario_layout.source_scenic_file(d)):
            print(f"[skip] {sid}: scenic file is empty")
            results["skipped"].append(sid)
            continue
        print(f"\n=== [{i}/{len(scenario_dirs)}] {sid} ===", flush=True)
        # The child gets exactly one scenario directory and no --scenarios filter.
        argv = _strip_multi_flag(base_argv, "--scenarios") if args.scenarios else list(base_argv)
        argv = _replace_flag_value(argv, "--scenario-dir", d)
        rc = subprocess.run([sys.executable, osp.abspath(script_path)] + argv, cwd=_REPO_ROOT).returncode
        results["ok" if rc == 0 else "failed"].append(sid)

    print("\n=== Route generation summary ===")
    for k in ("ok", "skipped", "failed"):
        print(f"{k} ({len(results[k])}): {results[k]}")
    if results["failed"]:
        print("Failure details are in <scenario>/route_error.txt")
    return 1 if results["failed"] else 0
