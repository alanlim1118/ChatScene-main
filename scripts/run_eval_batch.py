#!/usr/bin/env python3
"""
Batch wrapper around scripts/run_eval.py.

Auto-discovers scenario_*.scenic files under scenic_dir/<bench_id> from the
scenario YAML, then runs each scenario via subprocess. Supports --mode eval
(evaluate entire bench once using saved scenario_N.json) and --mode
train_scenario (OPT optimization across the bench).

Bench dirs whose .scenic files aren't named scenario_NNN.scenic fall back to
a scenario_id_manifest.json in the bench dir for discovery (see
scripts/build_scenario_id_manifest.py).

All unrecognized flags are forwarded to run_eval.py.

Self-contained scenario directories (no YAML paths, ids or manifest):

  python scripts/run_eval_batch.py --scenario_dir safebench/scenario/scenic_data/NL2Scenic \
      --mode train_scenario --test_policy ppo --agent_cfg adv_scenic_ppo.yaml ...

runs every <dataset>/<id>/ that has a route.pickle (see
safebench/util/scenario_dir.py); --scenarios <id> ... restricts the set.

Notes:
  - Use --test_policy ppo with adv_scenic.yaml (not the default sac).
  - --average matches OPT_<name>_ROUTE-<id>_results.pkl (use --route_id 0
    unless your YAML lists multiple routes).
"""

import argparse
import os
import os.path as osp
import re
import subprocess
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

_REPO_ROOT = osp.abspath(osp.join(osp.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from safebench.util import scenario_id_manifest
from safebench.util import scenario_dir as scenario_layout

SCENARIO_RE = re.compile(r"^scenario_(\d+)\.scenic$")
RUN_EVAL_SCRIPT = osp.join(_REPO_ROOT, "scripts", "run_eval.py")
AVERAGE_EVAL_SCRIPT = osp.join(_REPO_ROOT, "scripts", "average_eval_results.py")


def load_config(path: str) -> Dict[str, Any]:
    """Load a YAML config file."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _get_forwarded_value(
    forward_args: Sequence[str], flag: str, default: Optional[str] = None
) -> Optional[str]:
    """Return the value following a CLI flag in forwarded args."""
    for i, arg in enumerate(forward_args):
        if arg == flag:
            if i + 1 < len(forward_args):
                return forward_args[i + 1]
            return default
        if arg.startswith(flag + "="):
            return arg.split("=", 1)[1]
    return default


def _parse_scenario_range(scenario_range: str) -> List[int]:
    """Parse an inclusive range string like '1-9'."""
    if "-" not in scenario_range:
        raise ValueError(
            f"Invalid --scenario_range '{scenario_range}'. Expected format: START-END"
        )
    start_str, end_str = scenario_range.split("-", 1)
    start, end = int(start_str), int(end_str)
    if start > end:
        raise ValueError(
            f"Invalid --scenario_range '{scenario_range}': start must be <= end"
        )
    return list(range(start, end + 1))


def discover_scenario_ids(root_dir: str, scenario_cfg: str) -> List[int]:
    """Discover scenario IDs from scenic_dir/<bench_id>/scenario_*.scenic."""
    scenario_config_path = osp.join(
        root_dir, "safebench/scenario/config", scenario_cfg
    )
    if not osp.isfile(scenario_config_path):
        raise FileNotFoundError(f"Scenario config not found: {scenario_config_path}")

    scenario_config = load_config(scenario_config_path)
    scenic_dir = osp.join(root_dir, scenario_config["scenic_dir"])
    bench_id = scenario_config.get("bench_id")

    if bench_id:
        scan_dir = osp.join(scenic_dir, str(bench_id))
        if not osp.isdir(scan_dir):
            raise FileNotFoundError(f"Bench scenic directory not found: {scan_dir}")
        ids = []
        for filename in os.listdir(scan_dir):
            match = SCENARIO_RE.match(filename)
            if match:
                ids.append(int(match.group(1)))
        if not ids:
            # Bench dirs with non-scenario_NNN.scenic filenames: fall back to
            # a pre-built manifest (see scripts/build_scenario_id_manifest.py).
            manifest = scenario_id_manifest.load_manifest(scan_dir)
            if manifest:
                ids = [int(sid) for sid in manifest["id_to_file"]]
        if not ids:
            raise FileNotFoundError(
                f"No scenario_*.scenic files found in: {scan_dir} "
                "(and no scenario_id_manifest.json to fall back to)"
            )
        return sorted(set(ids))

    if not osp.isdir(scenic_dir):
        raise FileNotFoundError(f"Scenic directory not found: {scenic_dir}")

    ids = []
    for entry in sorted(os.listdir(scenic_dir)):
        scenario_subdir = osp.join(scenic_dir, entry)
        if not osp.isdir(scenario_subdir) or not entry.startswith("scenario_"):
            continue
        for filename in os.listdir(scenario_subdir):
            if filename.endswith(".scenic"):
                match = SCENARIO_RE.match(filename)
                if match:
                    ids.append(int(match.group(1)))
                else:
                    suffix = entry.replace("scenario_", "")
                    try:
                        ids.append(int(suffix))
                    except ValueError:
                        continue
                break

    if not ids:
        raise FileNotFoundError(
            f"No scenic scenarios found under legacy layout in: {scenic_dir}"
        )
    return sorted(set(ids))


def resolve_scenario_ids(
    discovered: Sequence[int],
    explicit_ids: Optional[Sequence[int]],
    scenario_range: Optional[str],
) -> List[int]:
    """Resolve the final scenario ID list from discovery and overrides."""
    discovered_set = set(discovered)

    if explicit_ids or scenario_range:
        selected = set()
        if explicit_ids:
            selected.update(explicit_ids)
        if scenario_range:
            selected.update(_parse_scenario_range(scenario_range))

        missing = sorted(selected - discovered_set)
        if missing:
            print(
                "Warning: requested scenario IDs not found on disk and will be "
                f"skipped: {missing}"
            )
        return sorted(selected & discovered_set)

    return list(discovered)


def build_bench_output_dir(
    root_dir: str,
    agent_cfg: str,
    scenario_cfg: str,
    test_policy: str,
    test_epoch: Optional[int],
    mode: str,
) -> str:
    """Compute the bench-level log directory used by run_eval.py."""
    scenario_config_path = osp.join(
        root_dir, "safebench/scenario/config", scenario_cfg
    )
    scenario_config = load_config(scenario_config_path)

    output_dir = osp.join(
        "log",
        "adv_train",
        mode,
        test_policy,
        f"{agent_cfg.split('.')[0]}_epoch{test_epoch}",
        scenario_cfg.split(".")[0],
    )
    bench_id = scenario_config.get("bench_id")
    if bench_id:
        output_dir = osp.join(output_dir, str(bench_id))
    return osp.join(root_dir, output_dir)


def build_subprocess_cmd(
    forward_args: Sequence[str], scenario_id: int
) -> List[str]:
    """Build the subprocess command for one scenario run."""
    cmd = [sys.executable, RUN_EVAL_SCRIPT]
    cmd.extend(forward_args)
    cmd.extend(["--scenario_id", str(scenario_id)])
    return cmd


def run_average(
    bench_output_dir: str, average_output: Optional[str]
) -> int:
    """Invoke average_eval_results.py on the bench output directory."""
    cmd = [sys.executable, AVERAGE_EVAL_SCRIPT, bench_output_dir]
    if average_output:
        cmd.extend(["-o", average_output])
    print(f"\nRunning average: {' '.join(cmd)}")
    return subprocess.run(cmd, cwd=_REPO_ROOT).returncode


def main_scenario_dirs(args, forward_args: List[str], run_script: str = RUN_EVAL_SCRIPT) -> int:
    """Batch over self-contained <dataset>/<id>/ scenario directories.

    Also used by run_eval_v2_batch.py with run_script=run_eval_v2.py.
    """
    mode = _get_forwarded_value(forward_args, "--mode") or _get_forwarded_value(forward_args, "-m", "eval")
    scenario_cfg = _get_forwarded_value(forward_args, "--scenario_cfg", "eval_scenic_scenario_dir.yaml")
    agent_cfg = _get_forwarded_value(forward_args, "--agent_cfg", "adv_scenic.yaml")
    test_policy = _get_forwarded_value(forward_args, "--test_policy", "sac")
    test_epoch_raw = _get_forwarded_value(forward_args, "--test_epoch")
    test_epoch = int(test_epoch_raw) if test_epoch_raw is not None else None

    try:
        scenario_dirs = scenario_layout.discover(args.scenario_dir)
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    if args.scenarios:
        wanted = set(args.scenarios)
        found = {scenario_layout.scenario_id(d) for d in scenario_dirs}
        if wanted - found:
            print(f"Warning: requested scenarios not found and will be skipped: {sorted(wanted - found)}")
        scenario_dirs = [d for d in scenario_dirs if scenario_layout.scenario_id(d) in wanted]

    runnable: List[str] = []
    skipped: List[Tuple[str, str]] = []
    for d in scenario_dirs:
        sid = scenario_layout.scenario_id(d)
        if not scenario_layout.has_route(d):
            skipped.append((sid, "no route.pickle"))
        elif mode in ("eval", "train_agent") and not osp.isfile(scenario_layout.opt_params_path(d)):
            skipped.append((sid, "no opt_params.json (run --mode train_scenario first)"))
        else:
            runnable.append(d)

    print(f"Mode: {mode}")
    print(f"Found {len(scenario_dirs)} scenario dir(s) under {args.scenario_dir}; {len(runnable)} runnable")
    if skipped:
        print(f"Skipping {len(skipped)}:")
        for sid, why in skipped:
            print(f"  {sid}: {why}")
    if not runnable:
        print("Error: nothing to run.", file=sys.stderr)
        return 1

    succeeded: List[str] = []
    failed: List[Tuple[str, int]] = []
    for index, d in enumerate(runnable, start=1):
        sid = scenario_layout.scenario_id(d)
        cmd = [sys.executable, run_script, *forward_args, "--scenario_dir", d]
        print(f"\n[{index}/{len(runnable)}] {sid}")
        print(" ".join(cmd))
        if args.dry_run:
            continue
        rc = subprocess.run(cmd, cwd=_REPO_ROOT).returncode
        if rc == 0:
            succeeded.append(sid)
        else:
            failed.append((sid, rc))
            print(f"Scenario {sid} failed with exit code {rc}.", file=sys.stderr)
            if not args.continue_on_error:
                print("Stopping because --no-continue_on_error was set.")
                break

    print("\n=== Batch summary ===")
    if args.dry_run:
        print(f"Dry run only. {len(runnable)} command(s) prepared, 0 executed.")
        return 0
    print(f"Succeeded ({len(succeeded)}): {succeeded}")
    print(f"Failed ({len(failed)}): {failed}")

    if args.average:
        bench_output_dir = osp.join(
            _REPO_ROOT, "log", "adv_train", mode, test_policy,
            f"{agent_cfg.split('.')[0]}_epoch{test_epoch}", scenario_cfg.split(".")[0],
            scenario_layout.dataset_name(runnable[0]),
        )
        if succeeded:
            avg_rc = run_average(bench_output_dir, args.average_output)
            if avg_rc != 0:
                return avg_rc
    return 1 if failed else 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Batch-run scripts/run_eval.py over multiple scenario IDs. "
            "Use --mode eval to evaluate the entire bench once (reads "
            "scenario_N.json), or --mode train_scenario to optimize OPT "
            "params. All unrecognized flags are forwarded to run_eval.py."
        )
    )
    parser.add_argument(
        "--scenario_ids",
        nargs="+",
        type=int,
        default=None,
        help="Explicit scenario IDs to run",
    )
    parser.add_argument(
        "--scenario_range",
        type=str,
        default=None,
        help="Inclusive scenario ID range, e.g. 1-9",
    )
    parser.add_argument(
        "--continue_on_error",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Continue with remaining scenarios if one subprocess fails",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="Print discovered IDs and subprocess commands without running",
    )
    parser.add_argument(
        "--average",
        action="store_true",
        help="After all runs, average pickle results under the bench output dir",
    )
    parser.add_argument(
        "--average_output",
        type=str,
        default=None,
        help="Output path for average_eval_results.json",
    )

    parser.add_argument(
        "--scenario_dir",
        type=str,
        default=None,
        help="Dataset directory of self-contained <id>/ scenario dirs (replaces YAML-based discovery)",
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=None,
        help="With --scenario_dir: only these scenario ids (directory names)",
    )

    args, forward_args = parser.parse_known_args()

    if args.scenario_dir is not None:
        return main_scenario_dirs(args, forward_args)

    scenario_cfg = _get_forwarded_value(forward_args, "--scenario_cfg")
    if scenario_cfg is None:
        scenario_cfg = _get_forwarded_value(forward_args, "-scenario_cfg")
    if scenario_cfg is None:
        print(
            "Error: --scenario_cfg is required (for discovery and subprocess runs).",
            file=sys.stderr,
        )
        return 1

    agent_cfg = _get_forwarded_value(forward_args, "--agent_cfg", "adv_scenic.yaml")
    mode = _get_forwarded_value(forward_args, "--mode")
    if mode is None:
        mode = _get_forwarded_value(forward_args, "-m", "eval")
    test_policy = _get_forwarded_value(forward_args, "--test_policy", "sac")
    test_epoch_raw = _get_forwarded_value(forward_args, "--test_epoch")
    test_epoch = int(test_epoch_raw) if test_epoch_raw is not None else None

    try:
        discovered = discover_scenario_ids(_REPO_ROOT, scenario_cfg)
        scenario_ids = resolve_scenario_ids(
            discovered, args.scenario_ids, args.scenario_range
        )
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if not scenario_ids:
        print("Error: no scenario IDs to run.", file=sys.stderr)
        return 1

    print(f"Mode: {mode}")
    print(f"Discovered {len(discovered)} scenario(s): {discovered}")
    print(f"Will run {len(scenario_ids)} scenario(s): {scenario_ids}")

    succeeded: List[int] = []
    failed: List[Tuple[int, int]] = []

    total = len(scenario_ids)
    for index, scenario_id in enumerate(scenario_ids, start=1):
        cmd = build_subprocess_cmd(forward_args, scenario_id)
        print(f"\n[{index}/{total}] scenario_id={scenario_id}")
        print(" ".join(cmd))

        if args.dry_run:
            continue

        result = subprocess.run(cmd, cwd=_REPO_ROOT)
        if result.returncode == 0:
            succeeded.append(scenario_id)
        else:
            failed.append((scenario_id, result.returncode))
            print(
                f"Scenario {scenario_id} failed with exit code {result.returncode}.",
                file=sys.stderr,
            )
            if not args.continue_on_error:
                print("Stopping because --no-continue_on_error was set.")
                break

    print("\n=== Batch summary ===")
    if args.dry_run:
        print(f"Dry run only. {total} command(s) prepared, 0 executed.")
    else:
        print(f"Succeeded ({len(succeeded)}): {succeeded}")
        if failed:
            print(f"Failed ({len(failed)}):")
            for scenario_id, exit_code in failed:
                print(f"  scenario_id={scenario_id}, exit_code={exit_code}")
        else:
            print("Failed (0): []")

    if args.average and not args.dry_run:
        if failed and not succeeded:
            print(
                "Skipping --average because every scenario run failed.",
                file=sys.stderr,
            )
            return 1

        bench_output_dir = build_bench_output_dir(
            _REPO_ROOT, agent_cfg, scenario_cfg, test_policy, test_epoch, mode
        )
        avg_rc = run_average(bench_output_dir, args.average_output)
        if avg_rc != 0:
            print(
                f"average_eval_results.py failed with exit code {avg_rc}.",
                file=sys.stderr,
            )
            return avg_rc

    if failed and not args.dry_run:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
