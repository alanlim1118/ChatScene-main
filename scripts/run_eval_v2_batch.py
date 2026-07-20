#!/usr/bin/env python3
"""
Batch wrapper around scripts/run_eval_v2.py.

Auto-discovers scenario_*.scenic files in a directory given as the first
positional argument (or --scenic_dir), then runs each scenario via
subprocess.  Supports --mode eval (evaluate using saved scenario_N.json)
and --mode train_scenario (OPT optimisation across the bench).

All unrecognised flags are forwarded to run_eval_v2.py.

Notes:
  - Use --test_policy ppo with adv_scenic.yaml (not the default sac).
  - --average only matches OPT_scenario_*_ROUTE-0_results.pkl (use --route_id 0).
  - run_eval_v2.py only supports modes: train_scenario, eval.
  - The --scenario_cfg YAML must have scenic_dir+bench_id that resolves to
    the same directory you pass as --scenic_dir / the positional argument.
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

SCENARIO_RE = re.compile(r"^scenario_(\d+)\.scenic$")
RUN_EVAL_V2_SCRIPT = osp.join(_REPO_ROOT, "scripts", "run_eval_v2.py")
AVERAGE_EVAL_SCRIPT = osp.join(_REPO_ROOT, "scripts", "average_eval_results.py")


def load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _get_forwarded_value(
    forward_args: Sequence[str], flag: str, default: Optional[str] = None
) -> Optional[str]:
    for i, arg in enumerate(forward_args):
        if arg == flag:
            if i + 1 < len(forward_args):
                return forward_args[i + 1]
            return default
        if arg.startswith(flag + "="):
            return arg.split("=", 1)[1]
    return default


def _parse_scenario_range(scenario_range: str) -> List[int]:
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


def discover_scenario_ids(scenic_dir: str) -> List[int]:
    """Discover scenario IDs from scenario_*.scenic files in scenic_dir."""
    if not osp.isdir(scenic_dir):
        raise FileNotFoundError(f"Scenic directory not found: {scenic_dir}")

    ids = []
    for filename in os.listdir(scenic_dir):
        match = SCENARIO_RE.match(filename)
        if match:
            ids.append(int(match.group(1)))

    if not ids:
        raise FileNotFoundError(
            f"No scenario_*.scenic files found in: {scenic_dir}"
        )
    return sorted(set(ids))


def resolve_scenario_ids(
    discovered: Sequence[int],
    explicit_ids: Optional[Sequence[int]],
    scenario_range: Optional[str],
) -> List[int]:
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
    agent_cfg: str,
    scenario_cfg: str,
    test_policy: str,
    test_epoch: Optional[int],
    mode: str,
) -> str:
    """Compute the bench-level log directory used by run_eval_v2.py."""
    scenario_config_path = osp.join(
        _REPO_ROOT, "safebench/scenario/config", scenario_cfg
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
    return osp.join(_REPO_ROOT, output_dir)


def build_subprocess_cmd(
    forward_args: Sequence[str], scenario_id: int, bench_id: Optional[str]
) -> List[str]:
    cmd = [sys.executable, RUN_EVAL_V2_SCRIPT]
    cmd.extend(forward_args)
    cmd.extend(["--scenario_id", str(scenario_id)])
    if bench_id is not None:
        cmd.extend(["--bench_id", bench_id])
    return cmd


def run_average(bench_output_dir: str, average_output: Optional[str]) -> int:
    cmd = [sys.executable, AVERAGE_EVAL_SCRIPT, bench_output_dir]
    if average_output:
        cmd.extend(["-o", average_output])
    print(f"\nRunning average: {' '.join(cmd)}")
    return subprocess.run(cmd, cwd=_REPO_ROOT).returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Batch-run scripts/run_eval_v2.py over multiple scenario IDs. "
            "Discovers scenario_*.scenic files in the directory given as the "
            "first positional argument (or --scenic_dir). "
            "All unrecognised flags are forwarded to run_eval_v2.py."
        )
    )
    parser.add_argument(
        "scenic_dir",
        nargs="?",
        default=None,
        help="Directory containing scenario_*.scenic files",
    )
    parser.add_argument(
        "--scenic_dir",
        dest="scenic_dir_flag",
        default=None,
        help="Directory containing scenario_*.scenic files (alternative to positional arg)",
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

    args, forward_args = parser.parse_known_args()

    scenic_dir = args.scenic_dir_flag or args.scenic_dir
    if scenic_dir is None:
        print(
            "Error: a scenic directory is required (positional arg or --scenic_dir).",
            file=sys.stderr,
        )
        return 1

    if not osp.isabs(scenic_dir):
        scenic_dir = osp.abspath(scenic_dir)

    scenario_cfg = _get_forwarded_value(forward_args, "--scenario_cfg")
    if scenario_cfg is None:
        scenario_cfg = _get_forwarded_value(forward_args, "-scenario_cfg", "eval_scenic_v2.yaml")

    agent_cfg = _get_forwarded_value(forward_args, "--agent_cfg", "adv_scenic.yaml")
    mode = _get_forwarded_value(forward_args, "--mode")
    if mode is None:
        mode = _get_forwarded_value(forward_args, "-m", "eval")
    if mode not in ("train_scenario", "eval"):
        print(
            f"Warning: run_eval_v2.py only supports modes 'train_scenario' and 'eval', got '{mode}'.",
            file=sys.stderr,
        )
    test_policy = _get_forwarded_value(forward_args, "--test_policy", "sac")
    test_epoch_raw = _get_forwarded_value(forward_args, "--test_epoch")
    test_epoch = int(test_epoch_raw) if test_epoch_raw is not None else None

    try:
        discovered = discover_scenario_ids(scenic_dir)
        scenario_ids = resolve_scenario_ids(
            discovered, args.scenario_ids, args.scenario_range
        )
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if not scenario_ids:
        print("Error: no scenario IDs to run.", file=sys.stderr)
        return 1

    bench_id = osp.basename(scenic_dir)
    print(f"Scenic directory: {scenic_dir}")
    print(f"Bench ID (derived): {bench_id}")
    print(f"Mode: {mode}")
    print(f"Discovered {len(discovered)} scenario(s): {discovered}")
    print(f"Will run {len(scenario_ids)} scenario(s): {scenario_ids}")

    succeeded: List[int] = []
    failed: List[Tuple[int, int]] = []

    total = len(scenario_ids)
    for index, scenario_id in enumerate(scenario_ids, start=1):
        cmd = build_subprocess_cmd(forward_args, scenario_id, bench_id)
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
            agent_cfg, scenario_cfg, test_policy, test_epoch, mode
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
