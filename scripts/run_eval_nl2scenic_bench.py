#!/usr/bin/env python3
"""
Loop scripts/run_eval_batch.py over all 5 nl2scenic-bench modality directories
(image-only, text-image, text-only, text-video, video-only), each backed by
its own safebench/scenario/config/eval_scenic_nl2scenic_<modality>.yaml.

Mirrors how scripts/run_eval_batch.py itself subprocess-invokes
scripts/run_eval.py: one subprocess per (modality, mode).

Example (train_scenario then eval, across all 5 modalities):
  python scripts/run_eval_nl2scenic_bench.py \\
    --test_policy ppo --route_id 0 --port 2005 --tm_port 8005 --device cpu \\
    --average

Preview without running CARLA:
  python scripts/run_eval_nl2scenic_bench.py --dry_run
"""

import argparse
import os.path as osp
import subprocess
import sys
from typing import List, Optional, Sequence, Tuple

_REPO_ROOT = osp.abspath(osp.join(osp.dirname(__file__), ".."))
RUN_EVAL_BATCH_SCRIPT = osp.join(_REPO_ROOT, "scripts", "run_eval_batch.py")

MODALITIES = ["image-only", "text-image", "text-only", "text-video", "video-only"]


def scenario_cfg_for(modality: str) -> str:
    return f"eval_scenic_nl2scenic_{modality}.yaml"


def build_cmd(
    modality: str,
    mode: str,
    args: argparse.Namespace,
    forward_args: Sequence[str],
) -> List[str]:
    cmd = [
        sys.executable,
        RUN_EVAL_BATCH_SCRIPT,
        "--scenario_cfg", scenario_cfg_for(modality),
        "--agent_cfg", args.agent_cfg,
        "--mode", mode,
        "--test_policy", args.test_policy,
        "--port", str(args.port),
        "--tm_port", str(args.tm_port),
        "--device", args.device,
        "--route_id", str(args.route_id),
    ]
    if args.test_epoch is not None:
        cmd.extend(["--test_epoch", str(args.test_epoch)])
    if args.scenario_ids:
        cmd.append("--scenario_ids")
        cmd.extend(str(i) for i in args.scenario_ids)
    if args.scenario_range:
        cmd.extend(["--scenario_range", args.scenario_range])
    if not args.continue_on_error:
        cmd.append("--no-continue_on_error")
    if mode == "eval" and args.average:
        cmd.append("--average")
    if args.save_video:
        cmd.append("--save_video")
    if args.dry_run:
        cmd.append("--dry_run")
    cmd.extend(forward_args)
    return cmd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--modalities",
        nargs="+",
        default=MODALITIES,
        choices=MODALITIES,
        help="Subset of modality dirs to run (default: all 5).",
    )
    parser.add_argument(
        "--modes",
        nargs="+",
        default=["train_scenario", "eval"],
        choices=["train_scenario", "eval"],
        help="Modes to run per modality, in order (default: train_scenario then eval).",
    )
    parser.add_argument("--agent_cfg", type=str, default="adv_scenic.yaml")
    parser.add_argument("--test_policy", type=str, default="ppo")
    parser.add_argument("--test_epoch", type=int, default=None)
    parser.add_argument("--route_id", type=int, default=0)
    parser.add_argument("--port", type=int, default=2005)
    parser.add_argument("--tm_port", type=int, default=8005)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--save_video", action="store_true")
    parser.add_argument("--scenario_ids", nargs="+", type=int, default=None)
    parser.add_argument("--scenario_range", type=str, default=None)
    parser.add_argument(
        "--continue_on_error",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Continue with remaining (modality, mode) runs if one fails.",
    )
    parser.add_argument("--average", action="store_true")
    parser.add_argument("--dry_run", action="store_true")

    args, forward_args = parser.parse_known_args()

    results: List[Tuple[str, str, Optional[int]]] = []
    for modality in args.modalities:
        for mode in args.modes:
            cmd = build_cmd(modality, mode, args, forward_args)
            print(f"\n=== modality={modality} mode={mode} ===")
            print(" ".join(cmd))

            returncode: Optional[int] = None
            if not args.dry_run:
                returncode = subprocess.run(cmd, cwd=_REPO_ROOT).returncode
                if returncode != 0:
                    print(
                        f"modality={modality} mode={mode} failed with exit code {returncode}.",
                        file=sys.stderr,
                    )
                    if not args.continue_on_error:
                        results.append((modality, mode, returncode))
                        print("Stopping because --no-continue_on_error was set.")
                        return _summarize(results, args.dry_run)
            results.append((modality, mode, returncode))

    return _summarize(results, args.dry_run)


def _summarize(
    results: Sequence[Tuple[str, str, Optional[int]]], dry_run: bool
) -> int:
    print("\n=== nl2scenic-bench summary ===")
    if dry_run:
        print(f"Dry run only. {len(results)} command(s) prepared, 0 executed.")
        return 0

    failed = [(m, mode, rc) for m, mode, rc in results if rc != 0]
    succeeded = [(m, mode) for m, mode, rc in results if rc == 0]
    print(f"Succeeded ({len(succeeded)}): {succeeded}")
    if failed:
        print(f"Failed ({len(failed)}):")
        for modality, mode, rc in failed:
            print(f"  modality={modality} mode={mode} exit_code={rc}")
        return 1
    print("Failed (0): []")
    return 0


if __name__ == "__main__":
    sys.exit(main())
