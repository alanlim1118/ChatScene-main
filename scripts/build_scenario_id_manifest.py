#!/usr/bin/env python3
"""
Build (or preview) a scenario_id_manifest.json for a bench directory whose
.scenic files aren't named `scenario_NNN.scenic`.

This is a cheap, CARLA-free step you can run and review before the
(expensive) route-pickle generation. The manifest it writes is the source of
truth `generate_scenic_route_pickle.py`, `scenic_parse()`, and
`run_eval_batch.py`'s discovery fall back to when the legacy
`scenario_NNN.scenic` naming isn't present.

Example:
  python scripts/build_scenario_id_manifest.py \\
    --scenic-dir safebench/scenario/scenic_data/nl2scenic-bench/results/scenic/image-only
"""

import argparse
import os.path as osp
import sys

_REPO_ROOT = osp.abspath(osp.join(osp.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from safebench.util import scenario_id_manifest as sim


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenic-dir",
        type=str,
        required=True,
        help="Bench directory containing the .scenic files (manifest is written here).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resulting mapping without writing scenario_id_manifest.json.",
    )
    args = parser.parse_args()

    if not osp.isdir(args.scenic_dir):
        print(f"Error: not a directory: {args.scenic_dir}", file=sys.stderr)
        return 1

    existing = sim.load_manifest(args.scenic_dir)
    manifest = sim.build_manifest(args.scenic_dir, existing=existing)

    for sid in sorted(manifest["id_to_file"], key=int):
        print(f"{sid}\t{manifest['id_to_file'][sid]}")
    print(f"\n{len(manifest['id_to_file'])} scenario(s) mapped for bench_id={manifest['bench_id']!r}.")

    if args.dry_run:
        print("(dry run - not written)")
        return 0

    sim.write_manifest(args.scenic_dir, manifest)
    print(f"Wrote {sim.manifest_path(args.scenic_dir)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
