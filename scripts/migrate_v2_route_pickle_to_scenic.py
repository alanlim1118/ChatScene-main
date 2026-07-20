#!/usr/bin/env python3
"""Convert legacy Scenic 2.x route pickles to the unified v1 schema.

Legacy v2 entries stored `trajectory` in CARLA coords and optional
`route_format` metadata. Unified pickles store all route geometry in Scenic
coords (same as pipeline v1 / generate_scenic_route_pickle.py).

Only entries with route_format.trajectory == "carla" are modified.
"""

import argparse
import math
import os.path as osp
import pickle
import shutil
from typing import Dict, List, Tuple


def _carla_to_scenic_point(point) -> Tuple[float, float, float]:
    x, y, z = float(point[0]), float(point[1]), float(point[2])
    return (x, -y, z)


def _scenic_radians_to_carla_degrees(yaw: float) -> float:
    """Inverse of the old v2 generator's spawn yaw write path."""
    return math.degrees(-yaw) - 90.0


def _migrate_entry(entry: Dict) -> bool:
    if not isinstance(entry, dict):
        return False

    route_format = entry.get("route_format") or {}
    if route_format.get("trajectory") != "carla":
        return False

    trajectory = entry.get("trajectory")
    if isinstance(trajectory, list) and trajectory:
        entry["trajectory"] = [_carla_to_scenic_point(p) for p in trajectory]

    spawn_pt = entry.get("spawnPt")
    if isinstance(spawn_pt, dict) and route_format.get("spawn_yaw") == "scenic_radians":
        yaw = float(spawn_pt["yaw"])
        if abs(yaw) <= math.pi + 0.1:
            spawn_pt["yaw"] = _scenic_radians_to_carla_degrees(yaw)

    entry.pop("route_format", None)
    return True


def migrate_pickle(data: Dict) -> List[str]:
    """Return keys of entries that were migrated."""
    migrated: List[str] = []
    for key, entry in data.items():
        if _migrate_entry(entry):
            migrated.append(key)
    return migrated


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Migrate legacy v2 route pickle entries to Scenic-coord schema."
    )
    parser.add_argument(
        "pickle_path",
        type=str,
        help="Path to scenic_route.pickle",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report changes without writing the pickle.",
    )
    parser.add_argument(
        "--backup",
        action="store_true",
        default=True,
        help="Write a .bak copy before modifying (default: on).",
    )
    parser.add_argument(
        "--no-backup",
        action="store_false",
        dest="backup",
        help="Skip backup when writing.",
    )
    args = parser.parse_args()

    pickle_path = args.pickle_path
    if not osp.exists(pickle_path):
        raise SystemExit(f"Pickle not found: {pickle_path}")

    with open(pickle_path, "rb") as f:
        data = pickle.load(f)
    if not isinstance(data, dict):
        raise SystemExit(f"Expected dict in pickle, got {type(data)}")

    migrated = migrate_pickle(data)
    print(f"Entries to migrate: {len(migrated)}")
    for key in migrated:
        entry = data[key]
        traj = entry.get("trajectory") or []
        wps = entry.get("waypoints") or []
        if traj and wps:
            print(
                f"  {key}: trajectory[0].y={traj[0][1]:.4f}, "
                f"waypoints[0].y={wps[0][1]:.4f}, "
                f"spawnPt.yaw={entry.get('spawnPt', {}).get('yaw')}"
            )
        else:
            print(f"  {key}")

    if args.dry_run:
        print("Dry run — no file written.")
        return

    if not migrated:
        print("Nothing to migrate.")
        return

    if args.backup:
        backup_path = pickle_path + ".bak"
        shutil.copy2(pickle_path, backup_path)
        print(f"Backup written to {backup_path}")

    with open(pickle_path, "wb") as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Updated {pickle_path}")


if __name__ == "__main__":
    main()
