#!/usr/bin/env python3
"""
Convert chatscene_overlap_c2s_route-driven Scenic files to truly route-driven format.

Replaces random intersection/lane-based ego placement with globalParameters-based
spawn point from route pickle, matching chatscene_overlap_nl2scenic_route-driven pattern.
"""

import re
import sys
import os


def adv_uses_ego_trajectory(content):
    """Check if AdvBehavior block references egoTrajectory."""
    m = re.search(r'behavior AdvBehavior\(\):(.*?)(?=\nparam |\nintersection |\nlaneSecsWith\w|\negoLaneSec\b|\negoInitLane\b|\negoManeuver\b|\negoSpawnPt\b|\nego = |\nEgoSpawnPt\b)',
                  content, re.DOTALL)
    return m is not None and 'egoTrajectory' in m.group(1)


def extract_maneuver_type(content):
    m = re.search(r'egoManeuver = Uniform\(\*filter\(lambda m: m\.type is ManeuverType\.(\w+)', content)
    return m.group(1) if m else 'STRAIGHT'


def transform_pattern_a(content, with_trajectory=False):
    """Pattern A/A+: intersection.maneuvers -> startLane, no require."""
    traj_line = 'egoTrajectory = PolylineRegion(globalParameters.waypoints)\n' if with_trajectory else ''

    route_block = (
        'EgoSpawnPt = globalParameters.spawnPt\n'
        'yaw = globalParameters.yaw\n'
        + traj_line +
        'egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)\n'
        '\n'
        "ego = Car at egoSpawnPt,\n"
        "    with rolename 'hero',\n"
        "    with regionContainedIn None,\n"
        "    with blueprint EGO_MODEL"
    )

    # Match the intersection setup through ego (with behavior)
    pat = re.compile(
        r'intersection = Uniform\(\*filter\(lambda i: [^\n]+, network\.intersections\)\)\n'
        r'egoManeuver = Uniform\(\*filter\(lambda m: m\.type is ManeuverType\.\w+, intersection\.maneuvers\)\)\n'
        r'egoInitLane = egoManeuver\.startLane\n'
        r'param OPT_EGO_SPEED = \d+\n'
        r'\n'
        r'egoTrajectory = \[[^\]]+\]\n'
        r'egoSpawnPt = OrientedPoint in egoInitLane\.centerline\n'
        r'\n'
        r'(?:# [^\n]+\n)?'  # optional comment before ego
        r"ego = Car at egoSpawnPt,\n"
        r"    with rolename 'hero',\n"
        r"    with regionContainedIn None,\n"
        r"    with blueprint EGO_MODEL,\n"
        r"    with behavior FollowTrajectoryBehavior\(globalParameters\.OPT_EGO_SPEED, egoTrajectory\)"
    )
    new, n = pat.subn(route_block, content)
    if n == 0:
        print("  WARNING: Pattern A match failed")
    return new


def transform_pattern_b(content):
    """Pattern B: has egoManeuver.conflictingManeuvers, must keep egoManeuver."""
    mtype = extract_maneuver_type(content)

    route_block = (
        'EgoSpawnPt = globalParameters.spawnPt\n'
        'yaw = globalParameters.yaw\n'
        'egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)\n'
        f'egoInitLane = network.laneAt(egoSpawnPt.position)\n'
        f'egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.{mtype}, egoInitLane.maneuvers))\n'
        '\n'
        "ego = Car at egoSpawnPt,\n"
        "    with rolename 'hero',\n"
        "    with regionContainedIn None,\n"
        "    with blueprint EGO_MODEL"
    )

    pat = re.compile(
        r'intersection = Uniform\(\*filter\(lambda i: [^\n]+, network\.intersections\)\)\n'
        r'egoManeuver = Uniform\(\*filter\(lambda m: m\.type is ManeuverType\.\w+, intersection\.maneuvers\)\)\n'
        r'egoInitLane = egoManeuver\.startLane\n'
        r'param OPT_EGO_SPEED = \d+\n'
        r'\n'
        r'egoTrajectory = \[[^\]]+\]\n'
        r'egoSpawnPt = OrientedPoint in egoInitLane\.centerline\n'
        r'\n'
        r"ego = Car at egoSpawnPt,\n"
        r"    with rolename 'hero',\n"
        r"    with regionContainedIn None,\n"
        r"    with blueprint EGO_MODEL,\n"
        r"    with behavior FollowTrajectoryBehavior\(globalParameters\.OPT_EGO_SPEED, egoTrajectory\)"
    )
    new, n = pat.subn(route_block, content)
    if n == 0:
        print("  WARNING: Pattern B match failed")
    return new


def transform_pattern_c(content):
    """Pattern C: incomingLanes -> startLane.centerline, has require."""
    mtype = extract_maneuver_type(content)

    route_block = (
        'EgoSpawnPt = globalParameters.spawnPt\n'
        'yaw = globalParameters.yaw\n'
        'egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)\n'
        'egoInitLane = network.laneAt(egoSpawnPt.position)\n'
        f'egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.{mtype}, egoInitLane.maneuvers))\n'
        'intersection = egoManeuver.intersection\n'
        '\n'
        '# Setting up the ego vehicle at the initial position\n'
        "ego = Car at egoSpawnPt,\n"
        "    with rolename 'hero',\n"
        "    with regionContainedIn None,\n"
        "    with blueprint EGO_MODEL\n"
        "\n"
        "require 10 <= (distance to intersection) <= 40"
    )

    pat = re.compile(
        r'intersection = Uniform\(\*filter\(lambda i: [^\n]+, network\.intersections\)\)\n'
        r'egoInitLane = Uniform\(\*intersection\.incomingLanes\)\n'
        r'egoManeuver = Uniform\(\*filter\(lambda m: m\.type is ManeuverType\.\w+, egoInitLane\.maneuvers\)\)\n'
        r'param OPT_EGO_SPEED = \d+\n'
        r'\n'
        r'egoTrajectory = \[[^\]]+\]\n'
        r'egoSpawnPt = OrientedPoint in egoManeuver\.startLane\.centerline\n'
        r'\n'
        r'# Setting up the ego vehicle at the initial position\n'
        r"ego = Car at egoSpawnPt,\n"
        r"    with rolename 'hero',\n"
        r"    with regionContainedIn None,\n"
        r"    with blueprint EGO_MODEL,\n"
        r"    with behavior FollowTrajectoryBehavior\(globalParameters\.OPT_EGO_SPEED, egoTrajectory\)\n"
        r"require 10 <= \(distance to intersection\) <= 40"
    )
    new, n = pat.subn(route_block, content)
    if n == 0:
        print("  WARNING: Pattern C match failed")
    return new


def transform_pattern_d(content, with_trajectory=False):
    """Pattern D/D+: laneSecsWithXxx loop for ego placement."""
    traj_line = 'egoTrajectory = PolylineRegion(globalParameters.waypoints)\n' if with_trajectory else ''

    route_lines = (
        'EgoSpawnPt = globalParameters.spawnPt\n'
        'yaw = globalParameters.yaw\n'
        + traj_line +
        'egoSpawnPt = OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)\n'
        '\n'
    )

    # Match the laneSecs block (comment + loop + select + egoSpawnPt line) + optional blank line
    # The block ends just before the ego comment or ego definition
    pat = re.compile(
        r'(?:# (?:Collecting|Identifying) [^\n]+\n)?'
        r'laneSecsWith\w+ = \[\]\n'
        r'for lane in network\.lanes:\n'
        r'    for laneSec in lane\.sections:\n'
        r'(?:        [^\n]+\n)+'       # all if/append lines
        r'\n'
        r'# Selecting a random lane section[^\n]*\n'
        r'egoLaneSec = Uniform\(\*laneSecsWith\w+\)\n'
        r'egoSpawnPt = OrientedPoint in egoLaneSec\.centerline\n'
        r'\n'
    )
    new, n = pat.subn(route_lines, content)
    if n == 0:
        print("  WARNING: Pattern D match failed")
    return new


def transform_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Skip already converted files
    if 'globalParameters.spawnPt' in content:
        print("  Already converted, skipping")
        return False

    has_laneSecs = bool(re.search(r'laneSecsWith\w+ = \[\]', content))
    has_incomingLanes = bool(re.search(r'egoInitLane = Uniform\(\*intersection\.incomingLanes\)', content))
    has_require = bool(re.search(r'require 10 <= \(distance to intersection\)', content))
    has_conflicting = bool(re.search(r'egoManeuver\.conflictingManeuvers', content))
    has_adv_ego_traj = adv_uses_ego_trajectory(content)
    has_follow_traj_behavior = bool(re.search(r'with behavior FollowTrajectoryBehavior', content))

    if has_laneSecs:
        pattern = 'D+' if has_adv_ego_traj else 'D'
        print(f"  Pattern: {pattern}")
        new_content = transform_pattern_d(content, with_trajectory=has_adv_ego_traj)
    elif has_require and has_incomingLanes:
        print("  Pattern: C")
        new_content = transform_pattern_c(content)
    elif has_conflicting:
        print("  Pattern: B")
        new_content = transform_pattern_b(content)
    elif has_follow_traj_behavior:
        pattern = 'A+' if has_adv_ego_traj else 'A'
        print(f"  Pattern: {pattern}")
        new_content = transform_pattern_a(content, with_trajectory=has_adv_ego_traj)
    else:
        print("  Unrecognized pattern, skipping")
        return False

    if new_content == content:
        print("  WARNING: No changes made")
        return False

    with open(filepath, 'w') as f:
        f.write(new_content)
    return True


def main():
    base_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'safebench/scenario/scenario_data/scenic_data_chatscene/chatscene_overlap_c2s_route-driven'
    )
    if len(sys.argv) > 1:
        base_dir = sys.argv[1]

    print(f"Processing: {base_dir}")

    scenic_files = sorted(
        os.path.join(r, f)
        for r, _, files in os.walk(base_dir)
        for f in files if f.endswith('.scenic')
    )

    success, failed = 0, 0
    for fp in scenic_files:
        rel = os.path.relpath(fp, base_dir)
        print(f"\n{rel}")
        if transform_file(fp):
            success += 1
            print("  OK")
        else:
            failed += 1

    print(f"\nDone: {success} converted, {failed} skipped/failed")


if __name__ == '__main__':
    main()
