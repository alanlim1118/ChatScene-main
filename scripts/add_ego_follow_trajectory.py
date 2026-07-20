#!/usr/bin/env python3
"""Add FollowTrajectoryBehavior to ego in .scenic files that define egoTrajectory."""

import argparse
import os
import re
import sys
from typing import List, Tuple

DEFAULT_ROOT = (
    "safebench/scenario/scenario_data/scenic_data_chatscene/chatscene_overlap_nl2scenic"
)

EGO_TRAJECTORY_RE = re.compile(r"^\s*egoTrajectory\s*=", re.MULTILINE)
OPT_EGO_SPEED_LINE = "param OPT_EGO_SPEED = 10"
ALREADY_DONE_RE = re.compile(
    r"with\s+behavior\s+FollowTrajectoryBehavior\s*\(\s*globalParameters\.OPT_EGO_SPEED\s*,\s*egoTrajectory\s*\)"
)
# Ego block ending with blueprint specifier (no trailing comma on blueprint line).
EGO_BLUEPRINT_END_RE = re.compile(
    r"(^ego\s*=\s*Car\s+at\s+[^\n]+,\n"
    r"(?:^[ \t]+with\s+[^\n]+\n)*"
    r"^[ \t]+with\s+blueprint\s+(\w+)[ \t]*)$",
    re.MULTILINE,
)
BROKEN_COMMA_RE = re.compile(
    r"(^[ \t]+with\s+blueprint\s+\w+)\n,\n"
    r"(^[ \t]+with\s+behavior\s+FollowTrajectoryBehavior)",
    re.MULTILINE,
)
BEHAVIOR_LINE = (
    ",\n    with behavior FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)"
)


def repair_broken_comma(content: str) -> Tuple[str, bool]:
    """Fix migration bug where comma landed on its own line after blueprint."""
    repaired, n = BROKEN_COMMA_RE.subn(r"\1,\n\2", content)
    return repaired, n > 0


def migrate_content(content: str) -> Tuple[str, bool, str]:
    """Return (new_content, changed, reason)."""
    if not EGO_TRAJECTORY_RE.search(content):
        return content, False, "no egoTrajectory"

    new_content, comma_fixed = repair_broken_comma(content)
    if comma_fixed and ALREADY_DONE_RE.search(new_content):
        return new_content, True, "repaired comma"

    if ALREADY_DONE_RE.search(new_content):
        return new_content, False, "already migrated"

    content = new_content

    if "OPT_EGO_SPEED" not in content:
        m = EGO_TRAJECTORY_RE.search(content)
        if m is None:
            return content, False, "no egoTrajectory"
        insert_at = m.start()
        prefix = content[:insert_at]
        suffix = content[insert_at:]
        if prefix and not prefix.endswith("\n"):
            prefix += "\n"
        new_content = prefix + OPT_EGO_SPEED_LINE + "\n\n" + suffix.lstrip("\n")
    else:
        new_content = content

    def _patch_ego_block(match: re.Match) -> str:
        block = match.group(1)
        if ALREADY_DONE_RE.search(block):
            return block
        return block + BEHAVIOR_LINE

    patched, n = EGO_BLUEPRINT_END_RE.subn(_patch_ego_block, new_content, count=1)
    if n == 0:
        return content, False, "ego block pattern not matched"

    if patched == content:
        return content, False, "unchanged"

    return patched, True, "modified"


def iter_scenic_files(root: str) -> List[str]:
    paths: List[str] = []
    for dirpath, _, filenames in os.walk(root):
        for fn in sorted(filenames):
            if fn.endswith(".scenic"):
                paths.append(os.path.join(dirpath, fn))
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Add ego FollowTrajectoryBehavior for files with egoTrajectory."
    )
    parser.add_argument(
        "--root",
        default=DEFAULT_ROOT,
        help=f"Directory tree to scan (default: {DEFAULT_ROOT})",
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--dry-run", action="store_true", help="Print changes without writing")
    group.add_argument("--apply", action="store_true", help="Write modified files")
    args = parser.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        raise SystemExit(f"Not a directory: {root}")

    modified: List[str] = []
    skipped: List[Tuple[str, str]] = []

    for path in iter_scenic_files(root):
        with open(path, "r", encoding="utf-8") as f:
            original = f.read()
        new_content, changed, reason = migrate_content(original)
        rel = os.path.relpath(path, root)
        if changed:
            modified.append(rel)
            if args.apply:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(new_content)
        elif reason != "no egoTrajectory":
            skipped.append((rel, reason))

    print(f"Scanned: {len(iter_scenic_files(root))} files under {root}")
    print(f"Modified: {len(modified)}")
    for rel in modified:
        print(f"  + {rel}")
    if skipped:
        print(f"Skipped (not modified): {len(skipped)}")
        for rel, reason in skipped:
            print(f"  - {rel}: {reason}")

    if args.dry_run and modified:
        print("\nRe-run with --apply to write changes.")
    if args.apply and modified:
        print(f"\nWrote {len(modified)} file(s).")

    if skipped and any(r == "ego block pattern not matched" for _, r in skipped):
        sys.exit(1)


if __name__ == "__main__":
    main()
