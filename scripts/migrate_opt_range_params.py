#!/usr/bin/env python3
"""Add OPT_ prefix to Range-only param declarations in .scenic files."""

import argparse
import os
import re
from typing import Dict, List, Tuple

EXCLUDED_PARAMS = {"map", "carla_map", "weather"}
PARAM_RE = re.compile(r"^param\s+(\w+)\s*=(.+)$", re.MULTILINE)
RANGE_RE = re.compile(r"\bRange\s*\(")

# Explicit camelCase → OPT_UPPER_SNAKE overrides
CAMELCASE_MAP = {
    "pedDist": "OPT_PED_DIST",
    "pedOffset": "OPT_PED_OFFSET",
    "distAhead": "OPT_DIST_AHEAD",
    "distSideways": "OPT_DIST_SIDEWAYS",
    "motoDist": "OPT_MOTO_DIST",
}


def camel_to_upper_snake(name: str) -> str:
    """Convert camelCase param names to UPPER_SNAKE for OPT_ prefix."""
    if name in CAMELCASE_MAP:
        return CAMELCASE_MAP[name]
    if name.isupper() or "_" in name:
        return f"OPT_{name}"
    # camelCase: insert underscores before capitals
    out = []
    for i, ch in enumerate(name):
        if ch.isupper() and i > 0:
            out.append("_")
        out.append(ch.upper())
    return f"OPT_{''.join(out)}"


def build_rename_map(content: str) -> Dict[str, str]:
    """Return old_name -> new_name for Range params needing OPT_ prefix."""
    renames: Dict[str, str] = {}
    for match in PARAM_RE.finditer(content):
        name, rhs = match.group(1), match.group(2)
        if name in EXCLUDED_PARAMS or name.startswith("OPT_"):
            continue
        if not RANGE_RE.search(rhs):
            continue
        new_name = camel_to_upper_snake(name)
        if new_name != name:
            renames[name] = new_name
    return renames


def apply_renames(content: str, renames: Dict[str, str]) -> str:
    """Apply param and globalParameters renames to file content."""
    if not renames:
        return content

    # Sort by name length descending to avoid partial replacements
    for old_name in sorted(renames, key=len, reverse=True):
        new_name = renames[old_name]
        # param declaration
        content = re.sub(
            rf"^param\s+{re.escape(old_name)}\s*=",
            f"param {new_name} =",
            content,
            flags=re.MULTILINE,
        )
        # globalParameters references
        content = re.sub(
            rf"globalParameters\.{re.escape(old_name)}\b",
            f"globalParameters.{new_name}",
            content,
        )
    return content


def migrate_file(path: str, dry_run: bool = False) -> Tuple[bool, Dict[str, str]]:
    """Migrate one .scenic file. Returns (changed, renames)."""
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    renames = build_rename_map(content)
    if not renames:
        return False, renames
    new_content = apply_renames(content, renames)
    if new_content != content:
        if not dry_run:
            with open(path, "w", encoding="utf-8") as f:
                f.write(new_content)
        return True, renames
    return False, renames


def fix_stale_opt_refs(content: str) -> Tuple[str, int]:
    """Fix globalParameters.X where param OPT_X exists but ref wasn't updated."""
    opt_params = set()
    for match in PARAM_RE.finditer(content):
        name = match.group(1)
        if name.startswith("OPT_"):
            opt_params.add(name)

    fixes = 0
    for opt_name in sorted(opt_params, key=len, reverse=True):
        # OPT_ADV_SPEED -> ADV_SPEED (strip OPT_ prefix for stale ref lookup)
        if not opt_name.startswith("OPT_"):
            continue
        old_name = opt_name[4:]  # strip "OPT_"
        # Also try camelCase reverse for distAhead etc - use explicit mapping
        reverse_map = {v: k for k, v in CAMELCASE_MAP.items()}
        candidates = [old_name]
        if opt_name in reverse_map:
            candidates.append(reverse_map[opt_name])

        for candidate in candidates:
            pattern = rf"globalParameters\.{re.escape(candidate)}\b"
            if re.search(pattern, content):
                content = re.sub(pattern, f"globalParameters.{opt_name}", content)
                fixes += 1
    return content, fixes


def migrate_directory(root: str, dry_run: bool = False) -> None:
    """Migrate all .scenic files under root."""
    changed_files: List[str] = []
    stale_fixed = 0

    for dirpath, _, files in os.walk(root):
        for filename in sorted(files):
            if not filename.endswith(".scenic"):
                continue
            path = os.path.join(dirpath, filename)
            changed, renames = migrate_file(path, dry_run=dry_run)

            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
            fixed_content, fixes = fix_stale_opt_refs(content)
            if fixes > 0:
                stale_fixed += fixes
                if not dry_run:
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(fixed_content)
                changed = True

            if changed or renames:
                rel = os.path.relpath(path, root)
                rename_str = ", ".join(f"{k}->{v}" for k, v in sorted(renames.items()))
                status = "would change" if dry_run else "changed"
                if renames or fixes:
                    print(f"[{status}] {rel}: {rename_str or '(refs only)'}")

            if changed:
                changed_files.append(path)

    print(f"\nTotal files {'that would change' if dry_run else 'changed'}: {len(changed_files)}")
    if stale_fixed:
        print(f"Stale globalParameters refs fixed: {stale_fixed}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Add OPT_ prefix to Range-only param declarations in .scenic files."
    )
    parser.add_argument(
        "scenic_dir",
        nargs="?",
        default="safebench/scenario/scenario_data/scenic_data_wenting/scenic_route-driven_Chat2Scenic",
        help="Root directory of .scenic files to migrate",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print changes without writing files",
    )
    args = parser.parse_args()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    target = args.scenic_dir
    if not os.path.isabs(target):
        target = os.path.join(repo_root, target)

    if not os.path.isdir(target):
        print(f"Error: directory not found: {target}")
        return 1

    migrate_directory(target, dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
