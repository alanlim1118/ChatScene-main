#!/usr/bin/env python3
"""
Rewrite `param carla_map = <identifier>` to a literal `param carla_map = '<value>'`
in nl2scenic-bench .scenic files.

All 250 nl2scenic-bench files declare the map indirectly, e.g.:

    Town = 'Town07'
    param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
    param carla_map = Town

scripts/generate_scenic_route_pickle.py's CARLA_MAP_RE only understands a
direct string literal (`param carla_map = 'Town07'`), so it fails with
"Missing `param carla_map`" on every one of these files. This script resolves
the indirection once, in the data, by finding the `<identifier> = '<value>'`
assignment the `param carla_map = <identifier>` line refers to, and rewriting
that one line to the literal value. Nothing else in the file is touched.

Usage:
  python scripts/fix_nl2scenic_carla_map.py \\
    safebench/scenario/scenic_data/nl2scenic-bench/results/scenic

  # preview only
  python scripts/fix_nl2scenic_carla_map.py --dry-run \\
    safebench/scenario/scenic_data/nl2scenic-bench/results/scenic
"""

import argparse
import os
import re
import sys

CARLA_MAP_LITERAL_RE = re.compile(r"^\s*param\s+carla_map\s*=\s*'[^']+'\s*$", re.MULTILINE)
CARLA_MAP_IDENT_RE = re.compile(r"^(\s*param\s+carla_map\s*=\s*)(\w+)(\s*)$", re.MULTILINE)


def resolve_and_rewrite(text: str, filename: str):
    """Returns (new_text, status) where status is one of:
    'already-literal', 'rewritten', 'no-carla-map-line', 'ambiguous'."""
    if CARLA_MAP_LITERAL_RE.search(text):
        return text, "already-literal"

    m = CARLA_MAP_IDENT_RE.search(text)
    if not m:
        return text, "no-carla-map-line"

    prefix, identifier, suffix = m.group(1), m.group(2), m.group(3)
    ident_re = re.compile(
        r"^\s*" + re.escape(identifier) + r"\s*=\s*'([^']+)'", re.MULTILINE
    )
    ident_matches = ident_re.findall(text)
    if len(ident_matches) != 1:
        return text, "ambiguous"

    value = ident_matches[0]
    new_line = f"{prefix}'{value}'{suffix}"
    new_text = text[: m.start()] + new_line + text[m.end() :]
    return new_text, "rewritten"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root_dir",
        help="Directory to scan recursively for .scenic files "
        "(e.g. safebench/scenario/scenic_data/nl2scenic-bench/results/scenic).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report what would change without writing any files.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.root_dir):
        print(f"Error: not a directory: {args.root_dir}", file=sys.stderr)
        return 1

    counts = {"already-literal": 0, "rewritten": 0, "no-carla-map-line": 0, "ambiguous": 0}
    problems = []

    for root, _, files in os.walk(args.root_dir):
        for fn in sorted(files):
            if not fn.endswith(".scenic"):
                continue
            path = os.path.join(root, fn)
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()

            new_text, status = resolve_and_rewrite(text, path)
            counts[status] += 1
            if status in ("no-carla-map-line", "ambiguous"):
                problems.append((path, status))
            elif status == "rewritten":
                print(f"[rewrite] {path}")
                if not args.dry_run:
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(new_text)

    print()
    print(
        f"already-literal={counts['already-literal']}  "
        f"rewritten={counts['rewritten']}  "
        f"no-carla-map-line={counts['no-carla-map-line']}  "
        f"ambiguous={counts['ambiguous']}"
    )
    if problems:
        print("\nFiles needing manual attention:")
        for path, status in problems:
            print(f"  [{status}] {path}")
    if args.dry_run:
        print("\n(dry run - no files written)")

    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
