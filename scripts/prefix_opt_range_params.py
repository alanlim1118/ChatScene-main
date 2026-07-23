#!/usr/bin/env python
"""Prefix `OPT_` onto Scenic `param` declarations whose value is a bare Range().

SafeBench only optimizes a scenario parameter when BOTH hold (see
`ScenicSimulator.get_params`, safebench/util/scenic_utils.py:240-254):

    if not param.startswith('OPT'): continue     # name convention
    if not _is_opt_range(value):    continue     # value has .low/.high, i.e. a Range

So `param ADV_SPEED = Range(6, 9)` is sampled randomly but never optimized,
while `param OPT_ADV_SPEED = Range(6, 9)` participates in train_scenario's
OPT search. This script renames the former into the latter, rewriting the
declaration and every `globalParameters.<NAME>` reference in the same file.

Only declarations whose right-hand side is a *bare* `Range(...)` are touched -
a composite RHS such as `Range(8, 12) - 1` or `globalParameters.X * Range(1.2, 1.5)`
is not a Range object at runtime, fails `_is_opt_range`, and would gain nothing
from the prefix. Those are reported as skipped so they can be handled by hand.

Usage:
    python scripts/prefix_opt_range_params.py --dry-run <dir> [<dir> ...]
    python scripts/prefix_opt_range_params.py <dir> [<dir> ...]
"""

import argparse
import os
import re
import sys

# `param NAME = Range(...)` with nothing else on the line (trailing comment ok).
BARE_RANGE_RE = re.compile(
    r"^(?P<prefix>param\s+)(?P<name>[A-Za-z_]\w*)(?P<mid>\s*=\s*)"
    r"(?P<value>Range\([^()]*\))\s*(?P<comment>#.*)?$"
)
# `param NAME = <anything mentioning Range(>` - superset, used to report skips.
ANY_RANGE_RE = re.compile(r"^param\s+(?P<name>[A-Za-z_]\w*)\s*=\s*(?P<rhs>.*Range\(.*)$")


def collect_files(roots):
    files = []
    for root in roots:
        if os.path.isfile(root) and root.endswith(".scenic"):
            files.append(root)
            continue
        for dirpath, _, filenames in os.walk(root):
            for fn in sorted(filenames):
                if fn.endswith(".scenic"):
                    files.append(os.path.join(dirpath, fn))
    return sorted(files)


def process(path, dry_run):
    with open(path, encoding="utf-8") as f:
        text = f.read()
    lines = text.splitlines(keepends=True)

    renames = {}       # old name -> new name
    skipped = []       # (name, rhs) for composite-RHS declarations
    for line in lines:
        stripped = line.rstrip("\n")
        m = BARE_RANGE_RE.match(stripped)
        if m:
            name = m.group("name")
            if not name.startswith("OPT"):
                renames[name] = f"OPT_{name}"
            continue
        m = ANY_RANGE_RE.match(stripped)
        if m and not m.group("name").startswith("OPT"):
            skipped.append((m.group("name"), m.group("rhs").strip()))

    if not renames:
        return renames, skipped, []

    # A file already containing the target name would end up with two params of
    # the same name - refuse rather than silently merge them.
    conflicts = [old for old, new in renames.items()
                 if re.search(rf"\bparam\s+{re.escape(new)}\b", text)]
    if conflicts:
        return {}, skipped, conflicts

    new_text = text
    for old, new in renames.items():
        # Declaration.
        new_text = re.sub(rf"^(param\s+){re.escape(old)}\b", rf"\g<1>{new}",
                          new_text, flags=re.MULTILINE)
        # References. Params are always read through globalParameters in Scenic,
        # so this is the complete reference set for a param name.
        new_text = re.sub(rf"\bglobalParameters\.{re.escape(old)}\b",
                          f"globalParameters.{new}", new_text)

    if not dry_run and new_text != text:
        with open(path, "w", encoding="utf-8") as f:
            f.write(new_text)

    return renames, skipped, []


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("roots", nargs="+", help="Directories (walked recursively) or .scenic files")
    ap.add_argument("--dry-run", action="store_true", help="Report changes without writing")
    args = ap.parse_args()

    files = collect_files(args.roots)
    if not files:
        print("No .scenic files found.", file=sys.stderr)
        return 1

    total_renames = 0
    changed_files = 0
    all_skipped = []
    all_conflicts = []

    for path in files:
        renames, skipped, conflicts = process(path, args.dry_run)
        if conflicts:
            all_conflicts.append((path, conflicts))
        if skipped:
            all_skipped.extend((path, name, rhs) for name, rhs in skipped)
        if renames:
            changed_files += 1
            total_renames += len(renames)
            rel = os.path.relpath(path)
            for old, new in renames.items():
                print(f"{rel}: {old} -> {new}")

    print(f"\n{len(files)} file(s) scanned, {changed_files} file(s) "
          f"{'would be ' if args.dry_run else ''}changed, {total_renames} param(s) renamed.")

    if all_skipped:
        print(f"\nSkipped ({len(all_skipped)}) - RHS is not a bare Range(), so "
              f"_is_opt_range() would reject it even with the prefix:")
        for path, name, rhs in all_skipped:
            print(f"  {os.path.relpath(path)}: param {name} = {rhs}")

    if all_conflicts:
        print(f"\nCONFLICTS ({len(all_conflicts)}) - target name already declared, "
              f"file left untouched:")
        for path, names in all_conflicts:
            print(f"  {os.path.relpath(path)}: {', '.join(names)}")

    if args.dry_run:
        print("\n(dry run - no files were modified)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
