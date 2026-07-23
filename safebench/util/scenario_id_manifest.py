'''
Per-bench-directory manifest mapping numeric scenario_id -> .scenic filename.

Lets bench directories whose .scenic files aren't named `scenario_NNN.scenic`
(e.g. descriptive/source-derived names) still work with the scenario_id-based
discovery and lookup used throughout the Scenic pipeline. Existing bench
directories that already use the `scenario_NNN.scenic` convention never need
this file: every consumer only falls back to it once the legacy pattern-based
resolution has already failed.
'''

import json
import os
import os.path as osp
import re

MANIFEST_NAME = "scenario_id_manifest.json"
SCENARIO_RE = re.compile(r"^scenario_(\d+)\.scenic$")


def manifest_path(bench_dir):
    return osp.join(bench_dir, MANIFEST_NAME)


def load_manifest(bench_dir):
    """Read-only load. Returns None if no manifest exists yet."""
    path = manifest_path(bench_dir)
    if not osp.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def build_manifest(bench_dir, existing=None):
    """Deterministically (re)build the id<->filename mapping for a directory.

    Files already named `scenario_NNN.scenic` keep their embedded id (so a
    coincidental match stays consistent with the legacy regex path used
    elsewhere). Every other file keeps its previously-assigned id if present
    in `existing`, otherwise gets the next free integer id, assigned in
    sorted (alphabetical) order so reruns over an unchanged directory
    reproduce the same manifest.
    """
    files = sorted(f for f in os.listdir(bench_dir) if f.endswith(".scenic"))

    id_to_file = dict(existing["id_to_file"]) if existing else {}
    file_to_id = {v: int(k) for k, v in id_to_file.items()}
    used_ids = set(int(k) for k in id_to_file)

    # Tier 1: files literally named scenario_NNN.scenic keep their embedded id.
    for f in files:
        m = SCENARIO_RE.match(f)
        if m and f not in file_to_id:
            i = int(m.group(1))
            id_to_file[str(i)] = f
            file_to_id[f] = i
            used_ids.add(i)

    # Tier 2: remaining files get the next free integer, in sorted order.
    next_id = 1
    for f in files:
        if f in file_to_id:
            continue
        while next_id in used_ids:
            next_id += 1
        id_to_file[str(next_id)] = f
        file_to_id[f] = next_id
        used_ids.add(next_id)

    return {
        "version": 1,
        "bench_id": osp.basename(bench_dir.rstrip(os.sep)),
        "id_to_file": id_to_file,
    }


def write_manifest(bench_dir, manifest):
    """Atomic write (tmp file + os.replace) to avoid partial/corrupt manifests."""
    path = manifest_path(bench_dir)
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp_path, path)


def load_or_build(bench_dir, write=True):
    """Load the existing manifest, extending it with any new files found.

    Safe to call repeatedly (e.g. once per bench dir at the start of a batch
    run) - ids already on disk are preserved; only genuinely new files get
    assigned new ids.
    """
    existing = load_manifest(bench_dir)
    manifest = build_manifest(bench_dir, existing=existing)
    if write:
        write_manifest(bench_dir, manifest)
    return manifest


def file_for_id(bench_dir, scenario_id):
    """Read-only lookup: scenario_id (int) -> filename, or None."""
    manifest = load_manifest(bench_dir)
    if not manifest:
        return None
    return manifest["id_to_file"].get(str(int(scenario_id)))


def id_for_file(bench_dir, filename):
    """Read-only lookup: filename -> scenario_id (int), or None."""
    manifest = load_manifest(bench_dir)
    if not manifest:
        return None
    for sid, fname in manifest["id_to_file"].items():
        if fname == filename:
            return int(sid)
    return None
