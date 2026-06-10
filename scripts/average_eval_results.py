#!/usr/bin/env python3
"""Average evaluation metrics from OPT_scenario_*_ROUTE-0_results.pkl files."""

import argparse
import json
import os
import os.path as osp
import re
import sys
from collections import defaultdict
from typing import Dict, List, Tuple

import joblib

RESULTS_PATTERN = re.compile(r"^OPT_scenario_(\d+)_ROUTE-0_results\.pkl$")
DEFAULT_OUTPUT_NAME = "average_eval_results.json"
METRIC_PRECISION = 6


def find_result_files(directory: str) -> List[str]:
    """Recursively find matching results pickle files under directory."""
    matches = []
    for root, _, files in os.walk(directory):
        for filename in files:
            if RESULTS_PATTERN.match(filename):
                matches.append(osp.join(root, filename))
    return sorted(matches)


def load_metrics(filepath: str) -> Dict[str, float]:
    """Load a results pickle and return its metric dict."""
    data = joblib.load(filepath)
    if not isinstance(data, dict):
        raise TypeError(
            f"Expected dict in {filepath}, got {type(data).__name__}"
        )
    return data


def average_metrics(
    filepaths: List[str],
) -> Tuple[Dict[str, float], List[str]]:
    """Compute average metrics across all result files."""
    totals: Dict[str, float] = defaultdict(float)
    counts: Dict[str, int] = defaultdict(int)
    source_files: List[str] = []

    for filepath in filepaths:
        metrics = load_metrics(filepath)
        for key, value in metrics.items():
            if isinstance(value, (int, float)):
                totals[key] += float(value)
                counts[key] += 1
        source_files.append(filepath)

    averages = {
        key: round(totals[key] / counts[key], METRIC_PRECISION)
        for key in sorted(totals.keys())
    }
    return averages, source_files


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Average evaluation metrics from OPT_scenario_XXX_ROUTE-0_results.pkl "
            "files found recursively under a directory."
        )
    )
    parser.add_argument(
        "directory",
        help="Root directory to search for results pickle files",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help=(
            "Output JSON path "
            f"(default: <directory>/{DEFAULT_OUTPUT_NAME})"
        ),
    )
    args = parser.parse_args()

    directory = osp.abspath(args.directory)
    if not osp.isdir(directory):
        print(f"Error: not a directory: {directory}", file=sys.stderr)
        return 1

    result_files = find_result_files(directory)
    if not result_files:
        print(
            "Error: no matching pickle files found.\n"
            "Expected filenames like: OPT_scenario_001_ROUTE-0_results.pkl",
            file=sys.stderr,
        )
        return 1

    try:
        averages, source_files = average_metrics(result_files)
    except (OSError, TypeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    output_path = (
        osp.abspath(args.output)
        if args.output
        else osp.join(directory, DEFAULT_OUTPUT_NAME)
    )

    output = {
        "num_pickle_files": len(result_files),
        "source_files": [
            osp.relpath(path, directory) for path in source_files
        ],
        "average_metrics": averages,
    }

    os.makedirs(osp.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
        f.write("\n")

    print(f"Averaged {len(result_files)} pickle files.")
    print(f"Wrote results to {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
