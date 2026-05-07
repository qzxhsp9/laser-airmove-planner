#!/usr/bin/env python3
"""Visualize laser air-move benchmark CSV reports."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required. Install it with: python -m pip install matplotlib"
        ) from exc
    return plt


def read_report(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if not value:
        return 0.0
    if value.lower() == "inf":
        return float("inf")
    return float(value)


def as_int(row: dict[str, str], key: str) -> int:
    value = row.get(key, "")
    return int(value) if value else 0


def cap_infinite(values: list[float]) -> list[float]:
    finite = [v for v in values if v != float("inf")]
    cap = (max(finite) * 1.2) if finite else 1.0
    return [cap if v == float("inf") else v for v in values]


def scene_labels(rows: list[dict[str, str]]) -> list[str]:
    return [row.get("scene") or Path(row.get("config", "")).stem for row in rows]


def plot_outcomes(rows: list[dict[str, str]], output: Path) -> None:
    plt = import_matplotlib()
    labels = scene_labels(rows)
    colors = ["#009E73" if as_int(row, "outcome_matched") else "#D55E00" for row in rows]
    values = [1 for _ in rows]

    fig, ax = plt.subplots(figsize=(max(8, len(rows) * 1.2), 4))
    ax.bar(labels, values, color=colors)
    ax.set_ylim(0, 1.2)
    ax.set_ylabel("matched")
    ax.set_title("Benchmark Expected Outcome Match")
    ax.tick_params(axis="x", rotation=35)
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_timing(rows: list[dict[str, str]], output: Path) -> None:
    plt = import_matplotlib()
    labels = scene_labels(rows)
    wall = [as_float(row, "wall_time_ms") for row in rows]
    duration = [as_float(row, "trajectory_duration") * 1000.0 for row in rows]

    fig, ax = plt.subplots(figsize=(max(8, len(rows) * 1.2), 5))
    x = list(range(len(rows)))
    width = 0.38
    ax.bar([v - width * 0.5 for v in x], wall, width=width, label="wall time ms", color="#0072B2")
    ax.bar([v + width * 0.5 for v in x], duration, width=width, label="trajectory duration ms", color="#E69F00")
    ax.set_xticks(x, labels, rotation=35, ha="right")
    ax.set_ylabel("ms")
    ax.set_title("Benchmark Timing")
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_path_quality(rows: list[dict[str, str]], output: Path) -> None:
    plt = import_matplotlib()
    labels = scene_labels(rows)
    raw = [as_float(row, "raw_path_length") for row in rows]
    shortcut = [as_float(row, "shortcut_path_length") for row in rows]
    smooth = [as_float(row, "smoothed_path_length") for row in rows]

    fig, ax = plt.subplots(figsize=(max(8, len(rows) * 1.2), 5))
    x = list(range(len(rows)))
    width = 0.25
    ax.bar([v - width for v in x], raw, width=width, label="raw", color="#777777")
    ax.bar(x, shortcut, width=width, label="shortcut", color="#E69F00")
    ax.bar([v + width for v in x], smooth, width=width, label="smooth", color="#0072B2")
    ax.set_xticks(x, labels, rotation=35, ha="right")
    ax.set_ylabel("path length")
    ax.set_title("Benchmark Path Length")
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_clearance(rows: list[dict[str, str]], output: Path) -> None:
    plt = import_matplotlib()
    success_rows = [row for row in rows if as_int(row, "success")]
    labels = scene_labels(success_rows)
    min_clearance = cap_infinite([as_float(row, "min_clearance") for row in success_rows])
    avg_clearance = cap_infinite([as_float(row, "average_clearance") for row in success_rows])

    fig, ax = plt.subplots(figsize=(max(8, len(success_rows) * 1.2), 5))
    x = list(range(len(success_rows)))
    width = 0.38
    ax.bar([v - width * 0.5 for v in x], min_clearance, width=width, label="min", color="#D55E00")
    ax.bar([v + width * 0.5 for v in x], avg_clearance, width=width, label="average", color="#009E73")
    ax.set_xticks(x, labels, rotation=35, ha="right")
    ax.set_ylabel("clearance")
    ax.set_title("Benchmark Clearance")
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize laser air-move benchmark reports.")
    parser.add_argument("--report", required=True, type=Path, help="Benchmark CSV report.")
    parser.add_argument("--output", required=True, type=Path, help="Directory for visualization PNG files.")
    args = parser.parse_args()

    rows = read_report(args.report)
    if not rows:
        raise SystemExit("benchmark report is empty")

    args.output.mkdir(parents=True, exist_ok=True)
    plot_outcomes(rows, args.output / "benchmark_outcomes.png")
    plot_timing(rows, args.output / "benchmark_timing.png")
    plot_path_quality(rows, args.output / "benchmark_path_lengths.png")
    plot_clearance(rows, args.output / "benchmark_clearance.png")

    print(f"benchmark visualization written to: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
