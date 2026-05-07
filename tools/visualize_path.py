#!/usr/bin/env python3
"""Visualize laser air-move planner CSV outputs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable


def import_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required. Install it with: python -m pip install matplotlib"
        ) from exc
    return plt, Poly3DCollection


def read_points_csv(path: Path) -> list[tuple[float, float, float]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return [(float(r["x"]), float(r["y"]), float(r["z"])) for r in reader]


def read_trajectory_csv(path: Path) -> dict[str, list[float]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        data: dict[str, list[float]] = {name: [] for name in reader.fieldnames or []}
        for row in reader:
            for name in data:
                data[name].append(float(row[name]))
        return data


def box_faces(center: Iterable[float], size: Iterable[float]) -> list[list[tuple[float, float, float]]]:
    cx, cy, cz = center
    sx, sy, sz = (v * 0.5 for v in size)
    p = [
        (cx - sx, cy - sy, cz - sz),
        (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz),
        (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz),
        (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz),
        (cx - sx, cy + sy, cz + sz),
    ]
    return [
        [p[0], p[1], p[2], p[3]],
        [p[4], p[5], p[6], p[7]],
        [p[0], p[1], p[5], p[4]],
        [p[2], p[3], p[7], p[6]],
        [p[1], p[2], p[6], p[5]],
        [p[3], p[0], p[4], p[7]],
    ]


def read_ascii_stl_triangles(path: Path) -> list[list[tuple[float, float, float]]]:
    triangles: list[list[tuple[float, float, float]]] = []
    current: list[tuple[float, float, float]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 4 and parts[0] == "vertex":
                current.append((float(parts[1]), float(parts[2]), float(parts[3])))
                if len(current) == 3:
                    triangles.append(current)
                    current = []
    return triangles


def cylinder_side_faces(
    center: Iterable[float], radius: float, height: float, segments: int = 32
) -> list[list[tuple[float, float, float]]]:
    cx, cy, cz = center
    z0 = cz - height * 0.5
    z1 = cz + height * 0.5
    faces: list[list[tuple[float, float, float]]] = []
    for i in range(segments):
        a0 = 2.0 * math.pi * i / segments
        a1 = 2.0 * math.pi * (i + 1) / segments
        p0 = (cx + radius * math.cos(a0), cy + radius * math.sin(a0), z0)
        p1 = (cx + radius * math.cos(a1), cy + radius * math.sin(a1), z0)
        p2 = (cx + radius * math.cos(a1), cy + radius * math.sin(a1), z1)
        p3 = (cx + radius * math.cos(a0), cy + radius * math.sin(a0), z1)
        faces.append([p0, p1, p2, p3])
    return faces


def sphere_points(
    center: Iterable[float], radius: float, rings: int = 8, segments: int = 24
) -> list[tuple[float, float, float]]:
    cx, cy, cz = center
    points: list[tuple[float, float, float]] = []
    for r in range(1, rings):
        phi = math.pi * r / rings
        z = cz + radius * math.cos(phi)
        rr = radius * math.sin(phi)
        for s in range(segments):
            theta = 2.0 * math.pi * s / segments
            points.append((cx + rr * math.cos(theta), cy + rr * math.sin(theta), z))
    return points


def set_axes_equal(ax, points: list[tuple[float, float, float]]) -> None:
    if not points:
        return
    xs, ys, zs = zip(*points)
    ranges = [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]
    radius = max(max(ranges) * 0.5, 1.0)
    centers = [
        (max(xs) + min(xs)) * 0.5,
        (max(ys) + min(ys)) * 0.5,
        (max(zs) + min(zs)) * 0.5,
    ]
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def collect_obstacle_faces(config: dict, config_path: Path) -> list[list[tuple[float, float, float]]]:
    faces: list[list[tuple[float, float, float]]] = []
    base_dir = config_path.parent
    for obstacle in config.get("obstacles", []):
        if obstacle.get("type") == "box":
            faces.extend(box_faces(obstacle["center"], obstacle["size"]))
        elif obstacle.get("type") == "cylinder":
            faces.extend(cylinder_side_faces(obstacle["center"], obstacle["radius"], obstacle["height"]))
        elif obstacle.get("type") == "ascii_stl":
            stl_path = Path(obstacle["file"])
            if not stl_path.is_absolute():
                stl_path = base_dir / stl_path
            faces.extend(read_ascii_stl_triangles(stl_path))
    return faces


def collect_sphere_samples(config: dict) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for obstacle in config.get("obstacles", []):
        if obstacle.get("type") == "sphere":
            points.extend(sphere_points(obstacle["center"], obstacle["radius"]))
    return points


def add_xy_obstacle_patches(plt, ax, config: dict) -> None:
    for obstacle in config.get("obstacles", []):
        obstacle_type = obstacle.get("type")
        if obstacle_type == "box":
            cx, cy, _ = obstacle["center"]
            sx, sy, _ = obstacle["size"]
            patch = plt.Rectangle(
                (cx - sx * 0.5, cy - sy * 0.5),
                sx,
                sy,
                facecolor="#D55E00",
                edgecolor="#8C3A00",
                alpha=0.22,
            )
            ax.add_patch(patch)
        elif obstacle_type == "sphere":
            cx, cy, _ = obstacle["center"]
            patch = plt.Circle(
                (cx, cy),
                obstacle["radius"],
                facecolor="#D55E00",
                edgecolor="#8C3A00",
                alpha=0.22,
            )
            ax.add_patch(patch)
        elif obstacle_type == "cylinder":
            cx, cy, _ = obstacle["center"]
            patch = plt.Circle(
                (cx, cy),
                obstacle["radius"],
                facecolor="#D55E00",
                edgecolor="#8C3A00",
                alpha=0.22,
            )
            ax.add_patch(patch)


def plot_path_3d(config: dict, config_path: Path, raw, shortcut, smooth, output: Path) -> None:
    plt, Poly3DCollection = import_matplotlib()
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    if raw:
        ax.plot(*zip(*raw), color="#777777", linewidth=1.2, label="raw path")
    if shortcut:
        ax.plot(*zip(*shortcut), color="#E69F00", linewidth=1.6, linestyle="--", label="shortcut path")
    if smooth:
        ax.plot(*zip(*smooth), color="#0072B2", linewidth=2.0, label="smoothed path")

    faces = collect_obstacle_faces(config, config_path)
    if faces:
        collection = Poly3DCollection(
            faces, facecolors="#D55E00", edgecolors="#8C3A00", alpha=0.22, linewidths=0.8
        )
        ax.add_collection3d(collection)

    sphere_samples = collect_sphere_samples(config)
    if sphere_samples:
        ax.scatter(*zip(*sphere_samples), color="#D55E00", s=4, alpha=0.16, label="sphere obstacle")

    request = config.get("request", {})
    if "start" in request:
        ax.scatter(*request["start"]["xyz"], color="#009E73", s=50, label="start")
    if "goal" in request:
        ax.scatter(*request["goal"]["xyz"], color="#CC79A7", s=50, label="goal")

    all_points = list(raw) + list(shortcut) + list(smooth) + sphere_samples
    for face in faces:
        all_points.extend(face)
    set_axes_equal(ax, all_points)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Air-Move Path")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_path_xy(config: dict, config_path: Path, raw, shortcut, smooth, output: Path) -> None:
    plt, _ = import_matplotlib()
    fig, ax = plt.subplots(figsize=(8, 7))

    if raw:
        xs, ys, _ = zip(*raw)
        ax.plot(xs, ys, color="#777777", linewidth=1.2, label="raw path")
    if shortcut:
        xs, ys, _ = zip(*shortcut)
        ax.plot(xs, ys, color="#E69F00", linewidth=1.6, linestyle="--", label="shortcut path")
    if smooth:
        xs, ys, _ = zip(*smooth)
        ax.plot(xs, ys, color="#0072B2", linewidth=2.0, label="smoothed path")

    add_xy_obstacle_patches(plt, ax, config)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Air-Move Path XY View")
    ax.grid(True, linewidth=0.4, alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_path_lengths(summary: dict, output: Path) -> None:
    plt, _ = import_matplotlib()
    labels = ["raw", "shortcut", "smooth"]
    values = [
        float(summary.get("raw_path_length", 0.0)),
        float(summary.get("shortcut_path_length", 0.0)),
        float(summary.get("smoothed_path_length", 0.0)),
    ]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(labels, values, color=["#777777", "#E69F00", "#0072B2"])
    ax.set_ylabel("Length")
    ax.set_title("Path Length Comparison")
    ax.grid(True, axis="y", linewidth=0.4, alpha=0.4)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_summary_dashboard(summary: dict, output: Path) -> None:
    plt, _ = import_matplotlib()
    metrics = [
        ("duration", summary.get("trajectory_duration", 0.0)),
        ("min clearance", summary.get("min_clearance", 0.0)),
        ("avg clearance", summary.get("average_clearance", 0.0)),
        ("max curvature", summary.get("max_curvature", 0.0)),
        ("jerk integral", summary.get("jerk_integral", 0.0)),
    ]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.axis("off")
    title = f"Planning Summary - {'success' if summary.get('success') else 'failed'}"
    ax.text(0.02, 0.92, title, fontsize=16, weight="bold", transform=ax.transAxes)
    ax.text(0.02, 0.82, str(summary.get("message", "")), fontsize=10, transform=ax.transAxes)

    for i, (name, value) in enumerate(metrics):
        x = 0.04 + (i % 3) * 0.31
        y = 0.58 - (i // 3) * 0.28
        if isinstance(value, float) and math.isinf(value):
            text = "inf"
        elif isinstance(value, (float, int)):
            text = f"{float(value):.4g}"
        else:
            text = str(value)
        ax.text(x, y + 0.08, name, fontsize=10, color="#555555", transform=ax.transAxes)
        ax.text(x, y, text, fontsize=18, weight="bold", transform=ax.transAxes)

    smoothing = summary.get("smoothing_message", "")
    ax.text(0.02, 0.08, f"smoothing: {smoothing}", fontsize=10, transform=ax.transAxes)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_motion_profiles(config: dict, trajectory: dict[str, list[float]], output: Path) -> None:
    plt, _ = import_matplotlib()
    time = trajectory.get("time", [])
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    limits = config.get("motion_limits", {})

    groups = [
        ("Velocity", ("vx", "vy", "vz"), limits.get("max_velocity")),
        ("Acceleration", ("ax", "ay", "az"), limits.get("max_acceleration")),
        ("Jerk", ("jx", "jy", "jz"), limits.get("max_jerk")),
    ]

    colors = ["#0072B2", "#009E73", "#D55E00"]
    for ax, (title, names, limit) in zip(axes, groups):
        for name, color in zip(names, colors):
            ax.plot(time, trajectory.get(name, []), label=name, linewidth=1.2, color=color)
        if limit:
            max_limit = max(abs(float(v)) for v in limit)
            ax.axhline(max_limit, color="#555555", linestyle="--", linewidth=0.8)
            ax.axhline(-max_limit, color="#555555", linestyle="--", linewidth=0.8)
        ax.set_ylabel(title)
        ax.grid(True, linewidth=0.4, alpha=0.4)
        ax.legend(loc="upper right")

    axes[-1].set_xlabel("Time")
    fig.suptitle("Trajectory Motion Profiles")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize laser air-move planner outputs.")
    parser.add_argument("--config", required=True, type=Path, help="Planner JSON config file.")
    parser.add_argument("--input", required=True, type=Path, help="Directory containing planner CSV outputs.")
    parser.add_argument("--output", required=True, type=Path, help="Directory for visualization PNG files.")
    args = parser.parse_args()

    config = load_config(args.config)
    raw = read_points_csv(args.input / "raw_path.csv")
    shortcut = read_points_csv(args.input / "shortcut_path.csv")
    smooth = read_points_csv(args.input / "smoothed_path.csv")
    trajectory = read_trajectory_csv(args.input / "trajectory.csv")
    summary_path = args.input / "summary.json"
    summary = load_config(summary_path) if summary_path.exists() else {}

    args.output.mkdir(parents=True, exist_ok=True)
    plot_path_3d(config, args.config, raw, shortcut, smooth, args.output / "path_3d.png")
    plot_path_xy(config, args.config, raw, shortcut, smooth, args.output / "path_xy.png")
    if trajectory:
        plot_motion_profiles(config, trajectory, args.output / "motion_profiles.png")
    if summary:
        plot_path_lengths(summary, args.output / "path_lengths.png")
        plot_summary_dashboard(summary, args.output / "summary_dashboard.png")

    print(f"visualization written to: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
