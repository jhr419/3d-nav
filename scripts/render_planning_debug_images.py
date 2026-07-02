#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


def read_path_csv(path: Path):
    if not path.is_file():
        return []
    points = []
    with path.open("r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            points.append((float(row["x"]), float(row["y"]), float(row["z"])))
    return points


def read_clearance_csv(path: Path):
    if not path.is_file():
        return []
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            clearance = row.get("clearance", "inf")
            rows.append(
                (
                    (float(row["x"]), float(row["y"]), float(row["z"])),
                    math.inf if clearance == "inf" else float(clearance),
                    float(row.get("risk_cost", 0.0)),
                )
            )
    return rows


def read_pcd_xyz(path: Path, max_points: int):
    if not path.is_file():
        return []

    with path.open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                return []
            text = line.decode("utf-8", errors="replace").strip()
            if not text or text.startswith("#"):
                continue
            tokens = text.split()
            header[tokens[0].upper()] = tokens[1:]
            if tokens[0].upper() == "DATA":
                break

        if header.get("DATA", [""])[0].lower() != "ascii":
            return []
        fields = header.get("FIELDS", [])
        if not {"x", "y", "z"}.issubset(set(fields)):
            return []
        columns = [fields.index("x"), fields.index("y"), fields.index("z")]
        points = []
        for index, line in enumerate(stream):
            if max_points > 0 and index % max(1, int(1000000 / max_points)) != 0:
                continue
            tokens = line.decode("utf-8", errors="replace").split()
            if len(tokens) <= max(columns):
                continue
            points.append(tuple(float(tokens[column]) for column in columns))
            if max_points > 0 and len(points) >= max_points:
                break
        return points


def main():
    parser = argparse.ArgumentParser(description="Render nav3d planning debug CSVs into PNG images.")
    parser.add_argument("--debug-dir", default="debug", help="Directory containing latest_*.csv")
    parser.add_argument("--map-pcd", default="maps/map_preprocessed.pcd", help="Optional ASCII PCD background map")
    parser.add_argument("--max-map-points", type=int, default=20000)
    args = parser.parse_args()

    debug_dir = Path(args.debug_dir)
    screenshots = debug_dir / "screenshots"
    screenshots.mkdir(parents=True, exist_ok=True)

    raw_path = read_path_csv(debug_dir / "latest_raw_path.csv")
    optimized_path = read_path_csv(debug_dir / "latest_optimized_path.csv")
    clearance_rows = read_clearance_csv(debug_dir / "latest_clearance.csv")
    map_points = read_pcd_xyz(Path(args.map_pcd), args.max_map_points)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    map_arr = np.asarray(map_points) if map_points else np.empty((0, 3))

    def setup(title):
        fig, ax = plt.subplots(figsize=(10, 8), dpi=140)
        ax.set_title(title)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="box")
        if len(map_arr):
            ax.scatter(map_arr[:, 0], map_arr[:, 1], s=0.3, c="#444444", alpha=0.18)
        return fig, ax

    def plot_path(ax, points, color, label, linewidth=2.0):
        if not points:
            return False
        arr = np.asarray(points)
        ax.plot(arr[:, 0], arr[:, 1], color=color, linewidth=linewidth, label=label)
        ax.scatter(arr[:1, 0], arr[:1, 1], c="#2ca02c", s=35)
        ax.scatter(arr[-1:, 0], arr[-1:, 1], c="#d62728", s=35)
        return True

    fig, ax = setup("01 original global path")
    if plot_path(ax, raw_path, "#1f77b4", "raw"):
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots / "01_original_global_path.png")
    plt.close(fig)

    fig, ax = setup("02 clearance cost map")
    if clearance_rows:
        pts = np.asarray([row[0] for row in clearance_rows])
        values = np.asarray([min(1.0, row[1]) if math.isfinite(row[1]) else 1.0 for row in clearance_rows])
        sc = ax.scatter(pts[:, 0], pts[:, 1], c=values, s=8, cmap="RdYlGn")
        fig.colorbar(sc, ax=ax, label="clearance [m]")
    if plot_path(ax, optimized_path, "#00a65a", "optimized", 1.6):
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots / "02_clearance_cost_map.png")
    plt.close(fig)

    fig, ax = setup("03 optimized global path")
    if plot_path(ax, optimized_path, "#00a65a", "optimized"):
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots / "03_optimized_global_path.png")
    plt.close(fig)

    fig, ax = setup("04 local planner debug overview")
    has_path = plot_path(ax, optimized_path, "#00a65a", "global path for local tracker")
    ax.text(
        0.02,
        0.98,
        "Use RViz topics:\n/ego_local_trajectory_marker\n/ego_candidate_trajectories_marker\n"
        "/ego_local_target_marker\n/ego_local_map_marker\n/ego_collision_points_marker\n"
        "/ego_footprint_marker\n/ego_cmd_vel_marker\n/ego_debug_text",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "#888888"},
    )
    if has_path:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots / "04_local_planner_debug.png")
    plt.close(fig)

    fig, ax = setup("05 before after comparison")
    has_raw = plot_path(ax, raw_path, "#1f77b4", "raw", 1.4)
    has_optimized = plot_path(ax, optimized_path, "#00a65a", "optimized", 2.0)
    if has_raw or has_optimized:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots / "05_before_after_comparison.png")
    plt.close(fig)

    print(f"Rendered debug images to {screenshots}")


if __name__ == "__main__":
    main()
