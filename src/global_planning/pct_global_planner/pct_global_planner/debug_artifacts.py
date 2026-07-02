import csv
import math
from pathlib import Path
from typing import Optional

from .path_postprocessor import (
    PathPostprocessConfig,
    PlanningSafetyMap,
    Point3,
    _find_workspace_root,
    compute_path_metrics,
)


def ensure_debug_dirs(debug_dir: str) -> tuple[Path, Path]:
    root = Path(debug_dir)
    if not root.is_absolute():
        root = _find_workspace_root() / root
    screenshots = root / "screenshots"
    root.mkdir(parents=True, exist_ok=True)
    screenshots.mkdir(parents=True, exist_ok=True)
    return root, screenshots


def write_path_csv(path: Path, points: list[Point3]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["index", "x", "y", "z"])
        for index, point in enumerate(points):
            writer.writerow([index, "%.6f" % point[0], "%.6f" % point[1], "%.6f" % point[2]])


def write_clearance_csv(
    path: Path,
    points: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "index",
                "x",
                "y",
                "z",
                "clearance",
                "risk_cost",
                "required_clearance",
                "narrow_passage",
                "stair_or_drop_edge",
            ]
        )
        for index, point in enumerate(points):
            if safety_map is not None and safety_map.loaded:
                clearance = safety_map.query_obstacle_distance(point)
                risk = safety_map.compute_clearance_cost(point)
                required = safety_map.required_clearance(point)
                narrow = safety_map.is_narrow_passage(point)
                stair = safety_map.is_stair_or_drop_edge(point)
            else:
                clearance = math.inf
                risk = 0.0
                required = cfg.hard_min_clearance
                narrow = False
                stair = False
            writer.writerow(
                [
                    index,
                    "%.6f" % point[0],
                    "%.6f" % point[1],
                    "%.6f" % point[2],
                    "%.6f" % clearance if math.isfinite(clearance) else "inf",
                    "%.6f" % risk,
                    "%.6f" % required,
                    int(narrow),
                    int(stair),
                ]
            )


def save_debug_artifacts(
    raw_path: list[Point3],
    optimized_path: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
    debug_dir: str,
    render_images: bool,
    logger=None,
) -> None:
    root, screenshots = ensure_debug_dirs(debug_dir)
    write_path_csv(root / "latest_raw_path.csv", raw_path)
    write_path_csv(root / "latest_optimized_path.csv", optimized_path)
    write_clearance_csv(root / "latest_clearance.csv", optimized_path, safety_map, cfg)

    if not render_images:
        return

    try:
        render_debug_images(raw_path, optimized_path, safety_map, cfg, screenshots)
    except Exception as exc:  # noqa: BLE001 - debug rendering must not fail planning.
        if logger:
            logger.warning(f"Failed to render planning debug PNGs: {exc}")


def render_debug_images(
    raw_path: list[Point3],
    optimized_path: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
    screenshots_dir: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    screenshots_dir.mkdir(parents=True, exist_ok=True)
    obstacle_points = (
        safety_map.obstacle_points if safety_map is not None and safety_map.loaded else np.empty((0, 3))
    )
    if len(obstacle_points) > 20000:
        obstacle_points = obstacle_points[:: int(math.ceil(len(obstacle_points) / 20000))]

    def setup_ax(title: str):
        fig, ax = plt.subplots(figsize=(10, 8), dpi=140)
        ax.set_title(title)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="box")
        if len(obstacle_points):
            ax.scatter(obstacle_points[:, 0], obstacle_points[:, 1], s=0.4, c="#444444", alpha=0.25)
        return fig, ax

    def plot_path(ax, points: list[Point3], color: str, label: str, linewidth: float = 2.0):
        if not points:
            return False
        arr = np.asarray(points)
        ax.plot(arr[:, 0], arr[:, 1], color=color, linewidth=linewidth, label=label)
        ax.scatter(arr[:1, 0], arr[:1, 1], c="#2ca02c", s=35, label=f"{label} start")
        ax.scatter(arr[-1:, 0], arr[-1:, 1], c="#d62728", s=35, label=f"{label} goal")
        return True

    fig, ax = setup_ax("01 original global path")
    if plot_path(ax, raw_path, "#1f77b4", "raw"):
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots_dir / "01_original_global_path.png")
    plt.close(fig)

    fig, ax = setup_ax("02 clearance cost map")
    if safety_map is not None and safety_map.loaded and len(obstacle_points):
        samples = safety_map.sample_clearance_points(3500)
        if samples:
            pts = np.asarray([item[0] for item in samples])
            values = np.asarray([min(cfg.preferred_clearance, item[1]) for item in samples])
            sc = ax.scatter(pts[:, 0], pts[:, 1], c=values, s=3, cmap="RdYlGn", alpha=0.85)
            fig.colorbar(sc, ax=ax, label="clearance [m]")
    if plot_path(ax, optimized_path, "#00a65a", "optimized", 1.6):
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots_dir / "02_clearance_cost_map.png")
    plt.close(fig)

    fig, ax = setup_ax("03 optimized global path")
    if plot_path(ax, optimized_path, "#00a65a", "optimized"):
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots_dir / "03_optimized_global_path.png")
    plt.close(fig)

    fig, ax = setup_ax("04 local planner debug overview")
    has_path = plot_path(ax, optimized_path, "#00a65a", "global path for local tracker")
    ax.text(
        0.02,
        0.98,
        "Runtime local debug topics:\n"
        "/ego_local_trajectory_marker\n"
        "/ego_candidate_trajectories_marker\n"
        "/ego_local_target_marker\n"
        "/ego_local_map_marker\n"
        "/ego_collision_points_marker\n"
        "/ego_footprint_marker\n"
        "/ego_cmd_vel_marker",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "#888888"},
    )
    if has_path:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots_dir / "04_local_planner_debug.png")
    plt.close(fig)

    fig, ax = setup_ax("05 before after comparison")
    has_raw = plot_path(ax, raw_path, "#1f77b4", "raw", 1.4)
    has_optimized = plot_path(ax, optimized_path, "#00a65a", "optimized", 2.0)
    before = compute_path_metrics(raw_path, safety_map, cfg)
    after = compute_path_metrics(optimized_path, safety_map, cfg)
    ax.text(
        0.02,
        0.98,
        "min clearance: %.2f -> %.2f m\navg clearance: %.2f -> %.2f m\nlength: %.2f -> %.2f m"
        % (
            before.min_clearance,
            after.min_clearance,
            before.avg_clearance,
            after.avg_clearance,
            before.path_length,
            after.path_length,
        ),
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "#888888"},
    )
    if has_raw or has_optimized:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(screenshots_dir / "05_before_after_comparison.png")
    plt.close(fig)
