import math
from dataclasses import dataclass


Point3 = tuple[float, float, float]


@dataclass
class PathPostprocessConfig:
    enable_path_smoothing: bool = True
    enable_path_resampling: bool = True
    path_resample_resolution: float = 0.2
    max_path_z_jump: float = 0.4
    remove_duplicate_points: bool = True
    duplicate_epsilon: float = 1.0e-4
    smoothing_passes: int = 1


def distance(a: Point3, b: Point3) -> float:
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )


def path_length(points: list[Point3]) -> float:
    return sum(distance(points[i], points[i + 1]) for i in range(len(points) - 1))


def remove_duplicates(points: list[Point3], epsilon: float = 1.0e-4) -> list[Point3]:
    if not points:
        return []

    out = [points[0]]
    for point in points[1:]:
        if distance(out[-1], point) > epsilon:
            out.append(point)
    return out


def clamp_z_jumps(points: list[Point3], max_jump: float) -> list[Point3]:
    if len(points) < 2 or max_jump <= 0.0:
        return points[:]

    out = [points[0]]
    for point in points[1:]:
        previous = out[-1]
        dz = abs(point[2] - previous[2])
        steps = max(1, int(math.ceil(dz / max_jump)))
        for step in range(1, steps + 1):
            ratio = step / float(steps)
            out.append(
                (
                    previous[0] + (point[0] - previous[0]) * ratio,
                    previous[1] + (point[1] - previous[1]) * ratio,
                    previous[2] + (point[2] - previous[2]) * ratio,
                )
            )
    return out


def resample_path(points: list[Point3], resolution: float) -> list[Point3]:
    if len(points) < 2 or resolution <= 0.0:
        return points[:]

    out = [points[0]]
    current = points[0]
    remaining = resolution

    for target in points[1:]:
        segment_length = distance(current, target)
        if segment_length <= 1.0e-9:
            current = target
            continue

        while segment_length >= remaining:
            ratio = remaining / segment_length
            next_point = (
                current[0] + (target[0] - current[0]) * ratio,
                current[1] + (target[1] - current[1]) * ratio,
                current[2] + (target[2] - current[2]) * ratio,
            )
            out.append(next_point)
            current = next_point
            segment_length = distance(current, target)
            remaining = resolution

        remaining -= segment_length
        current = target

    if distance(out[-1], points[-1]) > 1.0e-6:
        out.append(points[-1])
    return out


def smooth_path(points: list[Point3], passes: int = 1) -> list[Point3]:
    if len(points) < 3 or passes <= 0:
        return points[:]

    smoothed = points[:]
    for _ in range(passes):
        next_points = [smoothed[0]]
        for index in range(1, len(smoothed) - 1):
            prev_p = smoothed[index - 1]
            cur_p = smoothed[index]
            next_p = smoothed[index + 1]
            next_points.append(
                (
                    0.25 * prev_p[0] + 0.50 * cur_p[0] + 0.25 * next_p[0],
                    0.25 * prev_p[1] + 0.50 * cur_p[1] + 0.25 * next_p[1],
                    0.25 * prev_p[2] + 0.50 * cur_p[2] + 0.25 * next_p[2],
                )
            )
        next_points.append(smoothed[-1])
        smoothed = next_points
    return smoothed


def postprocess_path(points: list[Point3], cfg: PathPostprocessConfig) -> list[Point3]:
    processed = points[:]

    if cfg.remove_duplicate_points:
        processed = remove_duplicates(processed, cfg.duplicate_epsilon)

    processed = clamp_z_jumps(processed, cfg.max_path_z_jump)

    if cfg.enable_path_resampling:
        processed = resample_path(processed, cfg.path_resample_resolution)

    if cfg.enable_path_smoothing:
        processed = smooth_path(processed, cfg.smoothing_passes)
        processed = clamp_z_jumps(processed, cfg.max_path_z_jump)

    if cfg.remove_duplicate_points:
        processed = remove_duplicates(processed, cfg.duplicate_epsilon)

    return processed
