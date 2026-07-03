#!/usr/bin/env python3
import argparse
import json
import os
import re
from collections import Counter
from pathlib import Path


EVENTS = (
    "PLAN_SUCCESS",
    "PLAN_FAILED",
    "REPLAN",
    "TF_ERROR",
    "CLOUD_TIMEOUT",
    "BLOCKED",
    "GOAL_REACHED",
)


def workspace_root() -> Path:
    env_value = os.environ.get("NAV3D_WS", "")
    if env_value:
        return Path(env_value).expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def latest_log_file() -> Path:
    log_dir = workspace_root() / "debug" / "logs" / "local_planner"
    candidates = []
    for pattern in ("local_planner_*.log", "local_planner_*.jsonl", "local_planner_*.csv"):
        candidates.extend(log_dir.glob(pattern))
    if not candidates:
        raise FileNotFoundError(f"no local planner logs found in {log_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def event_from_line(line: str) -> str:
    line = line.strip()
    if not line:
        return ""
    if line.startswith("{"):
        try:
            return str(json.loads(line).get("event", ""))
        except json.JSONDecodeError:
            return ""
    matches = re.findall(r"\[([A-Z0-9_]+)\]", line)
    if matches:
        return matches[-1]
    parts = line.split(",", 3)
    if len(parts) >= 3:
        return parts[2].strip().strip('"')
    return ""


def message_from_line(line: str) -> str:
    line = line.strip()
    if line.startswith("{"):
        try:
            return str(json.loads(line).get("message", ""))
        except json.JSONDecodeError:
            return line
    parts = line.split("] ", 1)
    if len(parts) == 2:
        return parts[1]
    csv_parts = line.split(",", 3)
    if len(csv_parts) == 4:
        return csv_parts[3].strip().strip('"')
    return line


def count_event(counter: Counter, event: str, message: str) -> None:
    if event == "REPLAN" or "REPLAN_TRIGGERED" in message:
        counter["REPLAN"] += 1
        return
    if event in counter:
        counter[event] += 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze local planner log statistics.")
    parser.add_argument("log_file", nargs="?", type=Path, help="log file to analyze")
    args = parser.parse_args()

    log_file = args.log_file or latest_log_file()
    counter = Counter({event: 0 for event in EVENTS})
    failure_reasons = Counter()
    planning_times = []
    min_obstacle_distances = []

    with log_file.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            event = event_from_line(line)
            message = message_from_line(line)
            count_event(counter, event, message)

            reason_match = re.search(r'failure_reason="([^"]*)"', message)
            if reason_match:
                failure_reasons[reason_match.group(1) or "unknown"] += 1

            time_match = re.search(r"planning_time_ms=([-+]?[0-9]*\.?[0-9]+)", message)
            if time_match:
                planning_times.append(float(time_match.group(1)))

            distance_match = re.search(
                r"min_obstacle_distance=([-+]?[0-9]*\.?[0-9]+)", message
            )
            if distance_match:
                value = float(distance_match.group(1))
                if value >= 0.0:
                    min_obstacle_distances.append(value)

    print(f"log_file: {log_file}")
    for event in EVENTS:
        print(f"{event}: {counter[event]}")

    if failure_reasons:
        print("failure_reasons:")
        for reason, count in failure_reasons.most_common():
            print(f"  {reason}: {count}")
    else:
        print("failure_reasons: none")

    if planning_times:
        average = sum(planning_times) / len(planning_times)
        print(f"average_planning_time_ms: {average:.3f}")
    else:
        print("average_planning_time_ms: n/a")

    if min_obstacle_distances:
        print(f"minimum_obstacle_distance: {min(min_obstacle_distances):.3f}")
    else:
        print("minimum_obstacle_distance: n/a")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
