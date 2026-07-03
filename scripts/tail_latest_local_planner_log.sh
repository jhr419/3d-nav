#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="${NAV3D_WS:-$(cd "${script_dir}/.." && pwd)}"
log_dir="${workspace_root}/debug/logs/local_planner"

if [[ ! -d "${log_dir}" ]]; then
  echo "local planner log directory does not exist: ${log_dir}" >&2
  exit 1
fi

latest_log="$(
  find "${log_dir}" -maxdepth 1 -type f \
    \( -name 'local_planner_*.log' -o -name 'local_planner_*.jsonl' -o -name 'local_planner_*.csv' \) \
    -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -n 1 | cut -d' ' -f2-
)"

if [[ -z "${latest_log}" ]]; then
  echo "no local planner log files found in ${log_dir}" >&2
  exit 1
fi

echo "tailing ${latest_log}"
tail -f "${latest_log}"
