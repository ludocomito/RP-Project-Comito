#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/.." && pwd)"

source /opt/ros/humble/setup.bash

if [[ -n "${COURSE_WS:-}" ]]; then
  source "${COURSE_WS}/install/setup.bash"
elif [[ -f "${repo_dir}/../install/setup.bash" ]]; then
  source "${repo_dir}/../install/setup.bash"
fi

source "${repo_dir}/install/local_setup.bash"

# If another workspace contains a package with the same name, keep this
# repository's install prefix first for ament package lookup.
current_prefix="${repo_dir}/install/rp_simple_rviz"
filtered_prefixes="$(
  printf '%s' "${AMENT_PREFIX_PATH:-}" |
    tr ':' '\n' |
    grep -v -F "${current_prefix}" |
    paste -sd: -
)"
if [[ -n "${filtered_prefixes}" ]]; then
  export AMENT_PREFIX_PATH="${current_prefix}:${filtered_prefixes}"
else
  export AMENT_PREFIX_PATH="${current_prefix}"
fi

echo "rp_simple_rviz: $(ros2 pkg prefix rp_simple_rviz)"
echo "rp_simulator: $(ros2 pkg prefix rp_simulator)"

exec ros2 launch rp_simple_rviz with_simulator.launch.py "$@"
