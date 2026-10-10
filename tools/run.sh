#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ancestor="$repo_dir"
while [[ "$ancestor" != / && "$(basename "$ancestor")" != src ]]; do
  ancestor="$(dirname "$ancestor")"
done
[[ "$ancestor" != / ]] || { echo 'Repository must be below <workspace>/src.' >&2; exit 1; }
workspace_dir="$(dirname "$ancestor")"
mode=patrol
gui=true
rviz=true
case "${1:-}" in
  --headless) gui=false; rviz=false; shift ;;
  --manual) mode=robot; rviz=false; shift ;;
  --navigation) mode=navigation; shift ;;
  -h|--help)
    echo 'Usage: bash tools/run.sh [--headless|--manual|--navigation] [roslaunch arguments]'
    echo 'Default: complete patrol with Gazebo + RViz. Ctrl-C stops the session.'
    exit 0 ;;
esac
[[ -f "$workspace_dir/devel/setup.bash" && -x "$workspace_dir/.venv/bin/python3" ]] || {
  echo 'Run bash tools/setup.sh first.' >&2; exit 1;
}
if [[ "$gui" == true && -z "${DISPLAY:-}" ]]; then
  echo 'No DISPLAY. Use a desktop terminal or bash tools/run.sh --headless.' >&2
  exit 1
fi
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
set +u
source /opt/ros/noetic/setup.bash
source "$workspace_dir/.venv/bin/activate"
source "$workspace_dir/devel/setup.bash"
set -u
export PYTHONNOUSERSITE=1
python3 "$repo_dir/tools/check_environment.py" --quiet
if [[ "$mode" == robot ]]; then
  exec roslaunch --no-summary smart_community_sim robot.launch "gui:=$gui" "$@"
fi
exec roslaunch --no-summary smart_community_sim "$mode.launch" "gui:=$gui" "rviz:=$rviz" "$@"
