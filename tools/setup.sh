#!/usr/bin/env bash
# Run from a checkout below a catkin workspace's src directory.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
skip_apt=false
jobs=2
while (($#)); do
  case "$1" in
    --skip-apt) skip_apt=true; shift ;;
    --jobs) jobs="${2:?--jobs needs a number}"; shift 2 ;;
    -h|--help)
      echo 'Usage: bash tools/setup.sh [--skip-apt] [--jobs 2]'
      echo 'Ubuntu 20.04: install ROS/system dependencies, create a venv, build, check.'
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'Invalid --jobs' >&2; exit 2; }
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 20.04 ]]; then
  echo 'This setup is for Ubuntu 20.04 / ROS Noetic. Use an Ubuntu 20.04 VM on other systems.' >&2
  exit 1
fi
ancestor="$repo_dir"
while [[ "$ancestor" != / && "$(basename "$ancestor")" != src ]]; do
  ancestor="$(dirname "$ancestor")"
done
[[ "$ancestor" != / ]] || { echo 'Place this repository in <workspace>/src/smart-community first.' >&2; exit 1; }
workspace_dir="$(dirname "$ancestor")"
if ! "$skip_apt"; then
  admin=()
  if ((EUID != 0)); then admin=(sudo); fi
  "${admin[@]}" apt-get update
  "${admin[@]}" apt-get install -y ca-certificates curl gnupg software-properties-common
  "${admin[@]}" add-apt-repository -y universe
  # Preserve an existing ROS/mirror repository. For a fresh machine, use the
  # official ROS signing key and a dedicated signed-by source (no apt-key).
  if ! grep -qsE 'ros/ubuntu' /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; then
    key_file="$(mktemp)"
    curl --fail --location --retry 3 --connect-timeout 20 \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$key_file"
    "${admin[@]}" install -m 0644 "$key_file" /usr/share/keyrings/smart-community-ros.gpg
    rm -f "$key_file"
    printf 'deb [signed-by=/usr/share/keyrings/smart-community-ros.gpg] http://packages.ros.org/ros/ubuntu focal main\n' | \
      "${admin[@]}" tee /etc/apt/sources.list.d/smart-community-ros.list >/dev/null
  fi
  "${admin[@]}" apt-get update
  "${admin[@]}" apt-get install -y \
    build-essential cmake git python3-dev python3-venv python3-pip \
    python3-numpy python3-opencv python3-pil python3-yaml python3-rospkg \
    fonts-noto-cjk gazebo11 libgazebo11-dev \
    ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-dev \
    ros-noetic-xacro ros-noetic-robot-state-publisher ros-noetic-rviz \
    ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-slam-gmapping \
    ros-noetic-cv-bridge ros-noetic-tf2-geometry-msgs
fi
[[ -f /opt/ros/noetic/setup.bash ]] || { echo 'ROS Noetic is missing. Rerun without --skip-apt.' >&2; exit 1; }
venv_dir="$workspace_dir/.venv"
/usr/bin/python3 -m venv --system-site-packages --without-pip "$venv_dir"
# Use apt's pip through python -m pip; this also works without ensurepip.
# Packages are installed into sys.prefix (the venv), not into system Python.
"$venv_dir/bin/python3" -m pip install --disable-pip-version-check --no-deps \
  --ignore-installed --only-binary=:all: -r "$repo_dir/smart_community_sim/requirements-vision.txt"
# Build against ROS itself, rather than inheriting another user's workspace.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
set +u
source /opt/ros/noetic/setup.bash
source "$venv_dir/bin/activate"
set -u
export PYTHONNOUSERSITE=1
catkin_make -C "$workspace_dir" -j"$jobs" -l"$jobs" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE="$venv_dir/bin/python3"
set +u
source "$workspace_dir/devel/setup.bash"
set -u
"$venv_dir/bin/python3" "$repo_dir/tools/check_environment.py" --inference
echo "Ready. Run: bash '$repo_dir/tools/run.sh'"
