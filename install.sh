#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="${SCRIPT_DIR}"

ROS_DISTRO="${ROS_DISTRO:-noetic}"
WORKSPACE=""
SKIP_APT=0
SKIP_ROSDEP_INIT=0

usage() {
  cat << 'USAGE'
Usage: ./install.sh [options]

Options:
  --rosdistro <name>      ROS distro to use (default: noetic or $ROS_DISTRO)
  --workspace <path>      Catkin workspace root path (auto-detect by default)
  --skip-apt              Skip apt dependency installation
  --skip-rosdep-init      Skip rosdep init step
  -h, --help              Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rosdistro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --skip-rosdep-init)
      SKIP_ROSDEP_INIT=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${WORKSPACE}" ]]; then
  PARENT_NAME="$(basename "$(dirname "${PKG_DIR}")")"
  if [[ "${PARENT_NAME}" == "src" ]]; then
    WORKSPACE="$(dirname "$(dirname "${PKG_DIR}")")"
  else
    WORKSPACE="${PKG_DIR}"
  fi
fi

if [[ ! -d "${WORKSPACE}" ]]; then
  echo "[ERROR] Workspace path does not exist: ${WORKSPACE}"
  exit 1
fi

if [[ "$(basename "${WORKSPACE}")" == "src" ]]; then
  echo "[ERROR] --workspace should be the catkin root, not src/"
  echo "        Example: --workspace ~/catkin_ws"
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[ERROR] /opt/ros/${ROS_DISTRO}/setup.bash not found."
  echo "        Please install ROS ${ROS_DISTRO} first or set --rosdistro correctly."
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

SUDO=""
if [[ ${EUID} -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
  else
    echo "[ERROR] sudo is required for apt/rosdep init when not running as root."
    exit 1
  fi
fi

echo "[INFO] Package dir: ${PKG_DIR}"
echo "[INFO] Workspace : ${WORKSPACE}"
echo "[INFO] ROS_DISTRO: ${ROS_DISTRO}"

if [[ ${SKIP_APT} -eq 0 ]]; then
  echo "[INFO] Installing base tools via apt..."
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y \
    python3-rosdep \
    python3-rospkg \
    python3-yaml \
    build-essential
else
  echo "[INFO] Skip apt step by user request (--skip-apt)."
fi

if [[ ${SKIP_ROSDEP_INIT} -eq 0 ]]; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo "[INFO] Initializing rosdep..."
    ${SUDO} rosdep init
  else
    echo "[INFO] rosdep already initialized."
  fi
else
  echo "[INFO] Skip rosdep init by user request (--skip-rosdep-init)."
fi

echo "[INFO] Updating rosdep index..."
rosdep update

FROM_PATHS="${PKG_DIR}"
if [[ -d "${WORKSPACE}/src" ]]; then
  FROM_PATHS="${WORKSPACE}/src"
fi

echo "[INFO] Installing ROS package dependencies with rosdep..."
rosdep install \
  --from-paths "${FROM_PATHS}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  -r -y

echo "[INFO] Dependency installation finished successfully."
echo "[INFO] Next steps:"
echo "       cd ${WORKSPACE}"
echo "       catkin_make"
echo "       source devel/setup.bash"
