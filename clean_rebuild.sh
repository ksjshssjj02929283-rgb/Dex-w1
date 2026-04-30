#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

DEFAULT_PACKAGES=(robot_server_lib sim_server w1_server)
PACKAGES=("$@")
if [ "${#PACKAGES[@]}" -eq 0 ]; then
  PACKAGES=("${DEFAULT_PACKAGES[@]}")
fi

remove_path_entry() {
  local value="$1"
  local drop="$2"
  printf '%s' "${value}" | awk -v drop="${drop}" '
    BEGIN { RS = ":"; first = 1 }
    $0 != "" && $0 != drop {
      if (!first) {
        printf ":"
      }
      printf "%s", $0
      first = 0
    }
  '
}

echo "Workspace: ${WORKSPACE_DIR}"
echo "Repository: ${SCRIPT_DIR}"
echo "Packages to clean: ${PACKAGES[*]}"
echo

cd "${WORKSPACE_DIR}"

echo "Directories that will be removed:"
for pkg in "${PACKAGES[@]}"; do
  echo "=== ${pkg} ==="
  find build install log -maxdepth 3 \( \
    -path "build/${pkg}" -o \
    -path "install/${pkg}" -o \
    -path "log/latest_build/${pkg}" -o \
    -path "log/latest/${pkg}" -o \
    -path "log/*/${pkg}" \
  \) -print 2>/dev/null || true
done
echo

if [ "${CONFIRM_CLEAN:-}" != "1" ]; then
  read -r -p "Remove only the directories listed above and rebuild? [y/N] " answer
  case "${answer}" in
    y|Y|yes|YES) ;;
    *)
      echo "Canceled."
      exit 0
      ;;
  esac
fi

echo "Cleaning selected package artifacts..."
for pkg in "${PACKAGES[@]}"; do
  rm -rf "build/${pkg}" "install/${pkg}"
  find log -maxdepth 3 \( \
    -path "log/latest_build/${pkg}" -o \
    -path "log/latest/${pkg}" -o \
    -path "log/*/${pkg}" \
  \) -exec rm -rf {} + 2>/dev/null || true
done

echo
echo "Removing stale package paths from this script process..."
for pkg in "${PACKAGES[@]}"; do
  if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
    AMENT_PREFIX_PATH=$(remove_path_entry "${AMENT_PREFIX_PATH}" "${WORKSPACE_DIR}/install/${pkg}")
  fi
  if [ -n "${CMAKE_PREFIX_PATH:-}" ]; then
    CMAKE_PREFIX_PATH=$(remove_path_entry "${CMAKE_PREFIX_PATH}" "${WORKSPACE_DIR}/install/${pkg}")
  fi
done
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"

echo "Sourcing ROS Humble..."
source /opt/ros/humble/setup.bash

echo
echo "Running build.sh..."
cd "${SCRIPT_DIR}"
"${SCRIPT_DIR}/build.sh"

echo
echo "Refreshing dynamic linker cache..."
sudo ldconfig

echo
echo "Done."
echo "To refresh your current terminal environment, run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo
echo "For deployed runtime environment, run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /opt/w1_server/setup.bash"
