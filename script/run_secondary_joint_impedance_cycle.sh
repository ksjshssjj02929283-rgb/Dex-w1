#!/usr/bin/env bash

set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ACTION_NAME="/control/move_robot"
ACTION_TYPE="robot_server_interfaces/action/MoveRobot"
SET_PROPERTY_SERVICE="/control/set_property"
SET_PROPERTY_TYPE="robot_server_interfaces/srv/SetProperty"
MOVE_CONFIG='{"speed":0.1,"acceleration":0.5}'

declare -a GROUP_1=(
  "0.78,1.58,0,0,0,0,0"
  "1.58,1.58,0,0,0,0,0"
)
declare -a GROUP_2=(
  "0,0.78,0,0,0,0,0"
  "0,0,0,0,0,0,0"
)
declare -a GROUP_3=(
  "0,1.58,0,0,0,0,0"
  "0,1.58,0,1.58,0,0,0"
)
declare -a GROUP_4=(
  "0,1.58,0,1.58,0,-0.64,0"
  "0,1.58,0,1.58,0,0.64,0"
)

print_step() {
  echo
  echo "==== $1 ===="
}

send_set_property() {
  local property_type="$1"
  local value="$2"
  local request="{property_type: '${property_type}', value: '${value}'}"

  echo "Setting ${property_type} -> ${value}"
  ros2 service call "${SET_PROPERTY_SERVICE}" "${SET_PROPERTY_TYPE}" "${request}" >/tmp/robot_set_property.out 2>&1
  local rc=$?
  cat /tmp/robot_set_property.out
  if [[ ${rc} -ne 0 ]]; then
    echo "[ERROR] Failed to call set_property for ${property_type}"
    return 1
  fi
  return 0
}

run_move_joint() {
  local target_csv="$1"
  local goal="{motion_group: Secondary, type: MoveJoint, target: [${target_csv}], config: '${MOVE_CONFIG}'}"
  local output_file
  output_file="$(mktemp)"

  echo "Sending goal: [${target_csv}]"
  ros2 action send_goal "${ACTION_NAME}" "${ACTION_TYPE}" "${goal}" --feedback >"${output_file}" 2>&1
  local rc=$?
  cat "${output_file}"

  if [[ ${rc} -ne 0 ]]; then
    echo "[ERROR] ros2 action send_goal exited with code ${rc}"
    rm -f "${output_file}"
    return 1
  fi

  if grep -Eq "ABORTED|CANCELED|CANCELLED|REJECTED|STATUS_ABORTED|STATUS_CANCELED|STATUS_UNKNOWN|ErrorDrop" "${output_file}"; then
    echo "[ERROR] Non-success result detected for goal [${target_csv}]"
    rm -f "${output_file}"
    return 1
  fi

  if ! grep -Eqi "SUCCEEDED|STATUS_SUCCEEDED|Goal finished.*succeed" "${output_file}"; then
    echo "[ERROR] Could not confirm success result for goal [${target_csv}]"
    rm -f "${output_file}"
    return 1
  fi

  rm -f "${output_file}"
  return 0
}

run_group_three_times() {
  local group_name="$1"
  shift
  local -a pair=("$@")

  for round in 1 2 3; do
    print_step "${group_name} round ${round}/3"
    for target in "${pair[@]}"; do
      run_move_joint "${target}" || return 1
    done
  done
}

plot_right_arm_selected_joints() {
  print_step "Plot right arm joints 1/2/4/6"
  (
    cd "${SCRIPT_DIR}" || exit 1
    python3 plot_log.py --a1 --joint 7 &
    python3 plot_log.py --a1 --joint 8 &
    python3 plot_log.py --a1 --joint 10 &
    python3 plot_log.py --a1 --joint 12 &
    wait
  )
}

main() {
  print_step "Enable joint impedance"
  send_set_property "SwitchRunMode" "1" || exit 1

  run_group_three_times "Group 1" "${GROUP_1[@]}" || exit 1
  run_group_three_times "Group 2" "${GROUP_2[@]}" || exit 1
  run_group_three_times "Group 3" "${GROUP_3[@]}" || exit 1
  run_group_three_times "Group 4" "${GROUP_4[@]}" || exit 1

  plot_right_arm_selected_joints
}

main "$@"
