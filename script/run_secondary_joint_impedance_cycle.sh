#!/usr/bin/env bash

set -u
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CFG_PATH="${SCRIPT_DIR}/../w1_server/robot.cfg"

ACTION_NAME="/control/move_robot"
ACTION_TYPE="robot_server_interfaces/action/MoveRobot"
SET_PROPERTY_SERVICE="/control/set_property"
SET_PROPERTY_TYPE="robot_server_interfaces/srv/SetProperty"
MOTION_GROUP="Secondary"
MOVE_CONFIG='{"speed":0.5,"acceleration":0.5}'
MOVE_SPEED="$(python3 - <<'PY'
import json
cfg = json.loads('''{"speed":0.5,"acceleration":0.5}''')
print(cfg["speed"])
PY
)"
MOVE_ACCELERATION="$(python3 - <<'PY'
import json
cfg = json.loads('''{"speed":0.5,"acceleration":0.5}''')
print(cfg["acceleration"])
PY
)"
LOG_PATH="${HOME}/log/robot_record.jsonl"
FINAL_TARGET="0,1.58,0,0,0,0,0"
REPORT_DIR="${HOME}/dexe_robot_server_data"
REPORT_TS="$(date +%Y%m%d_%H%M%S)"
REPORT_PATH="${REPORT_DIR}/secondary_joint_impedance_cycle_${REPORT_TS}.md"
RUN_MODE_VALUE="1"
ARM_LABEL=""
ARM_STIFFNESS_LINE=""
SUMMARY_ROWS=""

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
  local goal="{motion_group: ${MOTION_GROUP}, type: MoveJoint, target: [${target_csv}], config: '${MOVE_CONFIG}'}"
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
  local logical_joint="$2"
  local start_ts
  local end_ts
  shift
  shift
  local -a pair=("$@")

  start_ts="$(date +%s.%N)"

  for round in 1 2 3; do
    print_step "${group_name} round ${round}/3"
    for target in "${pair[@]}"; do
      run_move_joint "${target}" || return 1
    done
  done

  end_ts="$(date +%s.%N)"
  summarize_group_error "${group_name}" "${logical_joint}" "${start_ts}" "${end_ts}" || return 1
}

summarize_group_error() {
  local group_name="$1"
  local logical_joint="$2"
  local start_ts="$3"
  local end_ts="$4"
  local summary_line

  print_step "${group_name} error summary"
  summary_line="$(python3 - "${LOG_PATH}" "${group_name}" "${logical_joint}" "${start_ts}" "${end_ts}" "${MOTION_GROUP}" <<'PY'
import json
import math
import sys
from pathlib import Path

log_path = Path(sys.argv[1]).expanduser()
group_name = sys.argv[2]
logical_joint = int(sys.argv[3])
start_ts = float(sys.argv[4])
end_ts = float(sys.argv[5])
motion_group = sys.argv[6]

if not log_path.exists():
    print(f"[ERROR] Log file not found: {log_path}")
    sys.exit(1)

def map_joint_index(joint_count: int, logical_joint_num: int) -> int:
    if joint_count == 14:
        if motion_group == "Primary":
            return logical_joint_num - 1
        if motion_group == "Secondary":
            return 6 + logical_joint_num
        raise ValueError(f"Unsupported motion_group: {motion_group}")
    if joint_count == 20:
        if motion_group == "Primary":
            return 5 + logical_joint_num
        if motion_group == "Secondary":
            return 12 + logical_joint_num
        raise ValueError(f"Unsupported motion_group: {motion_group}")
    raise ValueError(f"Unsupported joint_count: {joint_count}")

errors_deg = []
joint_index = None

with log_path.open("r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            data = json.loads(line)
        except Exception:
            continue

        try:
            ts = float(data["timestamp"])
        except Exception:
            continue

        if ts < start_ts or ts > end_ts:
            continue

        pos = data.get("joint_position")
        pos_cmd = data.get("joint_position_cmd")
        if not isinstance(pos, list) or not isinstance(pos_cmd, list) or len(pos) != len(pos_cmd):
            continue

        if joint_index is None:
            joint_index = map_joint_index(len(pos), logical_joint)

        if joint_index >= len(pos):
            print(f"[ERROR] Joint index {joint_index} out of range for joint_count {len(pos)}")
            sys.exit(1)

        try:
            err_deg = math.degrees(float(pos_cmd[joint_index]) - float(pos[joint_index]))
        except Exception:
            continue
        if math.isfinite(err_deg):
            errors_deg.append(abs(err_deg))

if not errors_deg:
    print(f"[ERROR] No valid error samples found for {group_name}")
    sys.exit(1)

avg_err = sum(errors_deg) / len(errors_deg)
max_err = max(errors_deg)
print(
    f"{group_name}|{motion_group} joint {logical_joint}|{len(errors_deg)}|{avg_err:.6f}|{max_err:.6f}"
)
PY
)" || return 1

  IFS='|' read -r group_label joint_label sample_count avg_err max_err <<< "${summary_line}"
  echo "${group_label}: ${joint_label}, samples=${sample_count}, avg_abs_error_deg=${avg_err}, max_abs_error_deg=${max_err}"
  SUMMARY_ROWS+=$'| '"${group_label}"$' | '"${joint_label}"$' | '"${sample_count}"$' | '"${avg_err}"$' | '"${max_err}"$' |\n'
}

write_report() {
  mkdir -p "${REPORT_DIR}" || return 1
  ARM_STIFFNESS_LINE="$(python3 - "${ROBOT_CFG_PATH}" "${MOTION_GROUP}" <<'PY'
import json
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1])
motion_group = sys.argv[2]
if not cfg_path.exists():
    print("unavailable (robot.cfg not found)")
    sys.exit(0)

try:
    data = json.loads(cfg_path.read_text(encoding="utf-8"))
    stiffness = data.get("joint_stiffness", [])
    if not isinstance(stiffness, list) or len(stiffness) < 20:
        print("unavailable (joint_stiffness invalid)")
        sys.exit(0)
    if motion_group == "Primary":
        arm_stiffness = stiffness[6:13]
    elif motion_group == "Secondary":
        arm_stiffness = stiffness[13:20]
    else:
        print(f"unavailable (unsupported motion_group: {motion_group})")
        sys.exit(0)
    print("[" + ", ".join(str(v) for v in arm_stiffness) + "]")
except Exception as exc:
    print(f"unavailable ({exc})")
PY
)"

  if [[ "${MOTION_GROUP}" == "Primary" ]]; then
    ARM_LABEL="Left"
  else
    ARM_LABEL="Right"
  fi

  cat > "${REPORT_PATH}" <<EOF
# ${MOTION_GROUP} Joint Impedance Cycle Report

- Timestamp: ${REPORT_TS}
- SwitchRunMode: ${RUN_MODE_VALUE}
- Motion group: ${MOTION_GROUP}
- Move type: MoveJoint
- Speed: ${MOVE_SPEED}
- Acceleration: ${MOVE_ACCELERATION}
- Final target: [${FINAL_TARGET}]
- ${ARM_LABEL} arm joint stiffness from ${ROBOT_CFG_PATH}: ${ARM_STIFFNESS_LINE}

## Error Summary

| Group | Joint | Samples | Average Abs Error (deg) | Max Abs Error (deg) |
| --- | --- | ---: | ---: | ---: |
${SUMMARY_ROWS}
EOF

  echo
  echo "Report saved to: ${REPORT_PATH}"
}

main() {
  print_step "Enable joint impedance"
  send_set_property "SwitchRunMode" "${RUN_MODE_VALUE}" || exit 1

  run_group_three_times "Group 1" "1" "${GROUP_1[@]}" || exit 1
  run_group_three_times "Group 2" "2" "${GROUP_2[@]}" || exit 1
  run_group_three_times "Group 3" "4" "${GROUP_3[@]}" || exit 1
  run_group_three_times "Group 4" "6" "${GROUP_4[@]}" || exit 1

  print_step "Return to final target"
  run_move_joint "${FINAL_TARGET}" || exit 1

  write_report || exit 1
}

main "$@"
