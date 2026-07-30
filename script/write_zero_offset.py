import subprocess
import time
import yaml
import sys
import os
import re

DEXE_RP_DATA_PATH = os.environ.get('DEXE_RP_DATA_PATH', os.path.expanduser('~/dexe_rp_data'))
CALIBRATION_FILE = os.path.join(DEXE_RP_DATA_PATH, 'w1_calibration.yaml')

def _pulse_to_degree(pulse):
    degree = ((pulse - 262144) / 262144) * 360.0
    return degree

def _parse_ethercat_value(result, label):
    if result.returncode != 0:
        raise RuntimeError(f'{label} 失败 (returncode={result.returncode}): {result.stderr.strip()}')
    parts = result.stdout.split()
    if len(parts) < 2:
        raise RuntimeError(f'{label} 输出无法解析: {result.stdout!r}')
    try:
        return int(parts[1])
    except ValueError:
        raise RuntimeError(f'{label} 输出无法解析: {result.stdout!r}')

def _get_motor_index_and_check_status(expected_count: int) -> list:
    try:
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
    except Exception as e:
        raise RuntimeError(f'执行 "ethercat sl" 失败: {e}')

    driver_positions = []
    non_preop = []

    for line in result.stdout.splitlines():
        if 'ZeroErr Driver' in line:
            m = re.match(r'^\s*(\d+)\s+\S+\s+([A-Z]+)\b.*ZeroErr Driver', line)
            if m:
                idx = int(m.group(1))
                state = m.group(2)
                driver_positions.append(idx)
                if state != 'PREOP':
                    non_preop.append((idx, state))
            else:
                m2 = re.match(r'^\s*(\d+)\b', line)
                if m2:
                    idx = int(m2.group(1))
                    driver_positions.append(idx)
                    non_preop.append((idx, 'UNKNOWN'))

    if len(driver_positions) != expected_count:
        raise RuntimeError(f'期望 {expected_count} 个 ZeroErr Driver，实际为 {len(driver_positions)}: {driver_positions}')

    if non_preop:
        details = ', '.join(f'{idx}:{state}' for idx, state in non_preop)
        raise RuntimeError(f'以下 ZeroErr Driver 非 PREOP 状态: {details}')

    print(f"Motor index: [{', '.join(map(str, driver_positions))}], total {len(driver_positions)} motors, all in PREOP state.")
    return driver_positions

def _get_motor_position_deg_by_index(motor_index_list: list) -> list:
    qpos_list = []
    for motor_index in motor_index_list:
        result = subprocess.run(['ethercat', 'upload', '-p', str(motor_index), '0x6064', '0', '-t', 'uint32'], capture_output=True, text=True)
        pulse = _parse_ethercat_value(result, f'读取电机 {motor_index} 位置')
        qpos_list.append(_pulse_to_degree(pulse))
    return qpos_list

def _write_zero_offset(index_list: list, zero_offset_list: list, joint_ids: list):
    if len(zero_offset_list) != len(joint_ids):
        raise ValueError(f"offset_list 长度应为 {len(joint_ids)}，当前为 {len(zero_offset_list)}")
    pre_offset_list = []
    new_offset_list = []

    for i_local, joint_id in enumerate(joint_ids):
        index = index_list[joint_id]
        offset = zero_offset_list[i_local]

        result = subprocess.run(['ethercat', 'upload', '-p', str(index), '0x3b75', '0', '-t', 'uint32'], capture_output=True, text=True)
        pre_offset = _parse_ethercat_value(result, f'读取电机 {index} 零位偏移')
        pre_offset_list.append(pre_offset)

        new_offset = (pre_offset + round(offset / 360.0 * 262144)) & 0xFFFFFFFF
        new_offset_list.append(new_offset)

        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x3b75', '0', hex(new_offset)], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f'写入电机 {index} 零位偏移失败 (returncode={result.returncode}): {result.stderr.strip()}')

        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x1010', '1', '0x65766173'], capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f'保存电机 {index} 零位偏移失败 (returncode={result.returncode}): {result.stderr.strip()}')

    print(f"pre_offset:", pre_offset_list)
    print(f"new_offset:", new_offset_list)
    time.sleep(2)
    print("Zero offsets updated!")

def set_zero_offset(zero_offsets_deg: list, joint_ids: list, expected_motor_count: int = 20) -> bool:
    motor_index_list = _get_motor_index_and_check_status(expected_motor_count)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) before calibration:", qpos_list)
    _write_zero_offset(motor_index_list, zero_offsets_deg, joint_ids)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) after calibration:", qpos_list)
    return True

def _load_section(section_name: str) -> tuple:
    if not os.path.exists(CALIBRATION_FILE):
        print(f"File not found: {CALIBRATION_FILE}")
        return None, False
    with open(CALIBRATION_FILE, "r") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        print(f"Invalid or empty YAML: {CALIBRATION_FILE}")
        return None, False
    section = data.get(section_name)
    if section is None:
        print(f"Section '{section_name}' not found in {CALIBRATION_FILE}")
        return None, False
    return section.get('zero_offset'), section.get('is_calibrated', False)

def load_zero_offsets(is_left: bool) -> tuple:
    return _load_section('left_arm' if is_left else 'right_arm')

def load_torso_offsets() -> tuple:
    return _load_section('torso_head')

def _update_calibration_status(is_left: bool = None, is_torso: bool = False):
    if not os.path.exists(CALIBRATION_FILE):
        return

    with open(CALIBRATION_FILE, "r") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        return

    if is_torso:
        section_name = 'torso_head'
    else:
        section_name = 'left_arm' if is_left else 'right_arm'

    if section_name not in data:
        return

    data[section_name]['is_calibrated'] = True

    with open(CALIBRATION_FILE, "w") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"Updated {CALIBRATION_FILE} [{section_name}] is_calibrated = true")

def main():
    a1_mode = '--a1' in sys.argv or 'a1' in sys.argv

    if a1_mode:
        print("=== A1 模式 ===")
        # A1 物理顺序可能为: 头部(0-1) -> 左臂(2-8) -> 右臂(9-15)
        # 总电机数预期为 16 (基于头部2 + 左右各7)
        # 如果没有头部，则左臂(0-6), 右臂(7-13), 总数 14
        
        # 自动探测总电机数
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
        motor_count = len(re.findall(r'ZeroErr Driver', result.stdout))
        print(f"Detected {motor_count} ZeroErr Drivers on bus.")

        if motor_count == 16:
            # 头部(0,1) -> 左臂(2-8) -> 右臂(9-15)
            left_ids = list(range(2, 9))
            right_ids = list(range(9, 16))
        else:
            # 左臂(0-6) -> 右臂(7-13)
            left_ids = list(range(7))
            right_ids = list(range(7, 14))

        left_zero_offsets, left_calibrated = load_zero_offsets(is_left=True)
        if left_zero_offsets is not None and not left_calibrated:
            set_zero_offset(zero_offsets_deg=left_zero_offsets, joint_ids=left_ids, expected_motor_count=motor_count)
            _update_calibration_status(is_left=True)
        elif left_zero_offsets is None:
            print("Left arm offset file missing, skipping.")
        else:
            print("Left arm already calibrated, skipping.")

        right_zero_offsets, right_calibrated = load_zero_offsets(is_left=False)
        if right_zero_offsets is not None and not right_calibrated:
            set_zero_offset(zero_offsets_deg=right_zero_offsets, joint_ids=right_ids, expected_motor_count=motor_count)
            _update_calibration_status(is_left=False)
        elif right_zero_offsets is None:
            print("Right arm offset file missing, skipping.")
        else:
            print("Right arm already calibrated, skipping.")
    else:
        print("=== W1 模式 ===")
        # 物理顺序: 躯干(0-5) -> 左臂(6-12) -> 右臂(13-19)
        # 躯干校准 (Joint IDs: 0, 1, 2, 3, 4, 5)
        joint_ids = list(range(6))
        torso_zero_offsets, torso_calibrated = load_torso_offsets()
        if torso_zero_offsets is not None and not torso_calibrated:
            set_zero_offset(zero_offsets_deg=torso_zero_offsets, joint_ids=joint_ids)
            _update_calibration_status(is_torso=True)
        elif torso_zero_offsets is None:
            print("Torso offset file missing, skipping.")
        else:
            print("Torso already calibrated, skipping.")

        # 左臂校准 (Joint IDs: 6, 7, 8, 9, 10, 11, 12)
        joint_ids = list(range(6, 13))
        left_zero_offsets, left_calibrated = load_zero_offsets(is_left=True)
        if left_zero_offsets is not None and not left_calibrated:
            set_zero_offset(zero_offsets_deg=left_zero_offsets, joint_ids=joint_ids)
            _update_calibration_status(is_left=True)
        elif left_zero_offsets is None:
            print("Left arm offset file missing, skipping.")
        else:
            print("Left arm already calibrated, skipping.")

        # 右臂校准 (Joint IDs: 13, 14, 15, 16, 17, 18, 19)
        joint_ids = list(range(13, 20))
        right_zero_offsets, right_calibrated = load_zero_offsets(is_left=False)
        if right_zero_offsets is not None and not right_calibrated:
            set_zero_offset(zero_offsets_deg=right_zero_offsets, joint_ids=joint_ids)
            _update_calibration_status(is_left=False)
        elif right_zero_offsets is None:
            print("Right arm offset file missing, skipping.")
        else:
            print("Right arm already calibrated, skipping.")

if __name__ == '__main__':
    main()
