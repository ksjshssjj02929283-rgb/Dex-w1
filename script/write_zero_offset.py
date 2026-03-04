import subprocess
import time
import json
import sys
import os

DEXE_RP_DATA_PATH = os.environ.get('DEXE_RP_DATA_PATH', '/home/dexforce/dexe_rp_data')

def _pulse_to_degree(pulse):
    degree = ((pulse - 262144) / 262144) * 360.0
    return degree

def _to_complement_hex(value, bits=32):
    if value < 0:
        value = (1 << bits) + value
    hex_value = hex(value)
    return hex_value

def _get_motor_index_and_check_status(expected_count: int) -> list:
    import re
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
        result = subprocess.run(['ethercat', 'upload', '-p', str(motor_index), '0x6064', '0'], capture_output=True, text=True)
        stdout = result.stdout
        pulse = int(stdout.split(' ')[1])
        qpos_list.append(_pulse_to_degree(pulse))
    return qpos_list

def _write_zero_offset(is_left: bool, index_list: list, zero_offset_list: list, joint_ids: list):
    target_joint_ids = joint_ids if is_left else [j + len(joint_ids) for j in joint_ids]
    if len(zero_offset_list) != len(target_joint_ids):
        raise ValueError(f"offset_list 长度应为 {len(target_joint_ids)}，当前为 {len(zero_offset_list)}")
    pre_offset_list = []
    new_offset_list = []

    for i_local, joint_id in enumerate(target_joint_ids):
        index = index_list[joint_id]
        offset = zero_offset_list[i_local]

        result = subprocess.run(['ethercat', 'upload', '-p', str(index), '0x3b75', '0'], capture_output=True, text=True)
        pre_offset = int(result.stdout.split(' ')[1])
        pre_offset_list.append(pre_offset)

        new_offset = int(pre_offset + offset / 360.0 * 262144)
        new_offset_list.append(new_offset)

        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x3b75', '0', _to_complement_hex(new_offset)], capture_output=True, text=True)

        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x1010', '1', '0x65766173'], capture_output=True, text=True)

    print(f"pre_offset:", pre_offset_list)
    print(f"new_offset:", new_offset_list)
    time.sleep(2)
    print("Zero offsets updated!")

def set_zero_offset_w1(is_left: bool, zero_offsets_deg: list, joint_ids: list) -> bool:
    motor_index_list = _get_motor_index_and_check_status(20)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) before calibration:", qpos_list)
    _write_zero_offset(is_left, motor_index_list, zero_offsets_deg, joint_ids)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) after calibration:", qpos_list)
    return True

def set_zero_offset_a1(is_left: bool, zero_offsets_deg: list) -> bool:
    motor_index_list = _get_motor_index_and_check_status(14)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) before calibration:", qpos_list)
    _write_zero_offset(is_left, motor_index_list, zero_offsets_deg, list(range(7)))
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) after calibration:", qpos_list)
    return True

def load_zero_offsets(is_left: bool) -> list:
    json_path = f'{DEXE_RP_DATA_PATH}/W1_V02_L.json' if is_left else f'{DEXE_RP_DATA_PATH}/W1_V02_R.json'
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data['zero_offset']

def main():
    a1_mode = '--a1' in sys.argv or 'a1' in sys.argv

    if a1_mode:
        print("=== A1 模式 ===")
        left_zero_offsets = load_zero_offsets(is_left=True)
        set_zero_offset_a1(is_left=True, zero_offsets_deg=left_zero_offsets)

        right_zero_offsets = load_zero_offsets(is_left=False)
        set_zero_offset_a1(is_left=False, zero_offsets_deg=right_zero_offsets)
    else:
        print("=== W1 模式 ===")
        joint_ids = list(range(6, 13))
        left_zero_offsets = load_zero_offsets(is_left=True)
        set_zero_offset_w1(is_left=True, zero_offsets_deg=left_zero_offsets, joint_ids=joint_ids)

        joint_ids = list(range(7))
        right_zero_offsets = load_zero_offsets(is_left=False)
        set_zero_offset_w1(is_left=False, zero_offsets_deg=right_zero_offsets, joint_ids=joint_ids)

if __name__ == '__main__':
    main()
