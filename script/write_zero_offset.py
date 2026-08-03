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

def _detect_config(ethercat_output: str) -> tuple:
    """从 ethercat sl 输出自动检测电机类型和构型, 返回 (is_v03, is_a1, motor_count)"""
    NON_MOTOR_KEYWORDS = ['jbox', 'junction', 'power', 'coupler', 'ek1100']
    output_upper = ethercat_output.upper()

    # 检测电机类型
    if 'ZERO_ERROR' in output_upper or 'ZEROERR' in output_upper:
        is_v03 = False
    elif 'TC' in output_upper:
        is_v03 = True
    else:
        raise RuntimeError('无法识别电机类型, ethercat sl 输出中未找到 ZERO_ERROR 或 TC 关键字')

    # 统计电机数量
    motor_count = 0
    for line in ethercat_output.splitlines():
        line_lower = line.lower()
        if any(kw in line_lower for kw in NON_MOTOR_KEYWORDS):
            continue
        if re.match(r'^\s*\d+\s+\S+\s+[A-Z]+\b', line):
            motor_count += 1

    # 检测构型: 20=W1, 14/16=A1
    if motor_count == 20:
        is_a1 = False
    elif motor_count in (14, 16):
        is_a1 = True
    else:
        raise RuntimeError(f'无法识别构型, 电机数量 {motor_count} 不匹配 W1(20) 或 A1(14/16)')

    return is_v03, is_a1, motor_count

def _get_motor_index_and_check_status(expected_count: int) -> list:
    try:
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
    except Exception as e:
        raise RuntimeError(f'执行 "ethercat sl" 失败: {e}')

    # 非电机从站关键词, 这些行不会被识别为关节电机
    NON_MOTOR_KEYWORDS = ['jbox', 'junction', 'power', 'coupler', 'ek1100']

    driver_positions = []
    non_preop = []

    for line in result.stdout.splitlines():
        line_lower = line.lower()
        if any(kw in line_lower for kw in NON_MOTOR_KEYWORDS):
            continue
        m = re.match(r'^\s*(\d+)\s+\S+\s+([A-Z]+)\b', line)
        if m:
            idx = int(m.group(1))
            state = m.group(2)
            driver_positions.append(idx)
            if state != 'PREOP':
                non_preop.append((idx, state))

    if len(driver_positions) != expected_count:
        raise RuntimeError(f'期望 {expected_count} 个电机, 实际为 {len(driver_positions)}: {driver_positions}')

    if non_preop:
        details = ', '.join(f'{idx}:{state}' for idx, state in non_preop)
        raise RuntimeError(f'以下电机非 PREOP 状态: {details}')

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

def _set_tc_zero(index_list: list, joint_ids: list, section_name: str):
    """同川电机标零: 将当前关节位置设为零点 (0x3801=5)"""
    print(f"\n=== [{section_name}] 同川电机标零 ===")
    print("将把以下电机的当前位置设为零点:")
    for joint_id in joint_ids:
        index = index_list[joint_id]
        print(f"  电机 {index} (joint_id={joint_id})")

    confirm = input("\n确认将当前位置设为零点? (Y/y 执行, 其他取消): ").strip()
    if confirm.lower() != 'y':
        print("已取消")
        return False

    for joint_id in joint_ids:
        index = index_list[joint_id]
        result = subprocess.run(
            ['ethercat', 'download', '-p', str(index), '0x3801', '0', '5', '-t', 'uint16'],
            capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[ERROR] 电机 {index} 标零失败: {result.stderr.strip()}")
            return False
        print(f"[OK] 电机 {index} 标零完成")

    print(f"[{section_name}] 标零完成")
    return True

def set_zero_offset(zero_offsets_deg: list, joint_ids: list, motor_index_list: list = None, expected_motor_count: int = 20) -> bool:
    if motor_index_list is None:
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

def _calibrate_section(is_v03: bool, motor_index_list: list, joint_ids: list,
                       section_name: str, is_left: bool, is_torso: bool) -> bool:
    """统一的分部位标零入口, 根据 is_v03 选择标零方式"""
    if is_torso:
        zero_offsets, calibrated = load_torso_offsets()
    else:
        zero_offsets, calibrated = load_zero_offsets(is_left=is_left)

    if calibrated:
        print(f"{section_name} already calibrated, skipping.")
        return True
    if zero_offsets is None and not is_v03:
        print(f"{section_name} offset file missing, skipping.")
        return False

    if is_v03:
        success = _set_tc_zero(motor_index_list, joint_ids, section_name)
    else:
        success = set_zero_offset(
            zero_offsets_deg=zero_offsets, joint_ids=joint_ids,
            motor_index_list=motor_index_list)

    if success:
        _update_calibration_status(is_left=is_left, is_torso=is_torso)
    return success

def main():
    # 获取 ethercat sl 输出, 自动检测电机类型和构型
    try:
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
    except Exception as e:
        raise RuntimeError(f'执行 "ethercat sl" 失败: {e}')

    # 允许命令行覆盖自动检测结果
    force_a1 = '--a1' in sys.argv or 'a1' in sys.argv
    force_v03 = '--v03' in sys.argv or 'v03' in sys.argv

    is_v03, is_a1, motor_count = _detect_config(result.stdout)
    if force_a1:
        is_a1 = True
    if force_v03:
        is_v03 = True

    motor_type = 'V03 (同川)' if is_v03 else 'V02 (零差)'
    config_type = 'A1' if is_a1 else 'W1'
    print(f"检测到: {motor_type} 电机, {motor_count} 个, {config_type} 模式")

    if is_v03:
        print("\n*** V03 同川电机标零模式: 将当前关节位置直接设为零点 ***")
        print("*** 请确保机器人已处于期望的零位姿态 ***\n")

    # 关节映射
    # YAML torso_head zero_offset 顺序固定: ANKLE KNEE BUTTOCK WAIST NECK1 NECK2
    if is_v03:
        torso_joint_ids = [19, 18, 17, 16, 0, 1]
        left_joint_ids = [2, 3, 4, 5, 6, 7, 8]
        right_joint_ids = [9, 10, 11, 12, 13, 14, 15]
    else:
        torso_joint_ids = [0, 1, 2, 3, 4, 5]
        left_joint_ids = [6, 7, 8, 9, 10, 11, 12]
        right_joint_ids = [13, 14, 15, 16, 17, 18, 19]

    if is_a1:
        if motor_count == 16:
            a1_left_ids = list(range(2, 9))
            a1_right_ids = list(range(9, 16))
        else:
            a1_left_ids = list(range(7))
            a1_right_ids = list(range(7, 14))

        motor_index_list = _get_motor_index_and_check_status(motor_count)

        left_calibrated = _calibrate_section(
            is_v03, motor_index_list, a1_left_ids, 'left_arm', is_left=True, is_torso=False)
        right_calibrated = _calibrate_section(
            is_v03, motor_index_list, a1_right_ids, 'right_arm', is_left=False, is_torso=False)
    else:
        motor_index_list = _get_motor_index_and_check_status(motor_count)

        _calibrate_section(
            is_v03, motor_index_list, torso_joint_ids, 'torso_head', is_left=False, is_torso=True)
        _calibrate_section(
            is_v03, motor_index_list, left_joint_ids, 'left_arm', is_left=True, is_torso=False)
        _calibrate_section(
            is_v03, motor_index_list, right_joint_ids, 'right_arm', is_left=False, is_torso=False)

if __name__ == '__main__':
    main()
