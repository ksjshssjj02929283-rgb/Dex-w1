import subprocess
import time
import json

def _pulse_to_degree(pulse):
    # 电机脉冲转角度
    degree = ((pulse - 262144) / 262144) * 360.0
    return degree

def _to_complement_hex(value, bits=32):
    # 若是负数，获取补码表示
    if value < 0:
        # 计算补码： 2^bits + value
        value = (1 << bits) + value
    # 将整数转换为16进制字符串
    hex_value = hex(value)
    return hex_value

def _get_motor_index_and_check_status() -> list:
    import re
    try:
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
    except Exception as e:
        raise RuntimeError(f'执行 "ethercat sl" 失败: {e}')

    driver_positions = []
    non_preop = []  # (index, state)

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

    if len(driver_positions) != 20:
        raise RuntimeError(f'期望 20 个 ZeroErr Driver，实际为 {len(driver_positions)}: {driver_positions}')

    if non_preop:
        details = ', '.join(f'{idx}:{state}' for idx, state in non_preop)
        raise RuntimeError(f'以下 ZeroErr Driver 非 PREOP 状态: {details}')

    print(f"Motor index: [{', '.join(map(str, driver_positions))}], total {len(driver_positions)} motors, all in PREOP state.")
    # 返回 ZeroErr Driver 在 ethercat 从站表中的索引列表（按出现顺序）
    return driver_positions

def _get_motor_position_deg_by_index(motor_index_list: list) -> list:
    # 通过 motor_index 获取电机的当前角度（单位：度）
    qpos_list = []
    for motor_index in motor_index_list:
        result = subprocess.run(['ethercat', 'upload', '-p', str(motor_index), '0x6064', '0'], capture_output=True, text=True)
        stdout = result.stdout
        pulse = int(stdout.split(' ')[1])
        qpos_list.append(_pulse_to_degree(pulse))
    return qpos_list

def _write_zero_offset(is_left: bool, index_list: list, zero_offset_list: list):
    # 仅处理对应手臂的关节索引：左臂 6..12，右臂 13..19
    target_joint_ids = list(range(6, 13)) if is_left else list(range(13, 20))
    if len(zero_offset_list) != len(target_joint_ids):
        raise ValueError(f"offset_list 长度应为 {len(target_joint_ids)}，当前为 {len(zero_offset_list)}")
    pre_offset_list = []
    new_offset_list = []

    for i_local, joint_id in enumerate(target_joint_ids):
        # joint_id 是 0..19 的关节序号，index_list[joint_id] 为对应 EtherCAT 从站索引（-p 参数）
        index = index_list[joint_id]
        offset = zero_offset_list[i_local]

        # 读取原本的零位偏差值
        result = subprocess.run(['ethercat', 'upload', '-p', str(index), '0x3b75', '0'], capture_output=True, text=True)
        pre_offset = int(result.stdout.split(' ')[1])
        pre_offset_list.append(pre_offset)

        # 计算新的零位偏差值
        new_offset = int(pre_offset + offset / 360.0 * 262144)
        new_offset_list.append(new_offset)

        # 写入新的零位偏差值
        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x3b75', '0', _to_complement_hex(new_offset)], capture_output=True, text=True)

        # 保存写入值（已解锁保存区）
        result = subprocess.run(['ethercat', 'download', '-p', str(index), '0x1010', '1', '0x65766173'], capture_output=True, text=True)

    print(f"pre_offset:", pre_offset_list)
    print(f"new_offset:", new_offset_list)
    time.sleep(2)
    print("Zero offsets updated!")

def set_zero_offset(is_left: bool, zero_offsets_deg: list) -> bool:
    motor_index_list = _get_motor_index_and_check_status()
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) before calibration:", qpos_list)
    _write_zero_offset(is_left, motor_index_list, zero_offsets_deg)
    qpos_list = _get_motor_position_deg_by_index(motor_index_list)
    print("Current motor positions (deg) after calibration:", qpos_list)

    return True

if __name__ == '__main__':
    # 示例：设置左臂零位偏差
    # 从~/dexeRP_data/W1_V02_L.json中获取零位偏差
    with open('/home/zh/dexeRP_data/W1_V02_L.json', 'r') as f:
        data = json.load(f)
        left_zero_offsets = data['zero_offset']  # 左臂7个关节的零位偏差（单位：度）
    set_zero_offset(is_left=True, zero_offsets_deg=left_zero_offsets)

    with open('/home/zh/dexeRP_data/W1_V02_R.json', 'r') as f:
        data = json.load(f)
        right_zero_offsets = data['zero_offset']  # 右臂7个关节的零位偏差（单位：度）
    set_zero_offset(is_left=False, zero_offsets_deg=right_zero_offsets)
