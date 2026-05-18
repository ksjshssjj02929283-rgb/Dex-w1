import os
import time
import subprocess
import re
import sys

def to_complement_hex(value, bits=32):
    # 若是负数，获取补码表示
    if value < 0:
        # 计算补码： 2^bits + value
        value = (1 << bits) + value
    # 将整数转换为16进制字符串
    hex_value = hex(value)
    return hex_value

def get_motor_indices():
    """获取所有 ZeroErr Driver 的电机索引"""
    try:
        result = subprocess.run(['ethercat', 'sl'], capture_output=True, text=True, check=True)
    except Exception as e:
        print(f'执行 "ethercat sl" 失败: {e}')
        sys.exit(1)

    motor_indices = []
    for line in result.stdout.splitlines():
        if 'ZeroErr Driver' in line:
            # 匹配行首的数字（索引）
            m = re.match(r'^\s*(\d+)\s+', line)
            if m:
                motor_indices.append(int(m.group(1)))
    
    if not motor_indices:
        print("未检测到任何 ZeroErr Driver 电机。")
        sys.exit(1)
    
    print(f"检测到电机索引: {motor_indices}")
    return motor_indices

def check_firmware_version(motor_indices):
    version_ok = True
    for pos in motor_indices:
        command = ['ethercat', 'upload', '-p', str(pos), '0x2422', '0']
        result = subprocess.run(command, capture_output=True, text=True)
        stdout = result.stdout
        
        if not stdout.strip():
            print(f"无法读取电机 {pos} 的固件版本")
            continue

        version = stdout.split(' ')[1]
        if int(version) != 550113100 and int(version) != 150113100:
            print('WRONG FIRMWARE VERSION: position:%d version:%s' % (pos, version))
            version_ok = False
    return version_ok

def change_to_180_degree(pos):

    # write 0x3b75 = 0
    command = ['ethercat', 'download', '-p', str(pos), '0x3b75', '0', '0']
    result = subprocess.run(command, capture_output=True, text=True)

    # write 0x2242 = 0
    command = ['ethercat', 'download', '-p', str(pos), '0x2242', '0', '0']
    result = subprocess.run(command, capture_output=True, text=True)

    # read 0x6064
    command = ['ethercat', 'upload', '-p', str(pos), '0x6064', '0', '-t', 'uint32']
    result = subprocess.run(command, capture_output=True, text=True)
    stdout = result.stdout
    qpos = int(stdout.split(' ')[1])
    print('current qpos: %d' % qpos)

    # calculate zero offset
    offset = 262144 - qpos
    #if offset < 0:
    #    offset += 2**19

    # write 0x3b75 = offset
    command = ['ethercat', 'download', '-p', str(pos), '0x3b75', '0', to_complement_hex(offset)]
    print(' '.join(command))
    result = subprocess.run(command, capture_output=True, text=True)

    # read 0x6064
    command = ['ethercat', 'upload', '-p', str(pos), '0x6064', '0', '-t', 'uint32']
    result = subprocess.run(command, capture_output=True, text=True)
    stdout = result.stdout
    qpos = int(stdout.split(' ')[1])
    print('current qpos: %d' % qpos)
    if abs(262144 - qpos) < 20:
        print('set ok')
    else:
        print('set failed')

    # write 0x1010:1 = 0x65766173
    command = ['ethercat', 'download', '-p', str(pos), '0x1010', '1', '0x65766173']
    result = subprocess.run(command, capture_output=True, text=True)

    #time.sleep(5)

def check_180_degree(motor_indices):
    qpos_ok = True
    for pos in motor_indices:
        command = ['ethercat', 'upload', '-p', str(pos), '0x6064', '0', '-t', 'uint32']
        result = subprocess.run(command, capture_output=True, text=True)
        stdout = result.stdout
        if stdout.strip():
            qpos = int(stdout.split(' ')[1])
            print(f"电机 {pos} 当前位置: {qpos}")
    return qpos_ok

if __name__ == '__main__':
    # 不需要配置文件，直接通过 ethercat sl 获取电机列表
    motor_indices = get_motor_indices()

    # 检查固件版本
    check_firmware_version(motor_indices)

    # 依次设置每个电机到 180 度零点
    for pos in motor_indices:
        print(f"正在配置电机 {pos}...")
        change_to_180_degree(pos)
    
    time.sleep(2)
    check_180_degree(motor_indices)