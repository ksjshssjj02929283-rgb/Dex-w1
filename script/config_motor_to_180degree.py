import os
import time
import subprocess
import numpy as np
from config import read_config_file


def to_complement_hex(value, bits=32):
    # 若是负数，获取补码表示
    if value < 0:
        # 计算补码： 2^bits + value
        value = (1 << bits) + value
    # 将整数转换为16进制字符串
    hex_value = hex(value)
    return hex_value

def check_firmware_version(config):
    version_ok = True

    pos_list = list(config.values())
    print(pos_list)

    for pos in pos_list:

        print(pos)

        command = ['ethercat', 'upload', '-p', str(pos), '0x2422', '0']
        print(command)

        result = subprocess.run(command, capture_output=True, text=True)
        stdout = result.stdout
        stderr = result.stderr

        print(stdout)
        version = stdout.split(' ')[1]
        #print(version)

        if int(version) != 550113100 and int(version) != 150113100:
            print('WRONG FIRMWARE VERSION: position:%d version:%s\n' % (pos, version))
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

def check_180_degree(config):
    qpos_ok = True
    pos_list = list(config.values())
    print(pos_list)

    for pos in pos_list:
        print(pos)

        command = ['ethercat', 'upload', '-p', str(pos), '0x6064', '0', '-t', 'uint32']

        result = subprocess.run(command, capture_output=True, text=True)
        stdout = result.stdout
        stderr = result.stderr

        qpos = int(stdout.split(' ')[1])
        print(qpos)

        #if int(version) != 550113100 and int(version) != 150113100:
        #    print('WRONG FIRMWARE VERSION: position:%d version:%s\n' % (pos, version))
        #    version_ok = False

    return qpos_ok

def save_zero_offset():
    offset_list = []
    with open('zeroerr_position_list.txt','r') as fp:
        for line in fp:
            pos = int(line.strip())
            print(pos)

            command = ['ethercat', 'upload', '-p', str(pos), '0x3b75', '0', '-t', 'uint32']

            result = subprocess.run(command, capture_output=True, text=True)
            stdout = result.stdout
            stderr = result.stderr

            offset = int(stdout.split(' ')[1])
            print(offset)

            offset_list.append(offset)

    with open('v2_zero_offset.txt', 'w') as fp:
        for offset in offset_list:
            fp.write('%d,' % offset)
        fp.write('0,0,0,0,0,0,0,0,0,0,0,0\n')

#从运行参数读取config文件path
def get_config_path():
    import sys
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        print(f"Config file {config_path} does not exist.")
        sys.exit(1)

    return config_path

if __name__ == '__main__':

    config_path = get_config_path()
    config = read_config_file(config_path)

    check_firmware_version(config)

    pos_list = list(config.values())
    print(pos_list)
    for pos in pos_list:
        change_to_180_degree(pos)
    time.sleep(5)

    check_180_degree(config)