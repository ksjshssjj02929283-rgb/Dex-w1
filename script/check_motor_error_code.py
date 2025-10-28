#!/usr/bin/env python3
import subprocess

def run_cmd(cmd):
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        return f"命令执行失败：{e}\n错误输出：{e.stderr.strip()}"

def get_device_list():
    cmd = ["ethercat", "sl"]
    output = run_cmd(cmd)
    lines = [line for line in output.splitlines() if line.strip()]

    device_lines = []
    for line in lines:
        first_word = line.strip().split()[0]
        if first_word.isdigit():
            device_lines.append(line)

    device_ids = [int(line.strip().split()[0]) for line in device_lines]
    return device_ids, output

def query_error_for_device(idx):
    cmd = ["ethercat", "upload", f"-p{idx}", "0x603f", "0", "-t", "uint16"]
    return run_cmd(cmd)

def main():
    device_ids, sl_output = get_device_list()

    print("ethercat sl 输出：\n")
    print(sl_output)
    print(f"\n检测到的从站设备编号：{device_ids}")
    print(f"从站设备总数：{len(device_ids)}\n")

    for idx in device_ids:
        print(f"查询从站 {idx} 的错误状态...")
        error_output = query_error_for_device(idx)
        print(f"从站 {idx} 错误查询结果：{error_output}\n")

if __name__ == "__main__":
    main()