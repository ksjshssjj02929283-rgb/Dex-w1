#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SRI M4313SXX 六维力传感器实时读取脚本

功能：
    1. 在用户明确指定当前协议为 RS232 或 RS485 的情况下，读取并解析实时六维力数据。
    2. 支持 RS485 14 字节 AA55 紧凑二进制帧解析。
    3. 支持 RS232 AT 指令模式下的 GOD/GSD IEEE 浮点数据解析。

协议选择：
    --protocol rs485
        用于读取连续输出的 14 字节二进制数据帧。
        典型参数：
            460800 baud
            8 数据位
            偶校验 Even
            1 停止位

        帧格式：
            AA 55 + 11 字节数据载荷 + 1 字节 CRC8
            总长度 14 字节

        数据编码：
            Fx/Fy/Fz：17 bit，符号位 + 16 bit 幅值
            Mx/My/Mz：12 bit，符号位 + 11 bit 幅值

        工程值换算：
            Fx/Fy/Fz = 原始整数 / force_drate
            Mx/My/Mz = 原始整数 / torque_drate

        如果你已经执行：
            AT+DRATE=100;100;100;100;100;100

        那么读取 RS485 数据时应使用：
            --drate 100

    --protocol rs232
        用于 AT 指令模式读取 IEEE 浮点数据。
        典型参数：
            115200 baud
            8 数据位
            无校验 None
            1 停止位

本脚本中使用到的 AT 指令：

    1. AT+DCKMD=SUM
        作用：
            设置数据校验方式为 SUM。
        用途：
            本脚本的 RS232 GSD/GOD 解析默认按 SUM 校验处理。
        注意：
            如果你的传感器已经被设置为 CRC32 校验，则需要使用 --check-mode crc32，
            但本脚本当前没有完整实现厂家 CRC32 校验算法。

    2. AT+SMPF=<sample_rate>
        作用：
            设置系统采样率。
        示例：
            AT+SMPF=100
        用途：
            当 RS232 使用 GSD 连续输出时，可以通过 --sample-rate 100 设置采样率。
        注意：
            该指令是否掉电保存取决于传感器固件/手册说明。

    3. AT+GSD
        作用：
            启动 RS232 连续获取数据。
        数据格式：
            IEEE754 单精度浮点数据。
        用途：
            本脚本在 --protocol rs232 --mode gsd 时发送该指令。

    4. AT+GSD=STOP
        作用：
            停止 RS232 GSD 连续数据输出。
        用途：
            本脚本在 Ctrl+C 退出或异常退出 GSD 循环时尝试自动发送。

    5. AT+GOD
        作用：
            RS232 单次获取一帧数据。
        数据格式：
            IEEE754 单精度浮点数据。
        用途：
            本脚本在 --protocol rs232 --mode god 时周期性发送该指令。

本脚本不会自动修改 DRATE。
如果需要修改 DRATE，请先通过 AT 指令口执行：
    AT+DRATE=100;100;100;100;100;100

然后读取 RS485 数据时使用：
    --drate 100

典型用法：

    1. 读取 RS485 AA55 数据口，DRATE=100：
        python3 read_sri_realtime.py --port /dev/ttyUSB2 --protocol rs485 --drate 100

    2. 读取 RS485 AA55 数据口，力倍率 100，力矩倍率 10：
        python3 read_sri_realtime.py --port /dev/ttyUSB2 --protocol rs485 --force-drate 100 --torque-drate 10

    3. 读取 RS232 AT 口，GSD 连续浮点输出：
        python3 read_sri_realtime.py --port /dev/ttyUSB2 --protocol rs232 --mode gsd --sample-rate 100

    4. 读取 RS232 AT 口，GOD 单次轮询：
        python3 read_sri_realtime.py --port /dev/ttyUSB2 --protocol rs232 --mode god --print-hz 10
"""

import argparse
import struct
import sys
import time
from dataclasses import dataclass

import serial


@dataclass
class SerialCfg:
    """串口配置结构体。"""

    baudrate: int
    parity: str
    stopbits: int


def parity_to_pyserial(parity: str):
    """将字符形式的校验位转换为 pyserial 使用的常量。"""
    parity = parity.upper()

    if parity == "N":
        return serial.PARITY_NONE
    if parity == "E":
        return serial.PARITY_EVEN
    if parity == "O":
        return serial.PARITY_ODD

    raise ValueError(f"不支持的校验位: {parity}")


def stopbits_to_pyserial(stopbits: int):
    """将停止位数量转换为 pyserial 使用的常量。"""
    if stopbits == 1:
        return serial.STOPBITS_ONE
    if stopbits == 2:
        return serial.STOPBITS_TWO

    raise ValueError(f"不支持的停止位: {stopbits}")


def open_serial(port: str, cfg: SerialCfg) -> serial.Serial:
    """打开串口。"""
    return serial.Serial(
        port=port,
        baudrate=cfg.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=parity_to_pyserial(cfg.parity),
        stopbits=stopbits_to_pyserial(cfg.stopbits),
        timeout=0.02,
        rtscts=False,
        xonxoff=False,
        dsrdtr=False,
    )


# ============================================================
# RS485：14 字节 AA55 紧凑二进制帧解析
# ============================================================


def sri_crc8(payload: bytes) -> int:
    """
    计算 SRI RS485 紧凑帧 CRC8。

    CRC 参数：
        多项式：0x8C
        初始值：0x00
        校验范围：11 字节数据载荷
        不包含帧头 AA55
        不包含最后 1 字节 CRC 本身
    """
    crc = 0x00

    for b in payload:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF

    return crc


def parse_rs485_compact_frame(frame: bytes, force_drate: float, torque_drate: float):
    """
    解析 RS485 14 字节紧凑帧。

    帧结构：
        Byte 0      : 0xAA
        Byte 1      : 0x55
        Byte 2~12   : 11 字节数据载荷
        Byte 13     : CRC8

    数据位分配：
        Fx：17 bit
        Fy：17 bit
        Fz：17 bit
        Mx：12 bit
        My：12 bit
        Mz：12 bit
        Reserved：1 bit

    编码方式：
        符号 + 幅值，不是二进制补码。
    """
    if len(frame) != 14:
        raise ValueError("RS485 紧凑帧长度必须为 14 字节")

    if frame[0] != 0xAA or frame[1] != 0x55:
        raise ValueError("RS485 紧凑帧帧头错误")

    payload = frame[2:13]
    recv_crc = frame[13]
    calc_crc = sri_crc8(payload)

    if recv_crc != calc_crc:
        raise ValueError(f"CRC 错误: recv=0x{recv_crc:02x}, calc=0x{calc_crc:02x}")

    b = payload

    # Fx/Fy/Fz：每个 17 bit
    fx0 = (b[0] << 9) + (b[1] << 1) + ((b[2] & 0x80) >> 7)
    fy0 = ((b[2] & 0x7F) << 10) + (b[3] << 2) + ((b[4] & 0xC0) >> 6)
    fz0 = ((b[4] & 0x3F) << 11) + (b[5] << 3) + ((b[6] & 0xE0) >> 5)

    # Mx/My/Mz：每个 12 bit
    mx0 = ((b[6] & 0x1F) << 7) + ((b[7] & 0xFE) >> 1)
    my0 = ((b[7] & 0x01) << 11) + (b[8] << 3) + ((b[9] & 0xE0) >> 5)
    mz0 = ((b[9] & 0x1F) << 7) + ((b[10] & 0xFE) >> 1)

    def sign_mag(raw: int, sign_bit: int, mag_mask: int, scale: float) -> float:
        """符号 + 幅值格式转工程值。"""
        mag = raw & mag_mask
        value = mag / scale
        return -value if (raw & sign_bit) else value

    fx = sign_mag(fx0, 0x10000, 0xFFFF, force_drate)
    fy = sign_mag(fy0, 0x10000, 0xFFFF, force_drate)
    fz = sign_mag(fz0, 0x10000, 0xFFFF, force_drate)

    mx = sign_mag(mx0, 0x0800, 0x07FF, torque_drate)
    my = sign_mag(my0, 0x0800, 0x07FF, torque_drate)
    mz = sign_mag(mz0, 0x0800, 0x07FF, torque_drate)

    return fx, fy, fz, mx, my, mz


def read_rs485_loop(ser: serial.Serial, force_drate: float, torque_drate: float, print_hz: float):
    """
    循环读取 RS485 AA55 紧凑二进制帧。

    注意：
        这里的 print_hz 只是终端打印频率，不是传感器采样率。
        传感器数据仍按自己的采样率持续上传。
    """
    buf = bytearray()
    last_print = 0.0
    frame_count = 0
    bad_crc = 0

    while True:
        data = ser.read(4096)
        if data:
            buf.extend(data)

        # 防止异常情况下缓冲区无限增长
        if len(buf) > 8192:
            del buf[:-1024]

        while True:
            # 查找帧头 AA55
            idx = buf.find(b"\xAA\x55")

            if idx < 0:
                # 保留最后 1 字节，防止 AA 和 55 被拆到两次 read 中
                if len(buf) > 1:
                    del buf[:-1]
                break

            # 丢弃帧头前的无效字节
            if idx > 0:
                del buf[:idx]

            # 不足一帧，等待更多数据
            if len(buf) < 14:
                break

            frame = bytes(buf[:14])
            del buf[:14]

            try:
                fx, fy, fz, mx, my, mz = parse_rs485_compact_frame(
                    frame,
                    force_drate=force_drate,
                    torque_drate=torque_drate,
                )
                frame_count += 1
            except ValueError:
                bad_crc += 1
                # 校验失败时轻微重同步
                if len(buf) > 0:
                    del buf[:1]
                continue

            now = time.time()
            if now - last_print >= 1.0 / print_hz:
                last_print = now
                print(
                    f"Fx={fx: .4f} N, Fy={fy: .4f} N, Fz={fz: .4f} N, "
                    f"Mx={mx: .4f} Nm, My={my: .4f} Nm, Mz={mz: .4f} Nm ",
                    flush=True,
                )


# ============================================================
# RS232：AT 指令 GOD/GSD IEEE 浮点数据解析
# ============================================================


def data_sum8(data: bytes) -> int:
    """
    计算 SUM 校验。

    默认假设传感器 DCKMD 为 SUM。
    SUM 校验范围为 Data 区域。
    """
    return sum(data) & 0xFF


def send_at(ser: serial.Serial, cmd: str, wait_s: float = 0.1, read_len: int = 4096) -> bytes:
    """
    发送一条 AT 指令。

    自动追加：
        \\r\\n

    例如：
        send_at(ser, "AT+GOD")
    实际发送：
        AT+GOD\\r\\n
    """
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    time.sleep(wait_s)
    return ser.read(read_len)


def parse_rs232_ieee_frame(frame: bytes, check_mode: str = "sum"):
    """
    解析 RS232 GOD/GSD IEEE 浮点数据帧。

    按手册中的 GOD/GSD 数据格式理解：
        AA 55
        PackageLength：2 字节，大端
        PackageNo：2 字节，大端
        Data：PackageLength 字节
        Check：SUM 模式下 1 字节；CRC32 模式下 4 字节

    本脚本默认：
        Check = SUM
        Data 前 24 字节为 6 个 float32：
            Fx, Fy, Fz, Mx, My, Mz

    浮点字节序：
        默认按 little-endian 解析，即 struct.unpack("<6f", ...)
    """
    if len(frame) < 7:
        raise ValueError("RS232 IEEE 数据帧过短")

    if frame[0] != 0xAA or frame[1] != 0x55:
        raise ValueError("RS232 IEEE 数据帧帧头错误")

    package_len = (frame[2] << 8) | frame[3]
    package_no = (frame[4] << 8) | frame[5]

    if package_len < 24:
        raise ValueError(f"PackageLength={package_len} 异常，至少应包含 6 个 float，即 24 字节")

    check_mode = check_mode.lower()
    check_len = 4 if check_mode == "crc32" else 1
    total_len = 2 + 2 + 2 + package_len + check_len

    if len(frame) < total_len:
        raise ValueError("RS232 IEEE 数据帧不完整")

    data = frame[6 : 6 + package_len]
    check = frame[6 + package_len : total_len]

    if check_mode == "sum":
        calc = data_sum8(data)
        recv = check[0]
        if calc != recv:
            raise ValueError(f"SUM 校验错误: recv=0x{recv:02x}, calc=0x{calc:02x}")

    elif check_mode == "crc32":
        # 注意：
        # 这里暂不实现厂家 CRC32 校验。
        # 如果你实际把 DCKMD 改为 CRC32，需要补充厂家对应 CRC32 算法。
        pass

    else:
        raise ValueError(f"不支持的校验模式: {check_mode}")

    # 只取前 6 个通道
    fx, fy, fz, mx, my, mz = struct.unpack("<6f", data[:24])

    return package_no, fx, fy, fz, mx, my, mz, total_len


def read_rs232_gsd_loop(
    ser: serial.Serial,
    print_hz: float,
    sample_rate: int | None,
    check_mode: str,
):
    """
    RS232 GSD 连续读取模式。

    使用指令：
        AT+DCKMD=SUM
            设置校验为 SUM。

        AT+SMPF=<sample_rate>
            设置采样率。仅当用户传入 --sample-rate 时发送。

        AT+GSD
            开始连续输出。

        AT+GSD=STOP
            退出时停止连续输出。
    """
    # 尽量将校验模式设置为 SUM，便于本脚本解析
    resp = send_at(ser, "AT+DCKMD=SUM", wait_s=0.2)
    if resp:
        text = resp.decode(errors="replace").strip()
        if text:
            print(f"[AT] DCKMD response: {text}")

    # 可选：设置采样率
    if sample_rate is not None:
        resp = send_at(ser, f"AT+SMPF={sample_rate}", wait_s=0.2)
        print(f"[AT] SMPF response: {resp.decode(errors='replace').strip()}")

    # 启动连续输出
    resp = send_at(ser, "AT+GSD", wait_s=0.1, read_len=4096)
    buf = bytearray(resp)

    print("[INFO] 已启动 RS232 GSD 连续输出。按 Ctrl+C 停止。")

    last_print = 0.0
    frame_count = 0
    bad_frame = 0

    try:
        while True:
            data = ser.read(4096)
            if data:
                buf.extend(data)

            # 防止异常情况下缓冲区无限增长
            if len(buf) > 16384:
                del buf[:-4096]

            while True:
                idx = buf.find(b"\xAA\x55")

                if idx < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break

                if idx > 0:
                    del buf[:idx]

                if len(buf) < 7:
                    break

                package_len = (buf[2] << 8) | buf[3]
                check_len = 4 if check_mode.lower() == "crc32" else 1
                total_len = 2 + 2 + 2 + package_len + check_len

                if package_len <= 0 or package_len > 4096:
                    bad_frame += 1
                    del buf[:1]
                    continue

                if len(buf) < total_len:
                    break

                frame = bytes(buf[:total_len])
                del buf[:total_len]

                try:
                    package_no, fx, fy, fz, mx, my, mz, _ = parse_rs232_ieee_frame(
                        frame,
                        check_mode=check_mode,
                    )
                    frame_count += 1

                except ValueError:
                    bad_frame += 1
                    continue

                now = time.time()
                if now - last_print >= 1.0 / print_hz:
                    last_print = now
                    print(
                        f"Fx={fx: .4f} N, Fy={fy: .4f} N, Fz={fz: .4f} N, "
                        f"Mx={mx: .4f} Nm, My={my: .4f} Nm, Mz={mz: .4f} Nm ",
                        flush=True,
                    )

    finally:
        # 无论正常退出还是 Ctrl+C，都尝试停止连续输出
        try:
            ser.write(b"AT+GSD=STOP\r\n")
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass

        print("\n[INFO] 已发送 AT+GSD=STOP。")


def read_rs232_god_loop(ser: serial.Serial, print_hz: float, check_mode: str):
    """
    RS232 GOD 单次轮询模式。

    使用指令：
        AT+GOD

    每次发送后等待一帧数据。
    适合低频调试，不适合高实时性控制。
    """
    period = 1.0 / print_hz

    print("[INFO] 已启动 RS232 GOD 单次轮询。按 Ctrl+C 停止。")

    while True:
        t0 = time.time()

        ser.reset_input_buffer()
        ser.write(b"AT+GOD\r\n")
        ser.flush()

        buf = bytearray()
        deadline = time.time() + 1.0
        got = False

        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)

            idx = buf.find(b"\xAA\x55")

            if idx >= 0 and len(buf) >= idx + 7:
                package_len = (buf[idx + 2] << 8) | buf[idx + 3]
                check_len = 4 if check_mode.lower() == "crc32" else 1
                total_len = 2 + 2 + 2 + package_len + check_len

                if len(buf) >= idx + total_len:
                    frame = bytes(buf[idx : idx + total_len])

                    try:
                        package_no, fx, fy, fz, mx, my, mz, _ = parse_rs232_ieee_frame(
                            frame,
                            check_mode=check_mode,
                        )

                        print(
                            f"Fx={fx: .4f} N, Fy={fy: .4f} N, Fz={fz: .4f} N, "
                            f"Mx={mx: .4f} Nm, My={my: .4f} Nm, Mz={mz: .4f} Nm "
                            f"| pkg={package_no}",
                            flush=True,
                        )
                        got = True

                    except ValueError as e:
                        print(f"[WARN] GOD 数据帧解析失败: {e}", file=sys.stderr)

                    break

        if not got:
            print("[WARN] GOD 超时或未收到有效帧", file=sys.stderr)

        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


# ============================================================
# 主函数
# ============================================================


def main():
    parser = argparse.ArgumentParser(
        description="读取 SRI M4313SXX 六维力传感器实时解析数据。"
    )

    parser.add_argument(
        "--port",
        "-p",
        required=True,
        help="串口设备，例如 /dev/ttyUSB2",
    )

    parser.add_argument(
        "--protocol",
        choices=["rs232", "rs485"],
        required=True,
        help="明确指定当前协议：rs232 或 rs485",
    )

    parser.add_argument(
        "--baudrate",
        type=int,
        default=None,
        help="覆盖默认波特率。例如 RS485 常用 460800，RS232 AT 口常用 115200。",
    )

    parser.add_argument(
        "--parity",
        choices=["N", "E", "O"],
        default=None,
        help="覆盖默认校验位：N=无校验，E=偶校验，O=奇校验。",
    )

    parser.add_argument(
        "--stopbits",
        type=int,
        choices=[1, 2],
        default=None,
        help="覆盖默认停止位：1 或 2。",
    )

    parser.add_argument(
        "--drate",
        type=float,
        default=100.0,
        help="RS485 模式下六个通道统一使用的 DRATE。例：--drate 100。",
    )

    parser.add_argument(
        "--force-drate",
        type=float,
        default=None,
        help="RS485 模式下 Fx/Fy/Fz 的 DRATE。若指定，则覆盖 --drate。",
    )

    parser.add_argument(
        "--torque-drate",
        type=float,
        default=None,
        help="RS485 模式下 Mx/My/Mz 的 DRATE。若指定，则覆盖 --drate。",
    )

    parser.add_argument(
        "--mode",
        choices=["gsd", "god"],
        default="gsd",
        help="RS232 采集模式：gsd=连续输出，god=单次轮询。",
    )

    parser.add_argument(
        "--sample-rate",
        type=int,
        default=None,
        help="RS232 GSD 模式下设置 SMPF 采样率。例如 --sample-rate 100。",
    )

    parser.add_argument(
        "--check-mode",
        choices=["sum", "crc32"],
        default="sum",
        help="RS232 GOD/GSD 数据校验方式。默认 sum。",
    )

    parser.add_argument(
        "--print-hz",
        type=float,
        default=20.0,
        help="终端打印频率，不等于传感器采样率。",
    )

    args = parser.parse_args()

    # 根据协议选择默认串口参数
    if args.protocol == "rs232":
        # RS232 AT 指令口默认：115200 8N1
        cfg = SerialCfg(
            baudrate=args.baudrate if args.baudrate is not None else 115200,
            parity=args.parity if args.parity is not None else "N",
            stopbits=args.stopbits if args.stopbits is not None else 1,
        )
    else:
        # RS485 AA55 数据口默认：460800 8E1
        cfg = SerialCfg(
            baudrate=args.baudrate if args.baudrate is not None else 460800,
            parity=args.parity if args.parity is not None else "E",
            stopbits=args.stopbits if args.stopbits is not None else 1,
        )

    print(
        f"[INFO] 打开串口 {args.port}: "
        f"{cfg.baudrate}, 8{cfg.parity}{cfg.stopbits}, protocol={args.protocol}"
    )

    ser = open_serial(args.port, cfg)

    try:
        if args.protocol == "rs485":
            force_drate = args.force_drate if args.force_drate is not None else args.drate
            torque_drate = args.torque_drate if args.torque_drate is not None else args.drate

            print(
                f"[INFO] 使用 RS485 AA55 紧凑帧解析器："
                f"force_drate={force_drate:g}, torque_drate={torque_drate:g}"
            )

            read_rs485_loop(
                ser,
                force_drate=force_drate,
                torque_drate=torque_drate,
                print_hz=args.print_hz,
            )

        else:
            print(f"[INFO] 使用 RS232 AT 指令模式：mode={args.mode}")

            if args.mode == "gsd":
                read_rs232_gsd_loop(
                    ser,
                    print_hz=args.print_hz,
                    sample_rate=args.sample_rate,
                    check_mode=args.check_mode,
                )
            else:
                read_rs232_god_loop(
                    ser,
                    print_hz=args.print_hz,
                    check_mode=args.check_mode,
                )

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断。")

    finally:
        ser.close()
        print("[INFO] 串口已关闭。")


if __name__ == "__main__":
    main()