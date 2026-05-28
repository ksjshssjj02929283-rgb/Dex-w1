#!/usr/bin/env python3
import argparse
import statistics
import time
from dataclasses import dataclass

import serial
import serial.tools.list_ports


@dataclass
class SerialCfg:
    baudrate: int
    parity: str
    stopbits: float

    def name(self) -> str:
        return f"{self.baudrate} 8{self.parity}{int(self.stopbits)}"


COMMON_CONFIGS = [
    SerialCfg(115200, "N", 1),
    SerialCfg(460800, "E", 1),
    SerialCfg(460800, "E", 2),
    SerialCfg(460800, "N", 1),
    SerialCfg(115200, "E", 1),
    SerialCfg(115200, "E", 2),
]


def sri_crc8(payload: bytes) -> int:
    """SRI manual CRC8: polynomial 0x8C, initial 0x00, over 11-byte payload."""
    crc = 0x00
    for b in payload:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc


def read_bytes(port: str, cfg: SerialCfg, seconds: float) -> bytes:
    parity_map = {
        "N": serial.PARITY_NONE,
        "E": serial.PARITY_EVEN,
        "O": serial.PARITY_ODD,
    }

    with serial.Serial(
        port=port,
        baudrate=cfg.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=parity_map[cfg.parity],
        stopbits=serial.STOPBITS_TWO if cfg.stopbits == 2 else serial.STOPBITS_ONE,
        timeout=0.05,
        rtscts=False,
        xonxoff=False,
        dsrdtr=False,
    ) as ser:
        ser.reset_input_buffer()
        data = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = ser.read(4096)
            if chunk:
                data.extend(chunk)
        return bytes(data)


def find_sri_frames(data: bytes):
    """Find 14-byte frames starting with AA 55 and validate CRC."""
    frames = []
    for i in range(0, max(0, len(data) - 13)):
        if data[i] == 0xAA and data[i + 1] == 0x55:
            frame = data[i : i + 14]
            crc_ok = sri_crc8(frame[2:13]) == frame[13]
            frames.append((i, crc_ok, frame))
    return frames


def parse_sri_frame(frame: bytes, force_rate: float, torque_rate: float):
    b = frame

    fx0 = (b[2] << 9) + (b[3] << 1) + ((b[4] & 0x80) >> 7)
    fy0 = ((b[4] & 0x7F) << 10) + (b[5] << 2) + ((b[6] & 0xC0) >> 6)
    fz0 = ((b[6] & 0x3F) << 11) + (b[7] << 3) + ((b[8] & 0xE0) >> 5)

    mx0 = ((b[8] & 0x1F) << 7) + ((b[9] & 0xFE) >> 1)
    my0 = ((b[9] & 0x01) << 11) + (b[10] << 3) + ((b[11] & 0xE0) >> 5)
    mz0 = ((b[11] & 0x1F) << 7) + ((b[12] & 0xFE) >> 1)

    def sign_mag(raw: int, sign_bit: int, mag_mask: int, rate: float) -> float:
        mag = raw & mag_mask
        value = mag / rate
        return -value if (raw & sign_bit) else value

    fx = sign_mag(fx0, 0x10000, 0xFFFF, force_rate)
    fy = sign_mag(fy0, 0x10000, 0xFFFF, force_rate)
    fz = sign_mag(fz0, 0x10000, 0xFFFF, force_rate)

    mx = sign_mag(mx0, 0x800, 0x7FF, torque_rate)
    my = sign_mag(my0, 0x800, 0x7FF, torque_rate)
    mz = sign_mag(mz0, 0x800, 0x7FF, torque_rate)

    return fx, fy, fz, mx, my, mz


def printable_ratio(data: bytes) -> float:
    if not data:
        return 0.0

    printable = 0
    for c in data:
        if c in (9, 10, 13) or 32 <= c <= 126:
            printable += 1

    return printable / len(data)


def hexdump(data: bytes, max_len: int = 128) -> str:
    data = data[:max_len]
    return " ".join(f"{x:02x}" for x in data)


def serial_parity(parity: str):
    parity_map = {
        "N": serial.PARITY_NONE,
        "E": serial.PARITY_EVEN,
        "O": serial.PARITY_ODD,
    }
    return parity_map[parity]


def serial_stopbits(stopbits: float):
    return serial.STOPBITS_TWO if stopbits == 2 else serial.STOPBITS_ONE


def send_at_command(ser: serial.Serial, cmd: str, wait_s: float = 0.3, max_read: int = 1024) -> bytes:
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    time.sleep(wait_s)
    return ser.read(max_read)


def is_at_ack(resp: bytes) -> bool:
    text = resp.decode(errors="replace")
    return "ACK+" in text and "$OK" in text


def try_at_query(port: str, cfg: SerialCfg, drate: float | None = None) -> bool:
    """
    Try AT command protocol on a serial port/config.

    If drate is not None and AT protocol is detected, set:
        AT+DRATE=drate;drate;drate;drate;drate;drate

    Returns:
        True if AT ACK is detected.
    """

    with serial.Serial(
        port=port,
        baudrate=cfg.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial_parity(cfg.parity),
        stopbits=serial_stopbits(cfg.stopbits),
        timeout=0.1,
        rtscts=False,
        xonxoff=False,
        dsrdtr=False,
    ) as ser:
        # 1. Query software version
        resp_sfwv = send_at_command(ser, "AT+SFWV=?")
        text_sfwv = resp_sfwv.decode(errors="replace")

        print("Sent: AT+SFWV=?")
        print(f"Raw response hex: {hexdump(resp_sfwv, 256)}")
        if is_at_ack(resp_sfwv):
            print("[MATCH] AT response:")
            print(text_sfwv.strip())
        else:
            print("No clear ACK response.")

        # 2. Query current DRATE
        resp_drate_before = send_at_command(ser, "AT+DRATE=?")
        text_drate_before = resp_drate_before.decode(errors="replace")

        print("Sent: AT+DRATE=?")
        print(f"Raw response hex: {hexdump(resp_drate_before, 256)}")
        if is_at_ack(resp_drate_before):
            print("[MATCH] AT response:")
            print(text_drate_before.strip())
        else:
            print("No clear ACK response.")

        at_found = is_at_ack(resp_sfwv) or is_at_ack(resp_drate_before)

        if not at_found:
            return False

        if drate is None:
            return True

        # 3. Set DRATE
        drate_text = f"{drate:g}"
        drate_cmd = (
            f"AT+DRATE={drate_text};{drate_text};{drate_text};"
            f"{drate_text};{drate_text};{drate_text}"
        )

        print(f"\n[INFO] AT port found at {cfg.name()}.")
        print(f"[INFO] Setting DRATE to {drate_text} for all six channels...\n")

        resp_set = send_at_command(ser, drate_cmd)
        text_set = resp_set.decode(errors="replace")

        print(f"Sent: {drate_cmd}")
        print(f"Raw response hex: {hexdump(resp_set, 256)}")
        if is_at_ack(resp_set):
            print("[MATCH] DRATE set response:")
            print(text_set.strip())
        else:
            print("[WARN] No clear ACK after DRATE set command.")

        # 4. Verify DRATE
        resp_verify = send_at_command(ser, "AT+DRATE=?")
        text_verify = resp_verify.decode(errors="replace")

        print("\nSent: AT+DRATE=?")
        print(f"Raw response hex: {hexdump(resp_verify, 256)}")
        if is_at_ack(resp_verify):
            print("[MATCH] DRATE verify response:")
            print(text_verify.strip())
        else:
            print("[WARN] No clear ACK during DRATE verification.")

        return True


def analyze_one(port: str, cfg: SerialCfg, seconds: float, force_rate: float, torque_rate: float):
    print(f"\n=== Testing {port} @ {cfg.name()} ===")

    try:
        data = read_bytes(port, cfg, seconds)
    except Exception as e:
        print(f"[ERROR] open/read failed: {e}")
        return

    print(f"Read bytes: {len(data)}")
    if not data:
        print("No data received.")
        return

    print(f"First bytes: {hexdump(data)}")

    ratio = printable_ratio(data)
    print(f"Printable ratio: {ratio:.3f}")

    if b"ACK+" in data:
        print("[MATCH] ASCII AT response detected: contains ACK+")
        print(data.decode(errors="replace")[:500])

    frames = find_sri_frames(data)
    ok_frames = [x for x in frames if x[1]]
    bad_frames = [x for x in frames if not x[1]]

    print(f"AA55 candidates: {len(frames)}")
    print(f"CRC OK frames:   {len(ok_frames)}")
    print(f"CRC BAD frames:  {len(bad_frames)}")

    if ok_frames:
        offsets = [x[0] for x in ok_frames]
        if len(offsets) >= 2:
            diffs = [b - a for a, b in zip(offsets[:-1], offsets[1:])]
            print(f"Median frame spacing in buffer: {statistics.median(diffs)} bytes")

        print("[MATCH] SRI 14-byte compact binary frame detected.")
        print("Sample decoded frames, assuming current DRATE:")
        for idx, (_, _, frame) in enumerate(ok_frames[:5]):
            fx, fy, fz, mx, my, mz = parse_sri_frame(frame, force_rate, torque_rate)
            print(
                f"  #{idx}: "
                f"Fx={fx:.4f} N, Fy={fy:.4f} N, Fz={fz:.4f} N, "
                f"Mx={mx:.4f} Nm, My={my:.4f} Nm, Mz={mz:.4f} Nm | "
                f"{frame.hex(' ')}"
            )
    elif frames:
        print("[WARN] AA55 found but CRC failed. Possible wrong serial config or wrong frame alignment.")
    else:
        if ratio > 0.7:
            print("[INFO] Mostly printable ASCII, likely text/AT stream or boot log.")
            print(data.decode(errors="replace")[:500])
        else:
            print("[INFO] Unknown binary or wrong serial parameters.")


def list_ports():
    print("Available serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}: {p.description}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", "-p", required=False, help="e.g. /dev/ttyUSB2")
    parser.add_argument("--seconds", "-t", type=float, default=2.0)
    parser.add_argument("--try-all", action="store_true", help="Try common baud/parity/stop configs")
    parser.add_argument("--send-at", action="store_true", help="Send safe AT query commands: SFWV and DRATE")
    parser.add_argument(
        "--drate",
        type=float,
        default=None,
        help="Set all six DRATE scaling factors after AT port is found, e.g. --drate 100",
    )
    parser.add_argument("--force-rate", type=float, default=10.0, help="Current DRATE for Fx/Fy/Fz")
    parser.add_argument("--torque-rate", type=float, default=10.0, help="Current DRATE for Mx/My/Mz")
    parser.add_argument("--list", action="store_true", help="List serial ports")

    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        raise SystemExit("Please specify --port, e.g. --port /dev/ttyUSB2")

    configs = COMMON_CONFIGS if args.try_all else [SerialCfg(460800, "E", 1)]

    # If --drate is provided, AT query must be enabled automatically.
    do_at = args.send_at or args.drate is not None

    for cfg in configs:
        analyze_one(args.port, cfg, args.seconds, args.force_rate, args.torque_rate)

        if do_at:
            print(f"\n--- Trying AT query @ {cfg.name()} ---")
            try:
                ok = try_at_query(args.port, cfg, drate=args.drate)

                if ok:
                    print(f"\n[SUCCESS] Found AT port at {cfg.name()}.")

                    if args.drate is not None:
                        print(f"[SUCCESS] DRATE set attempt completed with target value: {args.drate:g}")

                    # Stop scanning after AT port is found.
                    break

            except Exception as e:
                print(f"[ERROR] AT query failed: {e}")


if __name__ == "__main__":
    main()