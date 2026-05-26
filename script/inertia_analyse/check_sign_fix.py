#!/usr/bin/env python3
"""离线验证工具惯量补偿符号修复.

输入: 修复前 server 采集的 JSON (含 force_after_gravity_before_inertia,
      force_after_inertia, force_sensor_inertia 字段).

逻辑: 当前 C++ 是 wrench_after = wrench_before - expected. 修复后符号翻转 (expected
      取负号), 等价于 wrench_after_fixed = wrench_before + expected_old. 我们用旧
      JSON 里的 expected_old (= force_sensor_inertia) 离线模拟修复后效果, 比较 RMS.

用法:
    python3 check_sign_fix.py /path/to/dump.json
"""

import json
import math
import sys
from pathlib import Path


def rms_norm(rows):
    n = len(rows)
    if n == 0:
        return 0.0
    sq = 0.0
    for r in rows:
        sq += sum(x * x for x in r)
    return math.sqrt(sq / n)


def project_k(before_rows, inertia_rows):
    """对模型 after = before - k * inertia 做最小二乘拟合 k 的标量."""
    num = 0.0
    den = 0.0
    for br, ir in zip(before_rows, inertia_rows):
        for b, i in zip(br, ir):
            num += b * i
            den += i * i
    return num / den if den > 0 else float("nan")


def main():
    if len(sys.argv) < 2:
        print("usage: check_sign_fix.py <dump.json>", file=sys.stderr)
        sys.exit(1)
    path = Path(sys.argv[1])
    data = json.loads(path.read_text())
    samples = data.get("samples", data if isinstance(data, list) else [])
    if not samples:
        print("no samples in JSON", file=sys.stderr)
        sys.exit(2)

    before = [s["force_after_gravity_before_inertia"] for s in samples]
    after_cur = [s["force_after_inertia"] for s in samples]
    inertia_old = [s["force_sensor_inertia"] for s in samples]
    # 修复后效果: after_fixed = before - (-expected_old) = before + expected_old
    after_fixed = [
        [b + i for b, i in zip(br, ir)] for br, ir in zip(before, inertia_old)
    ]

    print(f"samples       : {len(samples)}")
    print(f"before RMS    : {rms_norm(before):.4f}")
    print(f"after current : {rms_norm(after_cur):.4f}  (current C++ subtracts wrong sign)")
    print(f"after fixed   : {rms_norm(after_fixed):.4f}  (sign-flipped expected)")
    k = project_k(before, inertia_old)
    print(f"k_opt (after = before - k * inertia_old): {k:.4f}")
    print(
        "Interpretation:"
        " k≈+1 means current convention correct;"
        " k≈-1 means sign needs flipping."
    )
    if rms_norm(after_fixed) < rms_norm(before):
        print("[OK] Sign flip improves residual RMS.")
    else:
        print("[FAIL] Sign flip does NOT improve residual.")


if __name__ == "__main__":
    main()
