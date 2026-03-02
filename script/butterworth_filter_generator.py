#!/usr/bin/env python3
"""
二阶巴特沃斯滤波器临界阻尼参数生成脚本

用法:
    python3 butterworth_filter_generator.py [选项]

选项:
    -s, --fs FS         采样频率 (Hz), 默认: 250
    -f, --fc FC         截止频率 (Hz), 默认: 5
    -fmt, --format      输出格式: cpp, json, simple

示例:
    python3 butterworth_filter_generator.py                    # 默认参数
    python3 butterworth_filter_generator.py --fc 3             # 截止频率 3Hz
    python3 butterworth_filter_generator.py --fs 1000 --fc 10  # 1kHz 采样, 10Hz 截止

输出:
    滤波器系数 b 和 a，用于差分方程:
    y[n] = b[0]*x[n] + b[1]*x[n-1] + b[2]*x[n-2] - a[1]*y[n-1] - a[2]*y[n-2]

原理:
    二阶巴特沃斯滤波器传递函数: H(s) = ωc² / (s² + 2ζωc s + ωc²)
    临界阻尼 (ζ = 1) 无超调。
"""

import math
import argparse
import numpy as np
from typing import Tuple, List

try:
    import matplotlib.pyplot as plt
    from scipy import signal
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def generate_second_order_critical_damping(fs: float, fc: float) -> Tuple[List[float], List[float]]:
    """
    生成二阶巴特沃斯临界阻尼滤波器系数

    Args:
        fs: 采样频率 (Hz)
        fc: 截止频率 (Hz)

    Returns:
        b: 分子系数 [b0, b1, b2]
        a: 分母系数 [a0, a1, a2]
    """
    wc = 2 * math.pi * fc
    T = 1.0 / fs
    omega_c = wc

    # 临界阻尼: ζ = 1.0
    zeta = 1.0

    # 连续系统: H(s) = omega_c^2 / (s^2 + 2*zeta*omega_c*s + omega_c^2)
    omega_c_sq = omega_c * omega_c
    two_zeta_omega_c = 2 * zeta * omega_c

    c = 2.0 / T
    c_sq = c * c

    denominator = c_sq + two_zeta_omega_c * c + omega_c_sq

    a0 = 1.0
    # IIR滤波器公式: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    # 传递函数: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    # 双线性变换后分母为: 1 + (-2(c²-ωc²)/denom)*z^-1 + ((c²-2ζωc*c+ωc²)/denom)*z^-2
    a1 = -2 * (c_sq - omega_c_sq) / denominator
    a2 = (c_sq - two_zeta_omega_c * c + omega_c_sq) / denominator

    b0 = omega_c_sq / denominator
    b1 = 2.0 * b0
    b2 = b0

    b = [b0, b1, b2]
    a = [a0, a1, a2]

    return b, a


def plot_filter_response(b: List[float], a: List[float], fs: float, fc: float):
    """绘制滤波器响应曲线"""
    if not HAS_MATPLOTLIB:
        print("提示: 安装 matplotlib 和 scipy 可查看滤波效果")
        print("  pip install matplotlib scipy")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"2nd Order Butterworth Lowpass (fs={fs}Hz, fc={fc}Hz, zeta=1.0)", fontsize=14)

    # 1. 幅频特性
    ax1 = axes[0, 0]
    w = np.logspace(np.log10(0.1), np.log10(fs/2), 500)
    _, H = signal.freqz(b, a, worN=w, fs=fs)
    ax1.semilogx(w, 20 * np.log10(np.abs(H) + 1e-10))
    ax1.axvline(fc, color='r', linestyle='--', label=f'fc={fc}Hz')
    ax1.axhline(-6, color='g', linestyle=':', label='-6dB')
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title('Magnitude Response')
    ax1.grid(True, which='both', alpha=0.3)
    ax1.legend()
    ax1.set_xlim(0.1, fs/2)
    ax1.set_ylim(-20, 5)

    # 2. 相频特性
    ax2 = axes[0, 1]
    ax2.semilogx(w, np.angle(H, deg=True))
    ax2.axvline(fc, color='r', linestyle='--', label=f'fc={fc}Hz')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Phase (deg)')
    ax2.set_title('Phase Response')
    ax2.grid(True, which='both', alpha=0.3)
    ax2.legend()
    ax2.set_xlim(0.1, fs/2)

    # 3. 阶跃响应
    ax3 = axes[1, 0]
    t = np.arange(0, 1.0, 1.0/fs)
    step_input = np.ones_like(t)
    zi = [0.0, 0.0]
    step_output, _ = signal.lfilter(b, a, step_input, zi=zi)
    ax3.plot(t, step_input, 'b--', label='Input (step)', alpha=0.7)
    ax3.plot(t, step_output, 'r-', label='Output', linewidth=2)
    ax3.axhline(1.0, color='g', linestyle=':', alpha=0.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Amplitude')
    ax3.set_title('Step Response (no overshoot)')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.set_xlim(0, 1.0)

    # 4. 方波响应
    ax4 = axes[1, 1]
    t = np.arange(0, 2.0, 1.0/fs)
    square_input = (np.sin(2 * np.pi * 1 * t) > 0).astype(float)
    zi = [0.0, 0.0]
    square_output, _ = signal.lfilter(b, a, square_input, zi=zi)
    ax4.plot(t, square_input, 'b--', label='Input (square 1Hz)', alpha=0.7)
    ax4.plot(t, square_output, 'r-', label='Output', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Amplitude')
    ax4.set_title('Square Wave Response (1Hz)')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    ax4.set_xlim(0, 2.0)

    plt.tight_layout()
    plt.show()


def format_cpp_array(arr: list, precision: int = 16) -> str:
    """格式化为 C++ 数组形式"""
    formatted = ", ".join([f"{v:.{precision}e}" for v in arr])
    return f"{{ {formatted} }}"


def main():
    parser = argparse.ArgumentParser(
        description="生成二阶巴特沃斯滤波器临界阻尼参数"
    )
    parser.add_argument(
        "--fs", "-s",
        type=float,
        default=250.0,
        help="采样频率 (Hz), 默认: 250"
    )
    parser.add_argument(
        "--fc", "-f",
        type=float,
        default=5.0,
        help="截止频率 (Hz), 默认: 5"
    )
    parser.add_argument(
        "--format", "-fmt",
        choices=["cpp", "json", "simple"],
        default="cpp",
        help="输出格式: cpp, json, simple"
    )

    args = parser.parse_args()

    b, a = generate_second_order_critical_damping(args.fs, args.fc)

    print(f"\n{'='*60}")
    print(f"二阶巴特沃斯滤波器 - 临界阻尼参数")
    print(f"{'='*60}")
    print(f"采样频率: {args.fs} Hz")
    print(f"截止频率: {args.fc} Hz")
    print(f"阻尼比:   ζ = 1.0 (临界阻尼, 无超调)")
    print(f"{'='*60}\n")

    if args.format == "cpp":
        print("C++ 代码:")
        print(f"filter_.b = {format_cpp_array(b)};")
        print(f"filter_.a = {format_cpp_array(a)};")
    elif args.format == "json":
        import json
        output = {
            "b": b,
            "a": a,
            "fs": args.fs,
            "fc": args.fc,
            "zeta": 1.0
        }
        print(json.dumps(output, indent=4))
    else:
        print(f"分子系数 b: {b}")
        print(f"分母系数 a: {a}")
        print(f"\n使用方式 (差分方程):")
        print(f"  y[n] = b[0]*x[n] + b[1]*x[n-1] + b[2]*x[n-2]")
        print(f"         - a[1]*y[n-1] - a[2]*y[n-2]")
        print(f"  (a[0] = 1)")

    # 绘制滤波效果
    plot_filter_response(b, a, args.fs, args.fc)


if __name__ == "__main__":
    main()
