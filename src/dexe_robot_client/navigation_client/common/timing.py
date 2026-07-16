"""挂钟时间戳工具 —— 供链路延迟诊断使用。

统一用 time.time()（系统挂钟时间，而非 ROS clock / time.monotonic()）打点，
可以直接与另一个进程（上层 application）自己记录的挂钟时间相减对比，
定位延迟发生在链路的哪一段。

不用 ROS clock 的原因：仿真 use_sim_time=True 时 ROS clock 可能与系统时钟
脱节；不用 monotonic 的原因：monotonic 时钟的起点因进程而异，跨进程/跨
主机不可比较。挂钟时间是唯一能在两个独立进程间直接相减出"延迟了多少毫秒"
的选择（前提是两边在同一台机器，或已做 NTP 时钟同步）。

**诊断开关**（默认关闭）：完整检查点链（server就绪/已发出/accepted/进度中间
态）只在诊断开关打开时打印，避免正常运行时刷屏。关闭时（默认）只打印
"触发"和"返回"两行普通日志（不带 `[延迟诊断]` 标签），返回那行额外带总耗时。
排查问题时用 `set_diagnostics_enabled(True)` 或设环境变量
`DEXE_NAV_CLIENT_DIAGNOSTICS=1` 打开完整检查点链。
"""

import os
import time

_ENV_VAR = "DEXE_NAV_CLIENT_DIAGNOSTICS"
_diagnostics_enabled = os.environ.get(_ENV_VAR, "0").strip().lower() in (
    "1", "true", "on", "yes")


def diagnostics_enabled() -> bool:
    """延迟诊断完整检查点链的开关状态（默认关闭）。"""
    return _diagnostics_enabled


def set_diagnostics_enabled(enabled: bool) -> None:
    """运行时开关延迟诊断完整检查点链。

    打开：打印发送链路（触发→server就绪→已发出→accepted）和反馈链路
    （进度→result）的全部中间检查点，带 `[延迟诊断]` 标签。
    关闭（默认）：只打印"触发"和"返回"两行普通日志（无标签），排查完
    问题后应该关掉，避免正常运行时刷屏。
    """
    global _diagnostics_enabled
    _diagnostics_enabled = bool(enabled)


def now() -> float:
    """当前挂钟时间（epoch 秒，float，微秒级精度）。"""
    return time.time()


def format_ts(ts: float) -> str:
    """人类可读格式：HH:MM:SS.mmm（本地时区）。"""
    ms = int((ts - int(ts)) * 1000)
    return time.strftime("%H:%M:%S", time.localtime(ts)) + f".{ms:03d}"


def format_checkpoint(label: str, stage: str, ts: float, *, tid=None,
                      prev_ts: float = None, prev_stage: str = None,
                      trigger_ts: float = None) -> str:
    """统一格式的延迟诊断检查点日志行。

    发送链路（触发→server就绪→已发出→accepted）与反馈链路（进度→result）
    的每一个检查点都调用本函数生成日志文本，保证**全链路格式完全一致**：
    阶段名 + 挂钟时间（可与上层 application 直接比较）+ 相对上一检查点的
    增量（定位这一步具体耗时多少）+ 相对链路起点"触发"的累计耗时（看总延迟）。
    """
    tid_str = f"task_id={tid} " if tid is not None else ""
    parts = [f"[延迟诊断][{label}] {tid_str}阶段={stage} wall={format_ts(ts)}"]
    if prev_ts is not None:
        prev_label = prev_stage or "上一阶段"
        parts.append(f"Δ({prev_label}→{stage})={(ts - prev_ts) * 1000:.1f}ms")
    if trigger_ts is not None:
        parts.append(f"累计(自触发)={(ts - trigger_ts) * 1000:.1f}ms")
    return "  ".join(parts)


def _format_duration(elapsed_ms: float) -> str:
    return (f"{elapsed_ms / 1000:.3f}s" if elapsed_ms >= 1000
            else f"{elapsed_ms:.1f}ms")


def format_plain_trigger(label: str, ts: float) -> str:
    """正常模式（诊断关闭）的触发日志——普通日志，不带 [延迟诊断] 标签。"""
    return f"[{label}] 触发 wall={format_ts(ts)}"


def format_plain_return(label: str, ts: float, trigger_ts: float = None, *,
                        tid=None) -> str:
    """正常模式（诊断关闭）的返回日志——普通日志，只给挂钟时间和总耗时。"""
    tid_str = f"task_id={tid} " if tid is not None else ""
    if trigger_ts is not None:
        return (f"[{label}] {tid_str}返回 wall={format_ts(ts)}  "
                f"耗时={_format_duration((ts - trigger_ts) * 1000)}")
    return f"[{label}] {tid_str}返回 wall={format_ts(ts)}"
