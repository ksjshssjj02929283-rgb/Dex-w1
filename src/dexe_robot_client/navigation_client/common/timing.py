"""挂钟时间戳工具 —— 供链路延迟诊断使用。

统一用 time.time()（系统挂钟时间，而非 ROS clock / time.monotonic()）打点，
可以直接与另一个进程（上层 application）自己记录的挂钟时间相减对比，
定位延迟发生在链路的哪一段。

不用 ROS clock 的原因：仿真 use_sim_time=True 时 ROS clock 可能与系统时钟
脱节；不用 monotonic 的原因：monotonic 时钟的起点因进程而异，跨进程/跨
主机不可比较。挂钟时间是唯一能在两个独立进程间直接相减出"延迟了多少毫秒"
的选择（前提是两边在同一台机器，或已做 NTP 时钟同步）。
"""

import time


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
