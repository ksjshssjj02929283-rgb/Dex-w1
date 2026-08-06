"""任务状态枚举 —— navigation_client 内所有 action 任务的统一状态空间。

对外（W1Manager 等既有调用方）仍输出**旧字符串取值**，由各 feature 的
LEGACY 映射表完成转换；模块内部一律使用 TaskState，不再手写字符串。
"""

from enum import Enum


class TaskState(Enum):
    IDLE = "IDLE"  # 初始空闲
    WAITING_SERVER = "WAITING_SERVER"  # 等待 action server 就绪
    SENDING = "SENDING"  # goal 已发出，等待 goal response
    ACCEPTED = "ACCEPTED"  # goal 被接受
    EXECUTING = "EXECUTING"  # 收到 feedback，执行中
    CANCELING = "CANCELING"  # 取消请求已发出
    SUCCEEDED = "SUCCEEDED"  # result 成功
    CANCELED = "CANCELED"  # result 为取消
    ABORTED = "ABORTED"  # result 为中止
    REJECTED = "REJECTED"  # goal 被拒绝
    FAILED_TO_SEND = "FAILED_TO_SEND"  # server 不可用 / 发送异常 / 未初始化
    LOST = "LOST"  # watchdog 判定通信丢失（response/result 未归）


#: 仍在进行中（可取消）的状态集合
ACTIVE_STATES = frozenset({
    TaskState.WAITING_SERVER,
    TaskState.SENDING,
    TaskState.ACCEPTED,
    TaskState.EXECUTING,
    TaskState.CANCELING,
})

#: 终态集合
TERMINAL_STATES = frozenset({
    TaskState.SUCCEEDED,
    TaskState.CANCELED,
    TaskState.ABORTED,
    TaskState.REJECTED,
    TaskState.FAILED_TO_SEND,
    TaskState.LOST,
})
