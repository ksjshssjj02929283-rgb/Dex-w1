'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import logging
from dexe_robot_client.common import get_logger, DexeRobotClientLogger

if __name__ == "__main__":
    logger = get_logger()  # 单例
    DexeRobotClientLogger.set_console_level(logging.DEBUG)

    # 会记录到文件 和 打印至终端
    logger.debug("debug")
    logger.info("info")
    logger.warning("warning")
    logger.error("error")
    logger.critical("critical")

    # 同一调用位置在 5 秒内最多打印一次；适合轮询、心跳等高频日志。
    for _ in range(10):
        logger.warning("throttled warning", throttle_interval=5.0)

    # 指定 key 后，即使消息内容变化，也会共享同一个节流窗口。
    logger.info("robot state: running",
                throttle_interval=1.0,
                throttle_key="robot-state")
