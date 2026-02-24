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
