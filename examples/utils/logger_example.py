# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_robot_client.common import get_logger

if __name__ == "__main__":
    logger = get_logger()  # 单例

    # 会记录到文件 和 打印至终端
    logger.debug("debug")
    logger.info("info")
    logger.warning("warning")
    logger.error("error")
    logger.critical("critical")
