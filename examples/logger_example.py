# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_robot_client.common import get_logger


if __name__ == "__main__":
    rcs_logger = get_logger()  # 单例

    # 会记录到文件 和 打印至终端
    rcs_logger.debug("debug")
    rcs_logger.info("info")
    rcs_logger.warning("warning")
    rcs_logger.error("error")
    rcs_logger.critical("critical")
