# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from .config import DATA_DIR, DOWNLOAD_DIR, EXTRACT_DIR, ASSETS_DIR, LOG_DIR, W1_DIR, A1_DIR, L1_DIR, ENVIRONMENT_DIR, URL_PREFIX
from .logger import get_logger, DexeRobotClientLogger
from .status import StatusCode, StatusContext
from .utils import (time_consuming, download_file, unzip_file,
                    download_and_unzip_file,
                    download_environment_assets)
from dexe_rp.utils.type import ArmType, W1Version
from .mapping_index import (RealRobotIdx, get_robot_sim2real_idx,
                            get_robot_real2sim_idx, get_ee_sim2real_idx,
                            get_ee_real2sim_idx, get_pin2real_idx,
                            get_real2pin_idx, get_real_idx, get_pin_idx)

__all__ = [
    "DATA_DIR", "DOWNLOAD_DIR", "EXTRACT_DIR", "ASSETS_DIR", "LOG_DIR", "W1_DIR", "A1_DIR", "L1_DIR", "ENVIRONMENT_DIR", "URL_PREFIX",
    "get_logger", "DexeRobotClientLogger", "StatusCode",
    "StatusContext", "time_consuming", "download_file", "unzip_file",
    "download_and_unzip_file",
    "download_environment_assets", "ArmType", "W1Version", "RealRobotIdx",
    "get_robot_sim2real_idx", "get_robot_real2sim_idx", "get_ee_sim2real_idx",
    "get_ee_real2sim_idx", "get_pin2real_idx", "get_real2pin_idx",
    "get_real_idx", "get_pin_idx"
]
