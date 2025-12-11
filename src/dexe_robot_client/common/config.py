# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import pathlib

PACKAGE_NAME = "dexe_robot_client"

DATA_DIR = pathlib.Path.home() / f"{PACKAGE_NAME}_data"
DATA_DIR.mkdir(parents=True, exist_ok=True)

ASSETS_DIR = DATA_DIR / "assets"
ASSETS_DIR.mkdir(parents=True, exist_ok=True)

LOG_DIR = pathlib.Path.home() / "log"
LOG_DIR.mkdir(parents=True, exist_ok=True)

DOWNLOAD_DIR = DATA_DIR / "downloads"
DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)

MODEL_DIR = ASSETS_DIR / "Model"
MODEL_DIR.mkdir(parents=True, exist_ok=True)

ENVIRONMENT_DIR = ASSETS_DIR / "Environment"
ENVIRONMENT_DIR.mkdir(parents=True, exist_ok=True)

URL_PREFIX = "http://192.168.3.120/CoreEngine/Data/RLIA/rlia_data/RobotModel/Dexe_robot_client"