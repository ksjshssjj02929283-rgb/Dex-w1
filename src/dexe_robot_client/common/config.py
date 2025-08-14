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

LOG_DIR = DATA_DIR / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

DOWNLOAD_DIR = DATA_DIR / "downloads"
DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)

URL_PREFIX = pathlib.Path("http://192.168.3.120/CoreEngine/Data/embodychain_data")