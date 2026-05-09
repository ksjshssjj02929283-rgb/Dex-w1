# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import pathlib

PACKAGE_NAME = "dexe_robot_client"

DATA_DIR = pathlib.Path.home() / ".cache" / "dexe_data"
DATA_DIR.mkdir(parents=True, exist_ok=True)

DOWNLOAD_DIR = DATA_DIR / "download"
DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)

EXTRACT_DIR = DATA_DIR / "extract"
EXTRACT_DIR.mkdir(parents=True, exist_ok=True)

ASSETS_DIR = DATA_DIR / "assembled"
ASSETS_DIR.mkdir(parents=True, exist_ok=True)

LOG_DIR = pathlib.Path.home() / "log"
LOG_DIR.mkdir(parents=True, exist_ok=True)

W1_DIR = ASSETS_DIR / "w1"
W1_DIR.mkdir(parents=True, exist_ok=True)

A1_DIR = ASSETS_DIR / "a1"
A1_DIR.mkdir(parents=True, exist_ok=True)

L1_DIR = ASSETS_DIR / "l1"
L1_DIR.mkdir(parents=True, exist_ok=True)

ENVIRONMENT_DIR = DATA_DIR / "environment"
ENVIRONMENT_DIR.mkdir(parents=True, exist_ok=True)

URL_PREFIX = "http://192.168.3.120/CoreEngine/Data/dexe_data/"