# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import pathlib

g_software_name = "dexe_robot_client"

g_log_dir = pathlib.Path.cwd() / "log"
g_data_dir = pathlib.Path.cwd() / "data"

if not g_log_dir.exists():
    g_log_dir.mkdir()
if not g_data_dir.exists():
    g_data_dir.mkdir()
