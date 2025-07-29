# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import pathlib
from setuptools import setup, find_packages

def get_requirements():
    req_path = pathlib.Path("./dependencies/requirements.txt")
    if not req_path.exists():
        raise FileNotFoundError(f"Missing {str(req_path)}, expected in dependencies.")
    with req_path.open("r", encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip() and not line.startswith("#")]

setup(
    name="dexe_robot_client",
    version="0.1.0",
    url="http://69.235.177.182:8081/dexembodied/dexe_robot_client",
    author="Dexforce",
    packages=find_packages(),
    install_requires=get_requirements(),
    python_requires=">=3.10, <3.11"
)