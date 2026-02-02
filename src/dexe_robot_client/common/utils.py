'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import os
import pathlib
import zipfile
import requests
import time
import numpy as np
from functools import wraps
from .logger import get_logger
from .config import DOWNLOAD_DIR, ASSETS_DIR, MODEL_DIR, ENVIRONMENT_DIR, URL_PREFIX
from .robot_type import W1ArmType, W1Version
from ..end_effector_client import EndEffectorType


# 函数运行耗时装饰器
def time_consuming(func):

    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed = end_time - start_time
        get_logger().info(f"[{func.__name__}] run time: {elapsed:.6f} s")
        return result

    return wrapper


def unzip_file(filename, zip_path=DOWNLOAD_DIR, unzip_path=ASSETS_DIR) -> bool:
    if not filename.lower().endswith(".zip"):
        get_logger().error(f"File '{filename}' is not a ZIP file.")
        return False

    src_path = pathlib.Path(zip_path) / filename
    if not src_path.exists():
        get_logger().error(f"ZIP file not found: {src_path}")
        return False

    dest_path = pathlib.Path(unzip_path) / pathlib.Path(filename).stem
    if dest_path.exists():
        get_logger().info(
            f"Folder already exists: {dest_path}, skipping unzip.")
        return True

    with zipfile.ZipFile(src_path, 'r') as zip_ref:
        zip_ref.extractall(unzip_path)
    get_logger().info(f"File unzip from {src_path} to {unzip_path}")
    return True


# TODO: 做唯一性检查
def download_file(url: str,
                  dest_path=DOWNLOAD_DIR,
                  filename: str = None) -> bool:
    if filename is None:
        filename = os.path.basename(url)

    file_path = pathlib.Path(dest_path) / filename
    if file_path.exists():
        get_logger().info(
            f"File already exists: {file_path}, skipping download.")
        return True

    response = requests.get(url, timeout=5)
    response.raise_for_status()
    with open(file_path, "wb") as file:
        file.write(response.content)
    get_logger().info(f"File downloaded from {url} to {file_path}")
    return True


def download_and_unzip_file(url: str) -> bool:
    download_file(url, DOWNLOAD_DIR)
    filename = os.path.basename(url)
    return unzip_file(filename, DOWNLOAD_DIR, ASSETS_DIR)


# TODO: 做唯一性检查
def generate_w1_urdf(
        arm_type: W1ArmType,
        w1_version: W1Version = W1Version.V2,
        end_effector_type: EndEffectorType = None) -> tuple[bool, str]:
    logger = get_logger()

    name = "W1"

    if w1_version != W1Version.V2:
        logger.error(f"{w1_version.name} are not support.")
        return False, None

    name += "_V2"
    if arm_type == W1ArmType.ANTHROPOMORPHIC:
        name += "_A"
    elif arm_type == W1ArmType.INDUSTRIAL:
        name += "_I"
    else:
        logger.error(f"{arm_type.name} are not support.")
        return False, None

    if end_effector_type is None:
        pass
    elif end_effector_type == EndEffectorType.BRAINCO_HAND:
        name += "_BRAINCO"
    elif end_effector_type == EndEffectorType.PGC_GRIPPER:
        name += "_PGC"
    else:
        logger.error(f"{end_effector_type.name} are not support.")
        return False, None

    file_path = ASSETS_DIR / name / f"{name}.urdf"
    if file_path.exists():
        logger.info(f"File already exists: {file_path}, skipping generation.")
        return True, str(file_path)

    import rlia
    urdf_manager = rlia.kit.toolkit.URDFAssemblyManager()

    urdf_manager.add_component("chassis",
                               str(ASSETS_DIR / "chassis/chassis.urdf"),
                               wheel_type="omni")
    urdf_manager.add_component("torso", str(ASSETS_DIR / "torso/torso.urdf"))
    urdf_manager.add_component("head", str(ASSETS_DIR / "head/head.urdf"))

    if arm_type == W1ArmType.ANTHROPOMORPHIC:
        urdf_manager.add_component("left_arm",
                                   str(ASSETS_DIR / "left_arm_a/left_arm.urdf"))
        urdf_manager.add_component(
            "right_arm", str(ASSETS_DIR / "right_arm_a/right_arm.urdf"))
    elif arm_type == W1ArmType.INDUSTRIAL:
        urdf_manager.add_component("left_arm",
                                   str(ASSETS_DIR / "left_arm_i/left_arm.urdf"))
        urdf_manager.add_component(
            "right_arm", str(ASSETS_DIR / "right_arm_i/right_arm.urdf"))

    if end_effector_type == EndEffectorType.BRAINCO_HAND:
        left_hand_attach_xpos = np.array([[-1, 0, 0, 0], [0, 0, 1, 0],
                                          [0, 1, 0, 0.02], [0, 0, 0, 1]])
        urdf_manager.add_component("left_hand",
                                   str(ASSETS_DIR /
                                       "left_brainco/left_hand.urdf"),
                                   transform=left_hand_attach_xpos)

        right_hand_attach_xpos = np.array([[1, 0, 0, 0], [0, 0, -1, 0],
                                           [0, 1, 0, 0.02], [0, 0, 0, 1]])
        urdf_manager.add_component("right_hand",
                                   str(ASSETS_DIR /
                                       "right_brainco/right_hand.urdf"),
                                   transform=right_hand_attach_xpos)
    elif end_effector_type == EndEffectorType.PGC_GRIPPER:
        urdf_manager.add_component("left_hand",
                                   str(ASSETS_DIR / "left_pgc/left_hand.urdf"),
                                   transform=np.eye(4))
        urdf_manager.add_component("right_hand",
                                   str(ASSETS_DIR /
                                       "right_pgc/right_hand.urdf"),
                                   transform=np.eye(4))

    try:
        urdf_manager.merge_urdfs(output_path=str(file_path))
    except Exception as e:
        logger.error(f"Generate W1 URDF failed: {e}")
        return False, None

    return True, str(file_path)


def download_w1_urdf(
        arm_type: W1ArmType,
        w1_version: W1Version = W1Version.V2,
        end_effector_type: EndEffectorType = None) -> tuple[bool, str]:
    logger = get_logger()

    name = "W1"

    if w1_version != W1Version.V2:
        logger.error(f"{w1_version.name} are not support.")
        return False, None

    name += "_V2"
    if arm_type == W1ArmType.ANTHROPOMORPHIC:
        name += "_A"
    elif arm_type == W1ArmType.INDUSTRIAL:
        name += "_I"
    else:
        logger.error(f"{arm_type.name} are not support.")
        return False, None

    if end_effector_type is None:
        pass
    elif end_effector_type == EndEffectorType.BRAINCO_HAND:
        name += "_BRAINCO"
        end_effector_folder_path = MODEL_DIR / "simplified_brainco"
        if not end_effector_folder_path.exists():
            end_effector_url = os.path.join(URL_PREFIX, "Model",
                                            "simplified_brainco.zip")
            download_file(url=end_effector_url, dest_path=DOWNLOAD_DIR)
            file_name = "simplified_brainco.zip"
            unzip_file(filename=file_name,
                       zip_path=DOWNLOAD_DIR,
                       unzip_path=MODEL_DIR)

    elif end_effector_type == EndEffectorType.PGC_GRIPPER:
        name += "_PGC"
        end_effector_folder_path = MODEL_DIR / "simplified_pgc"
        if not end_effector_folder_path.exists():
            end_effector_url = os.path.join(URL_PREFIX, "Model",
                                            "simplified_pgc.zip")
            download_file(url=end_effector_url, dest_path=DOWNLOAD_DIR)
            file_name = "simplified_pgc.zip"
            unzip_file(filename=file_name,
                       zip_path=DOWNLOAD_DIR,
                       unzip_path=MODEL_DIR)
    elif end_effector_type == EndEffectorType.LINKER_HAND:
        name += "_LINKER"
        end_effector_folder_path = MODEL_DIR / "simplified_linker"
        if not end_effector_folder_path.exists():
            end_effector_url = os.path.join(URL_PREFIX, "Model",
                                            "simplified_linker.zip")
            download_file(url=end_effector_url, dest_path=DOWNLOAD_DIR)
            file_name = "simplified_linker.zip"
            unzip_file(filename=file_name,
                       zip_path=DOWNLOAD_DIR,
                       unzip_path=MODEL_DIR)
    else:
        logger.error(f"{end_effector_type.name} are not support.")
        return False, None

    file_path = MODEL_DIR / name / f"{name}.urdf"
    if file_path.exists():
        logger.info(f"File already exists: {file_path}, skipping download.")
        return True, str(file_path)

    model_url = os.path.join(URL_PREFIX, "Model", f"{name}.zip")

    download_file(url=model_url, dest_path=DOWNLOAD_DIR)
    file_name = f"{name}.zip"
    unzip_file(filename=file_name, zip_path=DOWNLOAD_DIR, unzip_path=MODEL_DIR)

    return True, str(file_path)


def download_environment(env_name: str) -> tuple[bool, str]:
    """
    Download environment file from NAS to dexe_robot_client_data/assets/Environment
    """
    logger = get_logger()

    file_path = ENVIRONMENT_DIR / env_name
    if file_path.exists():
        logger.info(f"File already exists: {file_path}, skipping download.")
        return True, str(file_path)

    env_url = os.path.join(URL_PREFIX, "Environment", f"{env_name}.zip")
    download_file(url=env_url, dest_path=DOWNLOAD_DIR)
    file_name = f"{env_name}.zip"
    unzip_file(filename=file_name,
               zip_path=DOWNLOAD_DIR,
               unzip_path=ENVIRONMENT_DIR)

    return True, str(file_path)
