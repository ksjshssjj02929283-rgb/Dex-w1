'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

# import os
# import pathlib
# import zipfile
# import requests
import time
from functools import wraps
from .logger import get_logger

# from .config import DOWNLOAD_DIR, ASSETS_DIR, ENVIRONMENT_DIR, URL_PREFIX


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


# def unzip_file(filename, zip_path=DOWNLOAD_DIR, unzip_path=ASSETS_DIR) -> bool:
#     if not filename.lower().endswith(".zip"):
#         get_logger().error(f"File '{filename}' is not a ZIP file.")
#         return False

#     src_path = pathlib.Path(zip_path) / filename
#     if not src_path.exists():
#         get_logger().error(f"ZIP file not found: {src_path}")
#         return False

#     dest_path = pathlib.Path(unzip_path) / pathlib.Path(filename).stem
#     if dest_path.exists():
#         get_logger().info(
#             f"Folder already exists: {dest_path}, skipping unzip.")
#         return True

#     with zipfile.ZipFile(src_path, 'r') as zip_ref:
#         zip_ref.extractall(unzip_path)
#     get_logger().info(f"File unzip from {src_path} to {unzip_path}")
#     return True

# # TODO: 做唯一性检查
# def download_file(url: str,
#                   dest_path=DOWNLOAD_DIR,
#                   filename: str = None) -> bool:
#     if filename is None:
#         filename = os.path.basename(url)

#     file_path = pathlib.Path(dest_path) / filename
#     if file_path.exists():
#         get_logger().info(
#             f"File already exists: {file_path}, skipping download.")
#         return True

#     response = requests.get(url, timeout=5)
#     response.raise_for_status()
#     with open(file_path, "wb") as file:
#         file.write(response.content)
#     get_logger().info(f"File downloaded from {url} to {file_path}")
#     return True

# def download_and_unzip_file(url: str) -> bool:
#     download_file(url, DOWNLOAD_DIR)
#     filename = os.path.basename(url)
#     return unzip_file(filename, DOWNLOAD_DIR, ASSETS_DIR)

# def download_environment(env_name: str) -> tuple[bool, str]:
#     """
#     Download environment file from NAS to dexe_robot_client_data/assets/Environment
#     """
#     logger = get_logger()

#     file_path = ENVIRONMENT_DIR / env_name
#     if file_path.exists():
#         logger.info(f"File already exists: {file_path}, skipping download.")
#         return True, str(file_path)

#     env_url = os.path.join(URL_PREFIX, "Environment", f"{env_name}.zip")
#     download_file(url=env_url, dest_path=DOWNLOAD_DIR)
#     file_name = f"{env_name}.zip"
#     unzip_file(filename=file_name,
#                zip_path=DOWNLOAD_DIR,
#                unzip_path=ENVIRONMENT_DIR)

#     return True, str(file_path)
