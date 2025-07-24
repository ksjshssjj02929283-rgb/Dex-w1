# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
from functools import wraps
from .logger import get_logger


def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()  # 更高精度
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed = end_time - start_time
        get_logger().info(f"[{func.__name__}] run time: {elapsed:.6f} s")
        return result

    return wrapper


if __name__ == "__main__":

    @timer
    def test_func():
        time.sleep(1.0)

    test_func()
