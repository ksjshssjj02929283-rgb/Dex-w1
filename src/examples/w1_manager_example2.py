'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.w1 import W1Manager

if __name__ == "__main__":
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V3,
                           end_effector_type=None)
    if w1_manager.connect() and w1_manager.is_enabled():
        # yapf: disable
        w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                            5, -45, -5, -90, 0, -15, 0,
                                            -5, 45, 5, 90, 0, 15, 0]),
                                save_to_file=False)
        # yapf: enable
        w1_manager.go_home(1.)
        from IPython import embed
        embed()  # FIXME: embed退出后会报错！
