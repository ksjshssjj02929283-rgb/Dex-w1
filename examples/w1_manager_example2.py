# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1 import W1Manager


if __name__ == "__main__":
    """单点运动API move_arm_linear()和rotate_arm_linear()的使用示例"""
    w1_manager = W1Manager(is_sim=True,
                        arm_type=W1ArmType.ANTHROPOMORPHIC, 
                        w1_version=W1Version.V2, 
                        end_effector_type=EndEffectorType.BRAINCO_HAND,
                        use_rlia=False)
    if w1_manager.connect():
        w1_manager.go_home(1.)
        
        w1_manager.move_arm_linear("left_arm", [50,0,0], along_base_frame=True)
        w1_manager.rotate_arm_linear("left_arm", [10,0,0], along_base_frame=True)
        
        w1_manager.move_arm_linear("right_arm", [0,0,50], along_base_frame=False)
        w1_manager.rotate_arm_linear("right_arm", [0,0,10], along_base_frame=False)
        # from IPython import embed; embed() # FIXME: embed退出后会报错！