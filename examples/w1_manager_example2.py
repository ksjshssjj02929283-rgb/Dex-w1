# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from rlia.kit.drive_controllers.w1_drive_controller import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1 import W1Manager


if __name__ == "__main__":
    from IPython import embed

    w1_manager = W1Manager(is_sim=True,
                        arm_type=W1ArmType.INDUSTRIAL, 
                        w1_version=W1Version.V2, 
                        end_effector_type=None)
    # w1_manager.go_home(1.)
    
    # w1_manager.move_arm_linear("right_arm", [0,10,0], along_base_frame=True)
    # w1_manager.move_arm_linear("right_arm", [0,0,10], along_base_frame=False)
    # w1_manager.rotate_arm_linear("right_arm", [5,0,0], along_base_frame=False)
    embed() # FIXME: embed退出后会报错！