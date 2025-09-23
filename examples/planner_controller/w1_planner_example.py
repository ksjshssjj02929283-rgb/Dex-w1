import numpy as np
from dexe_robot_client.w1.w1_planner import W1Planner
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from IPython import embed

def test_w1_planner():
    w1_planner = W1Planner(
        is_sim=True,
        arm_type=W1ArmType.ANTHROPOMORPHIC,
        w1_version=W1Version.V2,
        end_effector_type=EndEffectorType.BRAINCO_HAND,
        use_rlia=False
    )
    res_set_home_qpos = w1_planner.set_home_qpos(np.deg2rad([45, -90, 45, 2, 
                                         0, -25,
                                         0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0]), save_to_file=False)
    



    embed()


if __name__ == "__main__":
    test_w1_planner()
