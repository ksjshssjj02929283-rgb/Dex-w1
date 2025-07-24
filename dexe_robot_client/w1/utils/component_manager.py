# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np

class W1ComponentManager:
    def __init__(self):
        # fmt: off
        self.real_motor_index = np.array([
                # ==== Torso Chain (0-3) [Core Stability & Postural Control] ====
                0,  # Ankle    -> Controls anterior/posterior stability
                1,  # Knee     -> Regulates torso height and posture
                2,  # Buttock  -> Manages pelvic tilt and rotation
                3,  # Loin     -> Controls trunk rotation and flexibility
                # ==== Head System (4-5) [Visual & Orientation System] ====
                4,  # Neck     -> Cervical rotation (yaw movement)
                5,  # Head     -> Cranial flexion/extension (pitch movement)
                # ==== Left Arm Chain (6-12) [7-DOF Manipulation] ====
                6,  # Left J1  -> Shoulder flexion/extension (sagittal plane)
                7,  # Left J2  -> Shoulder abduction/adduction (frontal plane)
                8,  # Left J3  -> Humeral rotation (longitudinal axis)
                9,  # Left J4  -> Elbow flexion/extension
                10,  # Left J5  -> Forearm pronation/supination
                11,  # Left J6  -> Wrist flexion/extension
                12,  # Left J7  -> Wrist radial/ulnar deviation
                # ==== Right Arm Chain (13-19) [7-DOF Manipulation] ====
                13,  # Right J1 -> Shoulder flexion/extension (sagittal plane)
                14,  # Right J2 -> Shoulder abduction/adduction (frontal plane)
                15,  # Right J3 -> Humeral rotation (longitudinal axis)
                16,  # Right J4 -> Elbow flexion/extension
                17,  # Right J5 -> Forearm pronation/supination
                18,  # Right J6 -> Wrist flexion/extension
                19,  # Right J7 -> Wrist radial/ulnar deviation
            ])
        # fmt: on
        self.torso_index = list(range(4))
        self.head_index = [4, 5]
        self.left_arm_index = list(range(6, 13))
        self.right_arm_index = list(range(13, 20))
        self.full_body_index = list(range(20))
        
        self.torso_dofs = 4
        self.head_dofs = 2
        self.arm_dofs = 7
        self.full_body_dofs = 20

        self.names = ("torso", "head", "left_arm", "right_arm", "full_body")
        self.infos = {"torso": {"dofs": self.torso_dofs, 
                                "index": self.torso_index},
                        "head": {"dofs": self.head_dofs,
                                "index": self.head_index},
                        "left_arm": {"dofs": self.arm_dofs,
                                    "index": self.left_arm_index},
                        "right_arm": {"dofs": self.arm_dofs,
                                    "index": self.right_arm_index},
                        "full_body": {"dofs": self.full_body_dofs,
                                    "index": self.full_body_index}}