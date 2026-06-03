#!/usr/bin/python3
# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import os
import sys
import math
import numpy as np
import yaml
import pinocchio as pin
from pathlib import Path
from scipy.signal import butter, filtfilt
from matplotlib import pyplot as plt

# -------------------------------
# 1) Trajectory Generation Logic
# -------------------------------
class CollisionChecker:
    def __init__(self, urdf_path, srdf_path=None):
        """
        Initialize the Pinocchio-based collision checker.
        """
        self.model = pin.buildModelFromUrdf(urdf_path)
        mesh_dir = os.path.dirname(urdf_path)
        # buildGeomFromUrdf can take mesh_dir as package path
        self.collision_model = pin.buildGeomFromUrdf(self.model, urdf_path, pin.GeometryType.COLLISION, mesh_dir)
        self.collision_model.addAllCollisionPairs()
        
        if srdf_path and os.path.exists(srdf_path):
            pin.removeCollisionPairs(self.model, self.collision_model, srdf_path)
        
        self.collision_data = pin.GeometryData(self.collision_model)
        self.data = self.model.createData()

        # Fallback/Safety: Remove any pairs that are in collision at the neutral configuration
        # This handles adjacent links not covered by SRDF or mesh overlaps in default pose.
        q_neutral = pin.neutral(self.model)
        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data, q_neutral)
        # Compute all collisions
        pin.computeCollisions(self.collision_model, self.collision_data, False)
        
        # Collect and remove pairs from BOTH model and data
        pairs_to_remove_indices = []
        for i in range(len(self.collision_model.collisionPairs)):
            if self.collision_data.collisionResults[i].isCollision():
                pairs_to_remove_indices.append(i)
        
        # Remove pairs (in reverse order to keep indices valid)
        for i in reversed(pairs_to_remove_indices):
            self.collision_model.removeCollisionPair(self.collision_model.collisionPairs[i])
        
        # Re-create collision data after removing pairs
        self.collision_data = pin.GeometryData(self.collision_model)
        
        if pairs_to_remove_indices:
            print(f"Removed {len(pairs_to_remove_indices)} naturally colliding pairs (SRDF/Adjacent).")

    def check_collision(self, q):
        """
        Check if the configuration q (NQ=20) has self-collisions.
        """
        # q must be (20,) array
        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data, q)
        return pin.computeCollisions(self.collision_model, self.collision_data, False)

def nonrepeating_covering_traj(joint_limits, wf, L, duration, dt, margin_ratio=0.03, filt_cutoff=None):
    """
    Generate non-repeating trajectory covering joint limits.
    """
    ndof = len(joint_limits)
    t = np.arange(0.0, duration, dt)
    q_raw = np.zeros((t.size, ndof))

    rng = np.random.default_rng()
    for k in range(ndof):
        u = rng.uniform(0.15, 0.85, size=L)
        freqs = wf * (np.arange(1, L+1) + u)
        phis = rng.uniform(0.0, 2*np.pi, size=L)
        amps = rng.uniform(0.6, 1.2, size=L)

        signal = np.zeros_like(t)
        for l in range(L):
            signal += (amps[l] / freqs[l]) * np.sin(freqs[l] * t + phis[l])
        q_raw[:, k] = signal

    if filt_cutoff is not None:
        b, a = butter(3, filt_cutoff, btype='low', fs=1.0/dt)
        q_raw = filtfilt(b, a, q_raw, axis=0)

    q = np.zeros_like(q_raw)
    for k, (low, high) in enumerate(joint_limits):
        margin = (high - low) * margin_ratio
        target_low = low + margin
        target_high = high - margin

        max_abs = np.max(np.abs(q_raw[:, k]))
        if max_abs < 1e-8:
            q_raw[:, k] = q_raw[:, k] + rng.normal(0, 1e-3, size=q_raw.shape[0])
            max_abs = np.max(np.abs(q_raw[:, k]))
        norm = q_raw[:, k] / max_abs
        q[:, k] = target_low + (norm + 1.0) * 0.5 * (target_high - target_low)

    return t, q

def generate_q_with_limits(joint_limits, wf, L, duration, dt, collision_checker=None, arm_indices=None, q_fixed=None, max_attempts=500):
    ndof = len(joint_limits)
    for attempt in range(1, max_attempts + 1):
        if attempt % 50 == 0:
            print(f"... Generation at attempt {attempt} ...")
        t, q = nonrepeating_covering_traj(
            joint_limits=joint_limits, wf=wf, L=L,
            duration=duration, dt=dt,
            margin_ratio=0.02 # 缩小边距比例，使轨迹覆盖 96% 的关节范围
        )

        ok = True
        # 1) Joint limit check
        for i in range(ndof):
            low, high = joint_limits[i]
            if not np.all((q[:, i] >= low) & (q[:, i] <= high)):
                ok = False
                break
        
        # 2) Self-collision check (if checker provided)
        if ok and collision_checker is not None:
            # Check collisions roughly every 50ms to save time
            stride = max(1, int(round(0.05 / dt)))
            
            # Use provided q_fixed as base for full body configuration
            if q_fixed is None:
                q_full_base = np.zeros(collision_checker.model.nq)
            else:
                q_full_base = q_fixed.copy()
            
            if arm_indices is None:
                arm_indices = range(6, 13)

            for idx in range(0, q.shape[0], stride):
                q_full = q_full_base.copy()
                q_full[arm_indices] = q[idx, :]
                
                if collision_checker.check_collision(q_full):
                    ok = False
                    break
            
            # Check last point if not included
            if ok and (q.shape[0]-1) % stride != 0:
                q_full = q_full_base.copy()
                q_full[arm_indices] = q[-1, :]
                if collision_checker.check_collision(q_full):
                    ok = False

        if ok:
            if collision_checker:
                print(f"✅ Trajectory within limits and collision-free on attempt {attempt}")
            else:
                print(f"✅ Trajectory within limits on attempt {attempt}")
            return t, q
    
    raise RuntimeError("Failed to generate a valid trajectory within joint limits.")

def save_q(q, out_dir):
    os.makedirs(os.path.dirname(out_dir), exist_ok=True)
    np.savetxt(out_dir, q, fmt="%.6f")
    print(f"Saved to: {out_dir}")

# -------------------------------
# 2) Plotting Logic
# -------------------------------
def arms_trajectory_plot(arm_side, data_dir, save_dir, ndof, joint_limits, dt=0.01):
    """
    Plot two trajectory segments for one arm side in a large figure.
    Layout: Rows = Segments, Columns = Joints
    """
    traj_paths = [os.path.join(data_dir, f"traj{i}.dat") for i in [1, 2]]
    
    # Check if files exist
    valid_paths = [p for p in traj_paths if os.path.exists(p)]
    if not valid_paths:
        print(f"No trajectory files found for {arm_side} at {data_dir}")
        return

    # Rows are segments, Columns are joints
    nrows = len(valid_paths)
    ncols = ndof
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(2.8 * ncols, 3.5 * nrows), sharex='row')
    
    if nrows == 1:
        axes = axes.reshape(1, -1)

    for row, path in enumerate(valid_paths):
        q_ref = np.loadtxt(path)
        if q_ref.ndim == 1:
            q_ref = q_ref.reshape(-1, q_ref.shape[0])
        q_ref = q_ref[:, :ndof]
        t = np.arange(q_ref.shape[0]) * dt
        
        for col in range(ndof):
            ax = axes[row, col]
            ax.plot(t, q_ref[:, col], color='C0')
            low, high = joint_limits[col]
            ax.set_ylim(low - 0.2, high + 0.2)
            
            if col == 0:
                ax.set_ylabel(f"Seg {row+1}\nrad", fontsize='small')
            if row == 0:
                ax.set_title(f"Joint {col+1}", fontsize='medium')
            if row == nrows - 1:
                ax.set_xlabel("Time (s)", fontsize='small')
            ax.grid(True, linestyle='--', alpha=0.7)

    fig.suptitle(f"{arm_side.upper()} ARM Trajectories Comparison", fontsize='large')
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    save_path = os.path.join(save_dir, f"{arm_side}_traj.png")
    plt.savefig(save_path)
    print(f"Saved plot to: {save_path}")
    plt.show()

# -------------------------------
# Main Execution
# -------------------------------
if __name__ == "__main__":
    # Parameters
    duration = 60.0
    dt = 0.01
    plot = False
    wf = 0.15 * np.pi # 降低基频权重（原为 0.3 * np.pi），减缓轨迹波动频率
    L = 3             # 减少叠加的正弦波项数（原为 5），使轨迹更平滑
    ndof = 7

    # Setup Paths
    config_path = os.path.join(os.path.expanduser("~"), "dexe_robot_server_data/robot_config.yaml")
    srdf_path = None
    robot_version_key = None
    
    if os.path.exists(config_path):
        import yaml
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                extracted_urdf = config.get('file_path', {}).get('robot_urdf')
                if extracted_urdf:
                    urdf_path = extracted_urdf
                    srdf_path = urdf_path.replace('.urdf', '.srdf')
                robot_type = config.get("robot_type", "")
                version = config.get("version", "")
                if robot_type and version:
                    robot_version_key = f"{robot_type}_{version[:3]}"
        except Exception as e:
            print(f"Warning: Could not load config from {config_path}: {e}")

    # Fallback for urdf_path if not found
    if not os.path.exists(urdf_path):
        print(f"Warning: URDF not found at {urdf_path}, using default path.")
        raise RuntimeError(f"URDF not found at {urdf_path}")

    print(f"Loaded URDF path: {urdf_path}")
    
    # Initialize collision checker
    collision_checker = None
    if urdf_path and os.path.exists(urdf_path):
        try:
            collision_checker = CollisionChecker(urdf_path, srdf_path)
            print("Collision checker initialized.")
        except Exception as e:
            print(f"Warning: Failed to initialize collision checker: {e}")

    if not collision_checker:
        print("[Error] Collision checker not initialized, please check URDF path.")
        exit(1)

    model = collision_checker.model
    
    # Resting positions (用于碰撞检测)
    q_torso_rest = np.array([0.5, -1.0, 0.5, 0.0, 0.0, 0.0])
    q_left_rest  = np.array([0.0, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5])
    q_right_rest = np.array([0.0, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5])

    # Define arms
    arms = {
        'left': {
            'indices': range(6, 13),
            'q_fixed': np.concatenate([q_torso_rest, np.zeros(7), q_right_rest])
        },
        'right': {
            'indices': range(13, 20),
            'q_fixed': np.concatenate([q_torso_rest, q_left_rest, np.zeros(7)])
        }
    }

    # Output directory
    script_dir = Path(__file__).parent
    base_traj_dir = script_dir / 'trajectories'
    if robot_version_key:
        base_traj_dir = base_traj_dir / robot_version_key

    for arm_side, arm_config in arms.items():
        print(f"\n--- Generating for {arm_side.upper()} ARM ---")
        indices = arm_config['indices']
        q_fixed = arm_config['q_fixed']
        
        # Get joint limits for this arm
        joint_limits = [(model.lowerPositionLimit[i], model.upperPositionLimit[i]) for i in indices]
        
        arm_traj_dir = base_traj_dir / arm_side
        for seg_idx in [1, 2]:
            traj_name = f"traj{seg_idx}"
            traj_path = arm_traj_dir / f"{traj_name}.dat"
            
            print(f"Generating {traj_name} ({duration}s)...")
            try:
                t, q = generate_q_with_limits(
                    joint_limits=joint_limits, 
                    wf=wf, L=L, 
                    duration=duration, 
                    dt=dt, 
                    collision_checker=collision_checker,
                    arm_indices=indices,
                    q_fixed=q_fixed
                )
                
                save_q(q, str(traj_path))
                
            except Exception as e:
                print(f"Error generating for {arm_side} {traj_name}: {e}")
                import traceback
                traceback.print_exc()

        # After generating both segments, plot them together
        try:
            arms_trajectory_plot(arm_side, str(arm_traj_dir), str(arm_traj_dir), ndof, joint_limits, dt=dt)
        except Exception as e:
            print(f"Error plotting for {arm_side}: {e}")
