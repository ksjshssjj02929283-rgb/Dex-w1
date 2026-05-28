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

    q0 = np.array([(low + high) / 2 for low, high in joint_limits])
    q[0, :] = q0

    return t, q

def generate_q_with_limits(joint_limits, wf, L, duration, dt, collision_checker=None, max_attempts=500):
    ndof = len(joint_limits)
    for attempt in range(1, max_attempts + 1):
        if attempt % 50 == 0:
            print(f"... Generation at attempt {attempt} ...")
        t, q = nonrepeating_covering_traj(
            joint_limits=joint_limits, wf=wf, L=L,
            duration=duration, dt=dt,
            margin_ratio=0.05 # 尽量覆盖限位范围，碰撞检查确保安全
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
            
            # Helper to pack full body configuration (NQ=20)
            const_prefix = np.array([0.5, -1.0, 0.5, 0.0, 0.0, 0.0], dtype=float)
            const_suffix = np.array([-0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5], dtype=float)
            
            for idx in range(0, q.shape[0], stride):
                q_full = np.zeros(20)
                q_full[0:6] = const_prefix
                q_full[6:13] = q[idx, :]
                q_full[13:20] = const_suffix
                
                if collision_checker.check_collision(q_full):
                    ok = False
                    break
            
            # Check last point if not included
            if ok and (q.shape[0]-1) % stride != 0:
                q_full = np.zeros(20)
                q_full[0:6] = const_prefix
                q_full[6:13] = q[-1, :]
                q_full[13:20] = const_suffix
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
def trajectory_plot(traj_path, ndof, joint_limits, dt=1.0, figsize=None):
    q_ref_orig = np.loadtxt(traj_path)
    if q_ref_orig.ndim == 1:
        q_ref_orig = q_ref_orig.reshape(-1, q_ref_orig.shape[0])
    
    q_ref = q_ref_orig[:, :ndof]
    s = q_ref.shape[0]
    t = np.arange(s) * dt

    ncols = int(math.ceil(math.sqrt(ndof)))
    nrows = int(math.ceil(ndof / ncols))
    if figsize is None:
        figsize = (4 * ncols, 2.5 * nrows)

    plt.close('all')
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=figsize, sharex=True)
    axes_flat = axes.flatten() if isinstance(axes, np.ndarray) else [axes]

    for i in range(ndof):
        ax = axes_flat[i]
        ax.plot(t, q_ref[:, i], label=f"$q_{i+1}$")
        low, high = joint_limits[i]
        ax.set_ylim(low, high)
        ax.set_ylabel("rad")
        if i >= ndof - ncols:
            ax.set_xlabel("time (s)" if dt != 1.0 else "sample index")
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True)
        ax.set_title(f"Joint {i+1}")

    for j in range(ndof, len(axes_flat)):
        axes_flat[j].set_visible(False)

    fig.suptitle(f"Trajectories: {os.path.basename(traj_path)}")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# -------------------------------
# Main Execution
# -------------------------------
if __name__ == "__main__":
    # Parameters
    duration = 60.0
    dt = 0.01
    traj_name = "traj_safe"
    plot = True
    wf = 0.3 * np.pi
    L = 5
    ndof = 7

    # Setup Paths
    config_path = os.path.join(os.path.expanduser("~"), "dexe_robot_server_data/robot_config.yaml")
    urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "../../install/robot_server_lib/share/robot_server_lib/robot_models/w1/robot.urdf")
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

    print(f"Loaded URDF path: {urdf_path}")
    
    # Initialize collision checker
    collision_checker = None
    if urdf_path and os.path.exists(urdf_path):
        try:
            collision_checker = CollisionChecker(urdf_path, srdf_path)
            print("Collision checker initialized.")
        except Exception as e:
            print(f"Warning: Failed to initialize collision checker: {e}")

    # Joint limits for W1 Arm (7 DOF)
    if collision_checker:
        # Get from model for precision
        model = collision_checker.model
        joint_limits = [(model.lowerPositionLimit[i+6], model.upperPositionLimit[i+6]) for i in range(ndof)]
    else:
        print("[Error] Collision checker not initialized, please check URDF path.")
        exit(1)

    print(f"Using joint limits from URDF path: {urdf_path}")
    for i, (low, high) in enumerate(joint_limits):
        print(f"  Joint {i+1}: [{low}, {high}]")
    # Paths
    script_dir = Path(__file__).parent
    traj_dir = script_dir / 'dyn_params_ident_data' / 'trajectories'
    if robot_version_key:
        traj_dir = traj_dir / robot_version_key
    traj_path = traj_dir / f"{traj_name}.dat"

    print(f"Generating trajectory: {traj_name} ({duration}s)")
    try:
        t, q = generate_q_with_limits(
            joint_limits=joint_limits, 
            wf=wf, L=L, 
            duration=duration, 
            dt=dt, 
            collision_checker=collision_checker
        )
        
        save_q(q, str(traj_path))
        
        if plot:
            trajectory_plot(str(traj_path), ndof=ndof, joint_limits=joint_limits, dt=dt)
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
