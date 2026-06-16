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
import time
import multiprocessing
import threading
from concurrent.futures import ProcessPoolExecutor, as_completed
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

    def check_collision(self, q, margin=0.0):
        """
        Check if the configuration q (NQ=20) has self-collisions or is within safety margin.
        """
        # q must be (20,) array
        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data, q)
        
        if margin > 1e-6:
            # Check proximity using computeDistances
            pin.computeDistances(self.collision_model, self.collision_data)
            for dist_res in self.collision_data.distanceResults:
                if dist_res.min_distance < margin:
                    return True
            return False
        else:
            # Classic boolean collision check
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

def _parallel_worker(worker_id, joint_limits, wf, L, duration, dt, urdf_path, srdf_path, arm_indices, q_fixed, attempts_per_worker):
    import numpy as np
    import os
    import time
    import tempfile
    np.random.seed((os.getpid() * worker_id + int(time.time() * 1000)) % 123456789)
    
    local_checker = None
    if urdf_path and os.path.exists(urdf_path):
        try:
            local_checker = CollisionChecker(urdf_path, srdf_path)
        except Exception:
            pass

    counter_file = os.path.join(tempfile.gettempdir(), f"trajgen_counter_{os.getppid()}")
    
    for attempt in range(1, attempts_per_worker + 1):
        try:
            with open(counter_file, 'r') as f:
                val = int(f.read().strip())
            with open(counter_file, 'w') as f:
                f.write(str(val + 1))
        except Exception:
            pass
        t, q = nonrepeating_covering_traj(
            joint_limits=joint_limits, wf=wf, L=L,
            duration=duration, dt=dt,
            margin_ratio=0.02 
        )

        ok = True
        # 1) Joint limit check
        for i in range(len(joint_limits)):
            low, high = joint_limits[i]
            if not np.all((q[:, i] >= low) & (q[:, i] <= high)):
                ok = False
                break
        
        # 2) Self-collision check
        if ok and local_checker is not None:
            stride = max(1, int(round(0.05 / dt)))
            if q_fixed is None:
                q_full_base = np.zeros(local_checker.model.nq)
            else:
                q_full_base = q_fixed.copy()
            
            for idx in range(0, q.shape[0], stride):
                q_full = q_full_base.copy()
                q_full[arm_indices] = q[idx, :]
                if local_checker.check_collision(q_full, margin=0.0):
                    ok = False
                    break
            
            if ok and (q.shape[0]-1) % stride != 0:
                q_full = q_full_base.copy()
                q_full[arm_indices] = q[-1, :]
                if local_checker.check_collision(q_full, margin=0.0):
                    ok = False

        if ok:
            return t, q, attempt
            
    return None

def generate_q_with_limits(joint_limits, wf, L, duration, dt, collision_checker=None, arm_indices=None, q_fixed=None, max_attempts=1000, multi_arm=False, urdf_path=None, srdf_path=None):
    ndof_total = len(joint_limits)
    
    if urdf_path and os.path.exists(urdf_path):
        num_workers = multiprocessing.cpu_count()
        print(f"🚀 Spawning {num_workers} processes for parallel trajectory generation...")
        
        attempts_per_worker = max(100, max_attempts // num_workers)

        import tempfile
        counter_file = os.path.join(tempfile.gettempdir(), f"trajgen_counter_{os.getpid()}")
        with open(counter_file, 'w') as f:
            f.write('0')

        stop_monitor = threading.Event()

        def _monitor_counter():
            while not stop_monitor.is_set():
                stop_monitor.wait(1.0)
                if not stop_monitor.is_set():
                    try:
                        with open(counter_file, 'r') as f:
                            val = int(f.read().strip())
                        print(f"  ... total attempts: {val} ...")
                    except Exception:
                        pass

        monitor_thread = threading.Thread(target=_monitor_counter, daemon=True)
        monitor_thread.start()

        executor = ProcessPoolExecutor(max_workers=num_workers)
        futures = {
            executor.submit(
                _parallel_worker,
                i, joint_limits, wf, L, duration, dt,
                urdf_path, srdf_path, arm_indices, q_fixed,
                attempts_per_worker
            ): i for i in range(num_workers)
        }

        def _kill_workers(exc):
            stop_monitor.set()
            procs = list(exc._processes.values()) if exc._processes else []
            exc.shutdown(wait=False, cancel_futures=True)
            for p in procs:
                try:
                    p.terminate()
                except Exception:
                    pass

        for future in as_completed(futures):
            result = future.result()
            if result is not None:
                t, q, attempt = result
                try:
                    with open(counter_file, 'r') as f:
                        total = int(f.read().strip())
                except Exception:
                    total = attempt
                print(f"✅ Trajectory found by parallel Worker {futures[future]} on attempt {attempt}, total attempts: {total}")
                _kill_workers(executor)
                try:
                    os.remove(counter_file)
                except Exception:
                    pass
                return t, q

        _kill_workers(executor)
        try:
            os.remove(counter_file)
        except Exception:
            pass
        print("❌ All parallel processes failed after exhausting all attempts. Please try again.")
        sys.exit(1)

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

# -------------------------------
# Main Execution
# -------------------------------
if __name__ == "__main__":
    # Parameters
    duration = 60.0
    dt = 0.01
    plot = False
    wf = 0.12 * np.pi # 轨迹基频
    L = 3             # 轨迹谐波数
    ndof = 7

    # Setup Paths
    config_path = os.path.join(os.path.expanduser("~"), "dexe_robot_server_data/robot_config.yaml")
    srdf_path = None
    robot_version_key = None
    config = {}
    
    if os.path.exists(config_path):
        import yaml
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                extracted_urdf = config.get('file_path', {}).get('robot_urdf')
                if extracted_urdf:
                    ee_urdf_path = os.path.join(os.path.dirname(extracted_urdf), "robot_with_ee.urdf")
                    if os.path.exists(ee_urdf_path):
                        urdf_path = ee_urdf_path
                    else:
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

    ros2_to_urdf_map = config.get('ros2_to_urdf_map', None)

    if ros2_to_urdf_map:
        map_keys = list(ros2_to_urdf_map.keys())

        def get_urdf_qidx(urdf_joint_name):
            joint_id = model.getJointId(urdf_joint_name)
            if joint_id >= model.njoints:
                raise RuntimeError(f"Joint '{urdf_joint_name}' not found in URDF model")
            return model.joints[joint_id].idx_v

        urdf_q_indices = {k: get_urdf_qidx(v) for k, v in ros2_to_urdf_map.items()}

        torso_indices = [urdf_q_indices[k] for k in map_keys[:6]]
        left_indices = [urdf_q_indices[k] for k in map_keys[6:13]]
        right_indices = [urdf_q_indices[k] for k in map_keys[13:20]]
    else:
        torso_indices = list(range(6))
        left_indices = list(range(6, 13))
        right_indices = list(range(13, 20))

    combined_indices = left_indices + right_indices

    q_torso_rest = np.zeros(len(torso_indices))
    q_left_rest  = np.array([0.0, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5])
    q_right_rest = np.array([0.0, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5])

    arm_names = ['left', 'right']

    script_dir = Path(__file__).parent
    base_traj_dir = script_dir / 'trajectories'
    if robot_version_key:
        base_traj_dir = base_traj_dir / robot_version_key

    combined_limits = [(model.lowerPositionLimit[i], model.upperPositionLimit[i]) for i in combined_indices]

    q_fixed_base = np.zeros(model.nq)
    q_fixed_base[torso_indices] = q_torso_rest

    for seg_idx in [1, 2]:
        traj_name = f"traj{seg_idx}"
        print(f"\n--- Generating Combined {traj_name.upper()} (Both Arms) ---")
        
        try:
            # Generate combined trajectory for both arms (14 DOF) with collision detection
            t, q_combined = generate_q_with_limits(
                joint_limits=combined_limits, 
                wf=wf, L=L, 
                duration=duration, 
                dt=dt, 
                collision_checker=collision_checker,
                arm_indices=combined_indices,
                q_fixed=q_fixed_base,
                max_attempts=10000, # Increased attempts for combined check
                urdf_path=urdf_path,
                srdf_path=srdf_path
            )
            
            # Split and save for each arm
            q_left = q_combined[:, :7]
            q_right = q_combined[:, 7:]
            
            for arm_side, q_arm in zip(['left', 'right'], [q_left, q_right]):
                arm_traj_dir = base_traj_dir / arm_side
                traj_path = arm_traj_dir / f"{traj_name}.dat"
                save_q(q_arm, str(traj_path))
                
        except Exception as e:
            print(f"Error generating combined {traj_name}: {e}")
            import traceback
            traceback.print_exc()

    # After generating all segments, plot for each arm
    for arm_side in ['left', 'right']:
        arm_traj_dir = base_traj_dir / arm_side
        indices = left_indices if arm_side == 'left' else right_indices
        joint_limits = [(model.lowerPositionLimit[i], model.upperPositionLimit[i]) for i in indices]
        try:
            arms_trajectory_plot(arm_side, str(arm_traj_dir), str(arm_traj_dir), ndof, joint_limits, dt=dt)
        except Exception as e:
            print(f"Error plotting for {arm_side}: {e}")
