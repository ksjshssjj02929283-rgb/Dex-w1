'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
import os
import csv
import time
from .fcl_checker import *
from .ompl_plan import *
from .planner_pinocchio import Planner_Pin
from ...common import get_logger
import matplotlib.pyplot as plt
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.heuristic import *
from scipy.interpolate import interp1d
import pinocchio
import ompl.base as ob
import ompl.geometric as og
from IPython import embed


class ForwardMotionValidator(ob.MotionValidator):
    """
    This is a inherited class from ompl.base.MotionValidator, which overrides the checkMothon() function.
    It will check the motion is towards the forward direction only and 
    check the motion validity by sampling according to a fixed distance resolution or according to the relative percentage resolution.
    """

    def __init__(self, si):
        super().__init__(si)
        self.si = si  # 保留 SpaceInformation 对象以便需要 checkMotion()
        self.res = 0.5

    def checkMotion(self, s1, s2):
        x1, y1 = s1[0], s1[1]
        x2, y2 = s2[0], s2[1]

        if x2 < x1 or y2 < y1:
            return False
        """According to the relative percentage resolution of each step to check"""
        # res = self.si.getStateValidityCheckingResolution()
        # steps = int(1/res)
        # for i in range(1, steps+1):
        #     interp = self.si.getStateSpace().allocState()
        #     self.si.getStateSpace().interpolate(s1, s2, i/steps, interp)
        #     if not self.si.isValid(interp):
        #         return False

        dist = self.si.distance(s1, s2)
        steps = max(1, int(dist / self.res))
        for i in range(1, steps + 1):
            interp = self.si.getStateSpace().allocState()
            self.si.getStateSpace().interpolate(s1, s2, i / steps, interp)
            if not self.si.isValid(interp):
                return False

        return True


class GridRRT:

    def __init__(self,
                 grid_map,
                 left_traj,
                 right_traj,
                 left_idx_chain,
                 right_idx_chain,
                 fcl_checker: FCLChecker,
                 planner_pin: Planner_Pin,
                 use_ee: bool = False):
        self.space = ob.RealVectorStateSpace(2)
        self.bounds = ob.RealVectorBounds(2)
        for i in range(2):
            self.bounds.setLow(i, 0)
            self.bounds.setHigh(i, grid_map.shape[i] - 1)
        self.space.setBounds(self.bounds)
        self.start_state = np.array([0, 0], dtype=np.int16)
        self.goal_state = np.array(
            [grid_map.shape[0] - 1, grid_map.shape[1] - 1], dtype=np.int16)
        self.grid_map = grid_map

        self.left_traj = left_traj
        self.right_traj = right_traj
        self.left_idx_chain = left_idx_chain
        self.right_idx_chain = right_idx_chain

        self.fcl_checker = fcl_checker
        self.planner_pin = planner_pin

        if fcl_checker.end_effector_type != planner_pin.ee_type:
            raise ValueError(
                f"FCL Checker end effector type {fcl_checker.end_effector_type} does not match Planner Pinocchio end effector type {planner_pin.ee_type}"
            )
        else:
            self.end_effector_type = fcl_checker.end_effector_type

        self.flag_map = np.zeros(
            (self.grid_map.shape[0], self.grid_map.shape[1]))

        self.use_ee = use_ee

        self.logger = get_logger()

    def pos_to_idx(self, state):
        state = np.array([state[i] for i in range(2)], dtype=np.float64)
        state = np.floor(state + 0.5).astype(int)
        return state[0], state[1]

    def set_head_body(self, head_qpos, body_qpos):
        if head_qpos.shape[0] != 2:
            raise ValueError(
                f"head_qpos must be a 2-element array, got {head_qpos.shape}")
        if body_qpos.shape[0] != 4:
            raise ValueError(
                f"body_qpos must be a 4-element array, got {body_qpos.shape}")
        self.head_qpos = head_qpos
        self.body_qpos = body_qpos
        arml_dim = self.left_traj.shape[1]
        armr_dim = self.right_traj.shape[1]
        temp_arml_qpos = np.zeros(arml_dim)
        temp_armr_qpos = np.zeros(armr_dim)
        real_qpos = np.concatenate(
            (self.body_qpos, self.head_qpos, temp_arml_qpos, temp_armr_qpos))
        torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(
            real_qpos)
        self.fcl_checker.set_head_body_transform(head_trans, torso_trans)

    def set_end_effector_qpos(self, armlee_qpos: np.ndarray,
                              armree_qpos: np.ndarray):
        if self.use_ee:
            if self.end_effector_type == EndEffectorType.BRAINCO_HAND:
                if armlee_qpos.shape != (6,) or armree_qpos.shape != (6,):
                    raise ValueError(
                        f"For BRAINCO_HAND, end_effector_qpos must be a 6-element array, got {armlee_qpos.shape} and {armree_qpos.shape}"
                    )
            if self.end_effector_type == EndEffectorType.PGC_GRIPPER:
                if armlee_qpos.shape != (2,) or armree_qpos.shape != (2,):
                    raise ValueError(
                        f"For PGC_GRIPPER, end_effector_qpos must be a 2-element array, got {armlee_qpos.shape} and {armree_qpos.shape}"
                    )
            self.armlee_qpos = armlee_qpos
            self.armree_qpos = armree_qpos

    def _update_fcl_transform(self, arml_qpos, armr_qpos):
        if not self.use_ee:
            real_qpos = np.concatenate(
                (self.body_qpos, self.head_qpos, arml_qpos, armr_qpos))
            torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(
                real_qpos)
            transform = arml_trans + armr_trans
            self.fcl_checker.update_arm_transform(transform)
        else:
            real_qpos = np.concatenate(
                (self.body_qpos, self.head_qpos, arml_qpos, armr_qpos))
            torso_trans, head_trans, arml_trans, armr_trans, armlee_trans, armree_trans = self.planner_pin.get_Kinematics(
                real_qpos, self.armlee_qpos, self.armree_qpos, compute_ee=True)
            transform = arml_trans + armr_trans
            self.fcl_checker.update_arm_transform(transform, armlee_trans,
                                                  armree_trans)

    def _is_state_valid(self, state):
        x, y = self.pos_to_idx(state)
        if self.flag_map[x, y] == 0:
            self.flag_map[x, y] = 1
            left_qpos = self.left_traj[self.left_idx_chain[x], :]
            right_qpos = self.right_traj[self.right_idx_chain[y], :]
            self._update_fcl_transform(left_qpos, right_qpos)
            is_collide = self.fcl_checker.check_collision_between_arms()
            if is_collide:
                self.grid_map[x, y] = 0
                return False
            else:
                return True
        else:
            if self.grid_map[x, y] == 0:
                return False
            else:
                return True

    def get_grid_map(self):
        return self.grid_map

    def get_flag_map(self):
        return self.flag_map

    def plan(self, planning_time=1.0):
        si = ob.SpaceInformation(self.space)
        si.setStateValidityCheckingResolution(0.1)
        si.setStateValidityChecker(
            ob.StateValidityCheckerFn(self._is_state_valid))
        motion_validator = ForwardMotionValidator(si)
        si.setMotionValidator(motion_validator)
        si.setup()

        pdef = ob.ProblemDefinition(si)
        si_start = si.allocState()
        si_goal = si.allocState()
        for i in range(self.space.getDimension()):
            si_start[i] = int(self.start_state[i])
            si_goal[i] = int(self.goal_state[i])
        pdef.setStartAndGoalStates(si_start, si_goal)

        planner = og.RRTConnect(si)
        planner.setRange(5)
        planner.setProblemDefinition(pdef)
        planner.setup()

        start_time = time.time()
        solved = planner.solve(planning_time)
        end_time = time.time()
        # self.logger.info(f"Planning time: {end_time - start_time:.6f} seconds")
        RRT_time = end_time - start_time
        if end_time - start_time > planning_time:
            # raise RuntimeError(f"Warning: Planning took longer than the specified time limit of {planning_time} seconds.")
            self.logger.warning(
                f"Warning: Planning took longer than the specified time limit of {planning_time} seconds."
            )
            self.logger.info("Replanning...")
            path, total_time, original_path = self.plan(
                planning_time=planning_time)
            return path, total_time, original_path

        if solved:
            # self.logger.info(f"Successfully found a path with {pdef.getSolutionPath().getStateCount()} states.")
            path = pdef.getSolutionPath()
            original_path = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                original_path.append((state[0], state[1]))
            # si.setStateValidityCheckingResolution(0.005)
            start_time = time.time()
            path_simplifier = og.PathSimplifier(si)
            path_simplifier.reduceVertices(path)
            end_time = time.time()
            Simplify_time = end_time - start_time
            # self.logger.info(f"Path simplification time: {end_time - start_time:.6f} seconds")
            # path_simplifier.smoothBSpline(path)
            # path.interpolate(50)
            total_time = RRT_time + Simplify_time
            return path, total_time, original_path
        else:
            self.logger.warning("Failed to find a path.")
            return None

    def set_ee_identifier(self, armlee_start_qpos: np.ndarray,
                          armree_start_qpos: np.ndarray,
                          left_identifier_list: list[tuple[int, np.ndarray]],
                          right_identifier_list: list[tuple[int, np.ndarray]]):
        if len(left_identifier_list) == 0 or left_identifier_list is None:
            raise ValueError("left_identifier_list must be a non-empty list")
        if len(right_identifier_list) == 0 or right_identifier_list is None:
            raise ValueError("right_identifier_list must be a non-empty list")

        self.armlee_start_qpos, self.armree_start_qpos = armlee_start_qpos, armree_start_qpos

        self.left_ee_trigger, self.right_ee_trigger = [], []
        self.left_ee_qpos_list, self.right_ee_qpos_list = [], []
        for idx, ee_qpos in left_identifier_list:
            self.left_ee_trigger.append(idx)
            self.left_ee_qpos_list.append(ee_qpos)
        for idx, ee_qpos in right_identifier_list:
            self.right_ee_trigger.append(idx)
            self.right_ee_qpos_list.append(ee_qpos)

    def _find_ee_qpos(self, index: int, which_arm: str):
        # NOTE: The count of connected trajectories won't be very large, so use linear search instead of binary search
        if which_arm == "left":
            for i in range(len(self.left_ee_trigger)):
                if index <= self.left_ee_trigger[i]:
                    if i == 0:
                        return self.armlee_start_qpos
                    else:
                        return self.left_ee_qpos_list[i - 1]
            return self.left_ee_qpos_list[-1]
        elif which_arm == "right":
            for i in range(len(self.right_ee_trigger)):
                if index <= self.right_ee_trigger[i]:
                    if i == 0:
                        return self.armree_start_qpos
                    else:
                        return self.right_ee_qpos_list[i - 1]
            return self.right_ee_qpos_list[-1]
        else:
            raise ValueError(
                f"which_arm must be 'left' or 'right', got {which_arm}")

    def _is_state_valid_with_ee(self, state):
        x, y = self.pos_to_idx(state)
        cur_left_ee_qpos = self._find_ee_qpos(self.left_idx_chain[x], "left")
        cur_right_ee_qpos = self._find_ee_qpos(self.right_idx_chain[y], "right")
        if self.flag_map[x, y] == 0:
            self.flag_map[x, y] = 1
            left_qpos = self.left_traj[self.left_idx_chain[x], :]
            right_qpos = self.right_traj[self.right_idx_chain[y], :]
            real_qpos = np.concatenate(
                (self.body_qpos, self.head_qpos, left_qpos, right_qpos))
            torso_trans, head_trans, arml_trans, armr_trans, armlee_trans, armree_trans = self.planner_pin.get_Kinematics(
                real_qpos, cur_left_ee_qpos, cur_right_ee_qpos, compute_ee=True)
            transform = arml_trans + armr_trans
            self.fcl_checker.update_arm_transform(transform, armlee_trans,
                                                  armree_trans)
            is_collide = self.fcl_checker.check_collision_between_arms()
            if is_collide:
                self.grid_map[x, y] = 0
                return False
            else:
                return True
        else:
            if self.grid_map[x, y] == 0:
                return False
            else:
                return True

    def plan_with_ee(self, planning_time=1.0):
        si = ob.SpaceInformation(self.space)
        si.setStateValidityCheckingResolution(0.1)
        si.setStateValidityChecker(
            ob.StateValidityCheckerFn(self._is_state_valid_with_ee))
        motion_validator = ForwardMotionValidator(si)
        si.setMotionValidator(motion_validator)
        si.setup()

        pdef = ob.ProblemDefinition(si)
        si_start = si.allocState()
        si_goal = si.allocState()
        for i in range(self.space.getDimension()):
            si_start[i] = int(self.start_state[i])
            si_goal[i] = int(self.goal_state[i])
        pdef.setStartAndGoalStates(si_start, si_goal)

        planner = og.RRTConnect(si)
        planner.setRange(5)
        planner.setProblemDefinition(pdef)
        planner.setup()

        start_time = time.time()
        solved = planner.solve(planning_time)
        end_time = time.time()
        # self.logger.info(f"Planning time: {end_time - start_time:.6f} seconds")
        RRT_time = end_time - start_time
        if end_time - start_time > planning_time:
            # raise RuntimeError(f"Warning: Planning took longer than the specified time limit of {planning_time} seconds.")
            self.logger.warning(
                f"Warning: Planning took longer than the specified time limit of {planning_time} seconds."
            )
            self.logger.info("Replanning...")
            path, total_time, original_path = self.plan(
                planning_time=planning_time)
            return path, total_time, original_path

        if solved:
            # self.logger.info(f"Successfully found a path with {pdef.getSolutionPath().getStateCount()} states.")
            path = pdef.getSolutionPath()
            original_path = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                original_path.append((state[0], state[1]))
            # si.setStateValidityCheckingResolution(0.005)
            start_time = time.time()
            path_simplifier = og.PathSimplifier(si)
            path_simplifier.reduceVertices(path)
            end_time = time.time()
            Simplify_time = end_time - start_time
            # self.logger.info(f"Path simplification time: {end_time - start_time:.6f} seconds")
            # path_simplifier.smoothBSpline(path)
            # path.interpolate(50)
            total_time = RRT_time + Simplify_time
            return path, total_time, original_path
        else:
            self.logger.warning("Failed to find a path.")
            return None


class Coordination:

    def __init__(self,
                 left_traj,
                 right_traj,
                 fcl_checker: FCLChecker,
                 planner_pin: Planner_Pin,
                 coordination_res=0.1,
                 traj_res=0.01,
                 use_ee: bool = False):
        # coordination_res: resolution of coordination map, unit: s
        # traj_res: resolution of trajectory(dt between two waypoints), unit: s
        self.left_traj = left_traj
        self.right_traj = right_traj
        self.fcl_checker = fcl_checker
        if coordination_res <= traj_res:
            raise ValueError(
                f"coordination_res ({coordination_res}) must be greater than traj_res ({traj_res})"
            )
        self.coordination_res = coordination_res
        self.traj_res = traj_res
        self.planner_pin = planner_pin
        self.use_ee = use_ee

        # self.pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        # self.pin_data = self.pin_model.createData()

        self.head_qpos = None
        self.body_qpos = None

        if fcl_checker.end_effector_type != planner_pin.ee_type:
            raise ValueError(
                f"FCL Checker end effector type {fcl_checker.end_effector_type} does not match Planner Pinocchio end effector type {planner_pin.ee_type}"
            )
        else:
            self.end_effector_type = fcl_checker.end_effector_type

        self.idx_increment = int(coordination_res / traj_res)
        if self.left_traj.shape[0] % self.idx_increment != 0:
            self.len_vert = self.left_traj.shape[0] // self.idx_increment + 2
        else:
            self.len_vert = self.left_traj.shape[0] // self.idx_increment + 1
        if self.right_traj.shape[0] % self.idx_increment != 0:
            self.len_hori = self.right_traj.shape[0] // self.idx_increment + 2
        else:
            self.len_hori = self.right_traj.shape[0] // self.idx_increment + 1
        self.left_idx_chain = np.arange(0, self.left_traj.shape[0],
                                        self.idx_increment)
        self.left_idx_chain = np.append(self.left_idx_chain,
                                        self.left_traj.shape[0] - 1)
        self.right_idx_chain = np.arange(0, self.right_traj.shape[0],
                                         self.idx_increment)
        self.right_idx_chain = np.append(self.right_idx_chain,
                                         self.right_traj.shape[0] - 1)

        self.grid_map = np.ones((self.len_vert, self.len_hori), dtype=np.int8)

        self.logger = get_logger()
        # self.logger.info(f"Get original {self.left_traj.shape[0]} waypoints for left arm and {self.right_traj.shape[0]} waypoints for right arm")
        # self.logger.info(f"Construct coordination space with {self.len_vert} * {self.len_hori}")

    def set_head_body(self, head_qpos, body_qpos):
        if head_qpos.shape[0] != 2:
            raise ValueError(
                f"head_qpos must be a 2-element array, got {head_qpos.shape}")
        if body_qpos.shape[0] != 4:
            raise ValueError(
                f"body_qpos must be a 4-element array, got {body_qpos.shape}")
        self.head_qpos = head_qpos
        self.body_qpos = body_qpos
        arml_dim = self.left_traj.shape[1]
        armr_dim = self.right_traj.shape[1]
        temp_arml_qpos = np.zeros(arml_dim)
        temp_armr_qpos = np.zeros(armr_dim)
        real_qpos = np.concatenate(
            (self.body_qpos, self.head_qpos, temp_arml_qpos, temp_armr_qpos))
        torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(
            real_qpos)
        self.fcl_checker.set_head_body_transform(head_trans, torso_trans)

    def set_end_effector_qpos(self, armlee_qpos: np.ndarray,
                              armree_qpos: np.ndarray):
        if self.use_ee:
            if self.end_effector_type == EndEffectorType.BRAINCO_HAND:
                if armlee_qpos.shape != (6,) or armree_qpos.shape != (6,):
                    raise ValueError(
                        f"For BRAINCO_HAND, end_effector_qpos must be a 6-element array, got {armlee_qpos.shape} and {armree_qpos.shape}"
                    )
            if self.end_effector_type == EndEffectorType.PGC_GRIPPER:
                if armlee_qpos.shape != (2,) or armree_qpos.shape != (2,):
                    raise ValueError(
                        f"For PGC_GRIPPER, end_effector_qpos must be a 2-element array, got {armlee_qpos.shape} and {armree_qpos.shape}"
                    )
            self.armlee_qpos = armlee_qpos
            self.armree_qpos = armree_qpos

    def _update_fcl_transform(self, arml_qpos, armr_qpos):
        if not self.use_ee:
            real_qpos = np.concatenate(
                (self.body_qpos, self.head_qpos, arml_qpos, armr_qpos))
            torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(
                real_qpos)
            transform = arml_trans + armr_trans
            self.fcl_checker.update_arm_transform(transform)
        else:
            real_qpos = np.concatenate(
                (self.body_qpos, self.head_qpos, arml_qpos, armr_qpos))
            torso_trans, head_trans, arml_trans, armr_trans, armlee_trans, armree_trans = self.planner_pin.get_Kinematics(
                real_qpos, self.armlee_qpos, self.armree_qpos, compute_ee=True)
            transform = arml_trans + armr_trans
            self.fcl_checker.update_arm_transform(transform, armlee_trans,
                                                  armree_trans)

    def _align_traj(self, left_traj, right_traj):
        if left_traj.shape[0] == right_traj.shape[0]:
            return left_traj, right_traj
        elif left_traj.shape[0] > right_traj.shape[0]:
            insert_num = left_traj.shape[0] - right_traj.shape[0]
            insert_row = right_traj[-1, :].copy()
            for i in range(insert_num):
                right_traj = np.append(right_traj, [insert_row], axis=0)
        else:
            insert_num = right_traj.shape[0] - left_traj.shape[0]
            insert_row = left_traj[-1, :].copy()
            for i in range(insert_num):
                left_traj = np.append(left_traj, [insert_row], axis=0)
        # self.logger.info(f"Aligned left trajectory length: {left_traj.shape[0]}")
        # self.logger.info(f"Aligned right trajectory length: {right_traj.shape[0]}")
        return left_traj, right_traj

    def Coordinate_Astar(self):
        """
        This method constructs the coordination space map first(can add obstacle expansion).
        Then use A* to find a path in the coordination space.
        Due to the A* algorithm's nature, there will be many stop points for one of the arms(discontinuous)
        But it will provide a function to manually add some stop points
        """
        start_time = time.time()
        # Construct the coordination space map
        for i in range(self.len_vert):
            for j in range(self.len_hori):
                left_qpos = self.left_traj[self.left_idx_chain[i], :]
                right_qpos = self.right_traj[self.right_idx_chain[j], :]
                self._update_fcl_transform(left_qpos, right_qpos)
                is_collide = self.fcl_checker.check_collision_between_arms()
                if is_collide:
                    self.grid_map[i, j] = 0
                else:
                    self.grid_map[i, j] = 1
        end_time = time.time()
        self.logger.info(
            f"Constructed coordination space map used {end_time - start_time:.6f} seconds"
        )

        grid = Grid(matrix=self.grid_map)
        start = grid.node(0, 0)
        end = grid.node(self.len_hori - 1, self.len_vert - 1)
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always,
                             weight=1.0)
        finder.heuristic = chebyshev

        path, runs = finder.find_path(start, end, grid)
        if not path:
            self.logger.warning(
                "Failed to find a path in the coordination space")
            return None, None
        px, py = zip(*path) if path else ([], [])
        px = np.array(px)
        py = np.array(py)

        # Plot the grid map and path in coordination space
        self.plot_grid_map_Astar(self.grid_map, start, end, px, py)

        left_traj = self.left_traj.copy()
        right_traj = self.right_traj.copy()
        left_idx_chain = self.left_idx_chain.copy()
        right_idx_chain = self.right_idx_chain.copy()
        for i in range(len(py) - 1):
            if py[i + 1] == py[i]:
                for j in range(self.idx_increment):
                    insert_pos = left_idx_chain[py[i]] + j + 1
                    row_to_insert = left_traj[insert_pos, :]
                    left_traj = np.insert(left_traj,
                                          insert_pos,
                                          row_to_insert,
                                          axis=0)
                    for k in range(len(left_idx_chain)):
                        if left_idx_chain[k] >= insert_pos:
                            left_idx_chain[k] += 1
            if px[i + 1] == px[i]:
                for j in range(self.idx_increment):
                    insert_pos = right_idx_chain[px[i]] + j + 1
                    row_to_insert = right_traj[insert_pos, :]
                    right_traj = np.insert(right_traj,
                                           insert_pos,
                                           row_to_insert,
                                           axis=0)
                    for k in range(len(right_idx_chain)):
                        if right_idx_chain[k] >= insert_pos:
                            right_idx_chain[k] += 1
        aligned_left, aligned_right = self._align_traj(left_traj, right_traj)
        self.logger.info(
            f"The coordinated trajectories have {aligned_left.shape[0]} waypoints for left arm and {aligned_right.shape[0]} waypoints for right arm"
        )
        # plt.show()
        return aligned_left, aligned_right

    def _tune_velocity(self, px, py):
        px = np.floor(np.array(px) + 0.5).astype(int)
        py = np.floor(np.array(py) + 0.5).astype(int)
        seg_cnt = len(px) - 1
        duration = np.zeros(seg_cnt)
        for i in range(seg_cnt):
            px_T = px[i + 1] - px[i]
            py_T = py[i + 1] - py[i]
            T = np.maximum(px_T, py_T)
            duration[i] = T
        # self.logger.info(f"Duration: {duration}")
        left_traj = self.left_traj.copy()
        right_traj = self.right_traj.copy()

        # self.logger.info(f"px {px}")
        # self.logger.info(f"py {py}")
        def modify_traj(left_traj, right_traj, px, py, duration):
            x_traj = []
            y_traj = []
            x_traj.append(np.atleast_2d(right_traj[0, :]))
            y_traj.append(np.atleast_2d(left_traj[0, :]))
            for i in range(len(duration)):
                ratio_x = duration[i] / (px[i + 1] - px[i])
                ratio_y = duration[i] / (py[i + 1] - py[i])
                # self.logger.info(f"ratio_x: {ratio_x}, ratio_y: {ratio_y}")
                x_start_idx = self.right_idx_chain[px[i]]
                x_end_idx = self.right_idx_chain[px[i + 1]]
                x_len = x_end_idx - x_start_idx
                y_start_idx = self.left_idx_chain[py[i]]
                y_end_idx = self.left_idx_chain[py[i + 1]]
                y_len = y_end_idx - y_start_idx
                if not np.isclose(ratio_x, 1.0):
                    if x_len == 0:
                        insert_row = right_traj[x_start_idx, :]
                        x_interpolated_traj = np.tile(
                            insert_row,
                            (int(self.idx_increment * duration[i]), 1))
                    else:
                        # time_orig = np.linspace(0, 1, x_len)
                        # time_target = np.linspace(0, 1, int(self.idx_increment*duration[i]))
                        # self.logger.info(f"x_len: {x_len}")
                        if x_len == 1:
                            x_len = 2
                            x_end_idx += 1
                        time_orig = np.linspace(0, x_len, x_len)
                        time_target = np.linspace(
                            0, x_len, int(self.idx_increment * duration[i]))
                        # self.logger.info(time_orig)
                        # self.logger.info(time_target)
                        # self.logger.info(f"right_traj: {right_traj}")
                        # self.logger.info(f"right_traj[x_start_idx:x_end_idx,:]: {right_traj[x_start_idx:x_end_idx, :]}")
                        x_interpolated_traj = np.zeros(
                            (int(self.idx_increment * duration[i]), 7))
                        for dim in range(7):
                            interp_func = interp1d(
                                time_orig,
                                right_traj[x_start_idx:x_end_idx, dim],
                                kind="linear")
                            x_interpolated_traj[:,
                                                dim] = interp_func(time_target)
                else:
                    x_interpolated_traj = right_traj[x_start_idx:x_end_idx, :]
                if not np.isclose(ratio_y, 1.0):
                    if y_len == 0:
                        insert_row = left_traj[y_start_idx, :]
                        y_interpolated_traj = np.tile(
                            insert_row,
                            (int(self.idx_increment * duration[i]), 1))
                    else:
                        # time_orig = np.linspace(0, 1, y_len)
                        # time_target = np.linspace(0, 1, int(self.idx_increment*duration[i]))
                        if y_len == 1:
                            y_len = 2
                            y_end_idx += 1
                        time_orig = np.linspace(0, y_len, y_len)
                        time_target = np.linspace(
                            0, y_len, int(self.idx_increment * duration[i]))
                        y_interpolated_traj = np.zeros(
                            (int(self.idx_increment * duration[i]), 7))
                        for dim in range(7):
                            interp_func = interp1d(
                                time_orig,
                                left_traj[y_start_idx:y_end_idx, dim],
                                kind="linear")
                            y_interpolated_traj[:,
                                                dim] = interp_func(time_target)
                else:
                    y_interpolated_traj = left_traj[y_start_idx:y_end_idx, :]
                x_traj.append(x_interpolated_traj)
                y_traj.append(y_interpolated_traj)
                # self.logger.info(x_interpolated_traj.shape[0])
                # self.logger.info(y_interpolated_traj.shape[0])

            x_traj = np.concatenate(x_traj, axis=0)
            y_traj = np.concatenate(y_traj, axis=0)
            x_traj = np.insert(x_traj,
                               -1,
                               right_traj[self.right_idx_chain[-1], :],
                               axis=0)
            y_traj = np.insert(y_traj,
                               -1,
                               left_traj[self.left_idx_chain[-1], :],
                               axis=0)
            return y_traj, x_traj

        modified_left, modified_right = modify_traj(left_traj, right_traj, px,
                                                    py, duration)
        # self.logger.info(f"Modified left trajectory length: {modified_left.shape[0]}")
        # self.logger.info(f"Modified right trajectory length: {modified_right.shape[0]}")

        return modified_left, modified_right

    def Coordinate_RRT(self):
        """
        This method use RRT Connect method based on OMPL to find a path in the coordination space.(Hard to do obstacle expansion)
        During the sampling process, it will check the sample grid is occupied or not, which will save the planning time.
        """
        start_time = time.time()
        ompl_planner = GridRRT(self.grid_map, self.left_traj, self.right_traj,
                               self.left_idx_chain, self.right_idx_chain,
                               self.fcl_checker, self.planner_pin, self.use_ee)
        ompl_planner.set_head_body(self.head_qpos, self.body_qpos)
        ompl_planner.set_end_effector_qpos(self.armlee_qpos, self.armree_qpos)
        path, planning_time, original_path = ompl_planner.plan(
            planning_time=1.0)
        new_grid_map = ompl_planner.get_grid_map()
        flag_map = ompl_planner.get_flag_map()

        px, py, ori_px, ori_py = [], [], [], []
        for i in range(len(original_path)):
            ori_px.append(original_path[i][1])
            ori_py.append(original_path[i][0])
        for i in range(path.getStateCount()):
            px.append(path.getState(i)[1])
            py.append(path.getState(i)[0])

        # Plot the grid map and path in coordination space
        # self.plot_grid_map_RRT(new_grid_map, 0, 0, px, py, ori_px, ori_py)
        # self.plot_flag_map_RRT(flag_map, 0, 0, px, py, ori_px, ori_py)
        # plt.show()

        modified_left, modified_right = self._tune_velocity(px, py)
        aligned_left, aligned_right = self._align_traj(modified_left,
                                                       modified_right)
        self.logger.info(
            f"The coordinated consuming time is {time.time() - start_time:.6f} seconds"
        )

        return aligned_left, aligned_right

    def set_ee_identifier(self, armlee_start_qpos: np.ndarray,
                          armree_start_qpos: np.ndarray,
                          left_identifier_list: list[tuple[int, np.ndarray]],
                          right_identifier_list: list[tuple[int, np.ndarray]]):
        """
        This function is used to set the end effector identifier list for dual arms
        (which is used for two joint-trajectories including ee waiting trajectories) to coordinate

        Args:
            left_identifier_list (list[tuple[int, np.ndarray]]): a list of tuple(action start index, ee qpos)
            right_identifier_list (list[tuple[int, np.ndarray]]): a list of tuple(action start index, ee qpos)

        Notes:
            - The index identifiers are based on the connected joint-trajectory including ee waiting time consumption
        """

        self.armlee_start_qpos, self.armree_start_qpos = armlee_start_qpos, armree_start_qpos
        self.left_identifier_list, self.right_identifier_list = left_identifier_list, right_identifier_list

        self.left_ee_trigger, self.right_ee_trigger = [], []
        self.left_ee_qpos_list, self.right_ee_qpos_list = [], []
        for idx, ee_qpos in left_identifier_list:
            self.left_ee_trigger.append(idx)
            self.left_ee_qpos_list.append(ee_qpos)
        for idx, ee_qpos in right_identifier_list:
            self.right_ee_trigger.append(idx)
            self.right_ee_qpos_list.append(ee_qpos)

    def _build_index_map(self, orig_spans, new_spans):
        """
        orig_spans/new_spans: [(os, oe), ...] / [(ns, ne), ...]，半开区间
        返回: old2new 映射字典
        """
        old2new = {}
        for (os, oe), (ns, ne) in zip(orig_spans, new_spans):
            orig_len = max(1, oe - os)
            new_len = max(1, ne - ns)

            if orig_len == 1:

                old2new[os] = ns
                continue

            denom = orig_len - 1
            for old in range(os, oe):
                alpha = (old - os) / denom
                new_local = int(round(alpha * (new_len - 1)))
                new_idx = ns + new_local
                old2new[old] = new_idx
        return old2new

    def _tune_velocity_with_EE(self, px, py):
        px = np.floor(np.array(px) + 0.5).astype(int)
        py = np.floor(np.array(py) + 0.5).astype(int)
        seg_cnt = len(px) - 1
        duration = np.zeros(seg_cnt)
        for i in range(seg_cnt):
            px_T = px[i + 1] - px[i]
            py_T = py[i + 1] - py[i]
            T = np.maximum(px_T, py_T)
            duration[i] = T
        left_traj = self.left_traj.copy()
        right_traj = self.right_traj.copy()

        def modify_traj(left_traj, right_traj, px, py, duration):
            x_traj, y_traj = [], []
            x_traj.append(np.atleast_2d(right_traj[0, :]))
            y_traj.append(np.atleast_2d(left_traj[0, :]))
            x_orig_duration, y_orig_duration = [], []
            x_new_duration, y_new_duration = [], []
            for i in range(len(duration)):
                ratio_x = duration[i] / (px[i + 1] - px[i])
                ratio_y = duration[i] / (py[i + 1] - py[i])

                x_start_idx = self.right_idx_chain[px[i]]
                x_end_idx = self.right_idx_chain[px[i + 1]]
                x_len = x_end_idx - x_start_idx
                y_start_idx = self.left_idx_chain[py[i]]
                y_end_idx = self.left_idx_chain[py[i + 1]]
                y_len = y_end_idx - y_start_idx
                if not np.isclose(ratio_x, 1.0):
                    if x_len == 0:
                        insert_row = right_traj[x_start_idx, :]
                        x_interpolated_traj = np.tile(
                            insert_row,
                            (int(self.idx_increment * duration[i]), 1))
                    else:
                        if x_len == 1:
                            x_len = 2
                            x_end_idx += 1
                        time_orig = np.linspace(0, x_len, x_len)
                        time_target = np.linspace(
                            0, x_len, int(self.idx_increment * duration[i]))
                        x_interpolated_traj = np.zeros(
                            (int(self.idx_increment * duration[i]), 7))
                        for dim in range(7):
                            interp_func = interp1d(
                                time_orig,
                                right_traj[x_start_idx:x_end_idx, dim],
                                kind="linear")
                            x_interpolated_traj[:,
                                                dim] = interp_func(time_target)
                else:
                    x_interpolated_traj = right_traj[x_start_idx:x_end_idx, :]

                if not np.isclose(ratio_y, 1.0):
                    if y_len == 0:
                        insert_row = left_traj[y_start_idx, :]
                        y_interpolated_traj = np.tile(
                            insert_row,
                            (int(self.idx_increment * duration[i]), 1))
                    else:
                        if y_len == 1:
                            y_len = 2
                            y_end_idx += 1
                        time_orig = np.linspace(0, y_len, y_len)
                        time_target = np.linspace(
                            0, y_len, int(self.idx_increment * duration[i]))
                        y_interpolated_traj = np.zeros(
                            (int(self.idx_increment * duration[i]), 7))
                        for dim in range(7):
                            interp_func = interp1d(
                                time_orig,
                                left_traj[y_start_idx:y_end_idx, dim],
                                kind="linear")
                            y_interpolated_traj[:,
                                                dim] = interp_func(time_target)
                else:
                    y_interpolated_traj = left_traj[y_start_idx:y_end_idx, :]

                x_orig_duration.append((x_start_idx, x_end_idx))
                x_new_duration.append(
                    (len(x_traj), len(x_traj) + len(x_interpolated_traj)))
                y_orig_duration.append((y_start_idx, y_end_idx))
                y_new_duration.append(
                    (len(y_traj), len(y_traj) + len(y_interpolated_traj)))
                x_traj.append(x_interpolated_traj)
                y_traj.append(y_interpolated_traj)

            left_old2new = self._build_index_map(y_orig_duration,
                                                 y_new_duration)
            right_old2new = self._build_index_map(x_orig_duration,
                                                  x_new_duration)
            left_ee_trigger_new = [
                left_old2new[idx]
                for idx in self.left_ee_trigger
                if idx in left_old2new
            ]
            right_ee_trigger_new = [
                right_old2new[idx]
                for idx in self.right_ee_trigger
                if idx in right_old2new
            ]
            self.logger.info(f"left_ee_trigger_new: {left_ee_trigger_new}")
            self.logger.info(f"right_ee_trigger_new: {right_ee_trigger_new}")

            x_traj = np.concatenate(x_traj, axis=0)
            y_traj = np.concatenate(y_traj, axis=0)
            x_traj = np.insert(x_traj,
                               -1,
                               right_traj[self.right_idx_chain[-1], :],
                               axis=0)
            y_traj = np.insert(y_traj,
                               -1,
                               left_traj[self.left_idx_chain[-1], :],
                               axis=0)
            return y_traj, x_traj, left_ee_trigger_new, right_ee_trigger_new

        modified_left, modified_right, modified_left_ee_trigger, modified_right_ee_trigger = modify_traj(
            left_traj, right_traj, px, py, duration)
        self.logger.info(
            f"Modified left trajectory length: {modified_left.shape[0]}")
        self.logger.info(
            f"Modified right trajectory length: {modified_right.shape[0]}")

        self.modified_left_ee_trigger, self.modified_right_ee_trigger = [], []
        for i in range(len(modified_left_ee_trigger)):
            self.modified_left_ee_trigger.append(
                (modified_left_ee_trigger[i], self.left_ee_qpos_list[i]))
        for i in range(len(modified_right_ee_trigger)):
            self.modified_right_ee_trigger.append(
                (modified_right_ee_trigger[i], self.right_ee_qpos_list[i]))

        return modified_left, modified_right

    def Coordinate_RRT_with_EE(self):
        ompl_planner = GridRRT(self.grid_map, self.left_traj, self.right_traj,
                               self.left_idx_chain, self.right_idx_chain,
                               self.fcl_checker, self.planner_pin, self.use_ee)
        ompl_planner.set_head_body(self.head_qpos, self.body_qpos)
        ompl_planner.set_ee_identifier(self.armlee_start_qpos,
                                       self.armree_start_qpos,
                                       self.left_identifier_list,
                                       self.right_identifier_list)
        path, planning_time, original_path = ompl_planner.plan_with_ee(
            planning_time=2.0)
        # new_grid_map = ompl_planner.get_grid_map()
        # flag_map = ompl_planner.get_flag_map()

        px, py, ori_px, ori_py = [], [], [], []
        for i in range(len(original_path)):
            ori_px.append(original_path[i][1])
            ori_py.append(original_path[i][0])
        for i in range(path.getStateCount()):
            px.append(path.getState(i)[1])
            py.append(path.getState(i)[0])

        modified_left, modified_right = self._tune_velocity_with_EE(px, py)
        aligned_left, aligned_right = self._align_traj(modified_left,
                                                       modified_right)

        return aligned_left, aligned_right, self.modified_left_ee_trigger, self.modified_right_ee_trigger

    def plot_grid_map_Astar(self, grid_map, start, end, px, py):
        fig, ax = plt.subplots(figsize=(8, 12))
        ax.imshow(grid_map, cmap="gray", origin="upper", vmin=0, vmax=1)
        ax.plot(px, py, "-or", linewidth=2, markersize=4, label="A* Path")
        # ax.scatter(start.x, start.y, c="green", s=80, marker="s", label="Start")
        # ax.scatter(end.x,   end.y,   c="blue",  s=80, marker="s", label="End")

        ax.set_xticks(np.arange(-.5, grid_map.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-.5, grid_map.shape[0], 1), minor=True)
        ax.grid(which="minor", color="black", linestyle="-", linewidth=0.4)
        ax.set_xticks([])
        ax.set_yticks([])  # 去掉坐标刻度

        ax.set_title("A* Path on Coordination Space -- Grid Map")
        ax.set_xlabel("Right Arm Local time")
        ax.set_ylabel("Left Arm Local time")
        ax.legend(loc="upper left", bbox_to_anchor=(1, 1))
        plt.tight_layout()
        # plt.show()

    def plot_grid_map_RRT(self, grid_map, start, end, px, py, original_px,
                          original_py):
        fig, ax = plt.subplots(figsize=(8, 12))
        ax.imshow(grid_map, cmap="gray", origin="upper", vmin=0, vmax=1)
        ax.plot(px,
                py,
                "-or",
                linewidth=2,
                markersize=4,
                label="reduced RRT Path")
        ax.plot(original_px,
                original_py,
                "-ob",
                linewidth=1,
                markersize=4,
                label="original RRT path")
        # ax.scatter(start.x, start.y, c="green", s=80, marker="s", label="Start")
        # ax.scatter(end.x,   end.y,   c="blue",  s=80, marker="s", label="End")

        ax.set_xticks(np.arange(-.5, grid_map.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-.5, grid_map.shape[0], 1), minor=True)
        ax.grid(which="minor", color="black", linestyle="-", linewidth=0.4)
        ax.set_xticks([])
        ax.set_yticks([])  # 去掉坐标刻度

        ax.set_title("RRT Path on Coordination Space -- Grid Map")
        ax.set_xlabel("Right Arm Local time")
        ax.set_ylabel("Left Arm Local time")
        ax.legend(loc="upper left", bbox_to_anchor=(1, 1))
        plt.tight_layout()
        # plt.show()

    def plot_flag_map_RRT(self, grid_map, start, end, px, py, original_px,
                          original_py):
        fig, ax = plt.subplots(figsize=(8, 12))
        ax.imshow(grid_map, cmap="gray_r", origin="upper", vmin=0, vmax=1)
        ax.plot(px,
                py,
                "-or",
                linewidth=2,
                markersize=4,
                label="reeduced RRT Path")
        ax.plot(original_px,
                original_py,
                "-ob",
                linewidth=1,
                markersize=4,
                label="original RRT path")
        # ax.scatter(start.x, start.y, c="green", s=80, marker="s", label="Start")
        # ax.scatter(end.x,   end.y,   c="blue",  s=80, marker="s", label="End")

        ax.set_xticks(np.arange(-.5, grid_map.shape[1], 1), minor=True)
        ax.set_yticks(np.arange(-.5, grid_map.shape[0], 1), minor=True)
        ax.grid(which="minor", color="black", linestyle="-", linewidth=0.4)
        ax.set_xticks([])
        ax.set_yticks([])  # 去掉坐标刻度

        ax.set_title("RRT Path on Coordination Space -- Flag Map")
        ax.set_xlabel("Right Arm Local time")
        ax.set_ylabel("Left Arm Local time")
        ax.legend(loc="upper left", bbox_to_anchor=(1, 1))
        plt.tight_layout()
        # plt.show()
