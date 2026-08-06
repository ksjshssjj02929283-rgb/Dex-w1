'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
import time
from dataclasses import dataclass
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.heuristic import *
from scipy.interpolate import interp1d
import ompl.base as ob
import ompl.geometric as og

from ...common import get_logger, StatusCode
from ..collision.collision_checker_wrapper import CollisionCheckerWrapper
from ..kinematics.pinocchio_kinematics import PinocchioKinematics

# Debug
from IPython import embed


@dataclass
class CoordinationPlannerConfig:
    sample_step: float = 5  # maximum sample step
    checking_resolution: float = 0.5  # absolute resolution
    max_retries: int = 10


@dataclass
class CoordinationSpaceConfig:
    space_resolution: float = 0.1
    trajectory_resolution: float = 0.01


class ForwardMotionValidator(ob.MotionValidator):
    """
    This is a inherited class from ompl.base.MotionValidator, which overrides the checkMothon() function.
    It will check the motion is towards the forward direction only and check the motion validity 
    by sampling according to a fixed distance resolution or according to the relative percentage resolution.
    """

    def __init__(self, si, checking_resolution: float, state_validity_fn):
        super().__init__(si)
        # self.si = si
        self.state_space = si.getStateSpace()
        self.state_validity_fn = state_validity_fn
        self.res = checking_resolution

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

        # dist = self.si.distance(s1, s2)
        dist = self.state_space.distance(s1, s2)
        steps = max(1, int(dist / self.res))
        for i in range(1, steps + 1):
            interp = self.state_space.allocState()
            self.state_space.interpolate(s1, s2, i / steps, interp)
            if not self.state_validity_fn(interp):
                return False

        return True


class CoordinationSpaceRRT:

    def __init__(self,
                 grid_map: np.ndarray,
                 left_arm_traj: np.ndarray,
                 right_arm_traj: np.ndarray,
                 left_idx_list: list[int],
                 right_idx_list: list[int],
                 collision_checker: CollisionCheckerWrapper,
                 coordination_planner_config: CoordinationPlannerConfig,
                 left_ee_trigger_list: list[tuple[int,
                                                  list | np.ndarray]] = None,
                 right_ee_trigger_list: list[tuple[int,
                                                   list | np.ndarray]] = None):
        """
        RRT-Connect method to find a collision-free path in coordination space.

        Args:
            - left_ee_trigger_list/right_ee_trigger_list (list of tuple[int, list[float]]): [(trig_index, [pulse1, pulse2, ...]), (trig_index, [pulse1, pulse2, ...]), ...]
                The first tuple should be the start_ee_pulse, like (0, [pulse1, pulse2, ...])

        Notes:
            - grid_map is a matrix with left_arm_traj as verticle axis(rows) and right_arm_traj as horizontal axis(columns)
            - The start state is left-upper corner of the matrix(or you can regard it as a 2D grid map)
            - The goal state is the right-bottom corner of the matrix
            - Each grid point is a element of the matrix (row, col) or (left_traj_point, right_traj_point)
            - The order of freedom of RRT state is (row, column) or (verticle, horizon) or (left_arm, right_arm)

            - The length of left_idx_list should be same as the number of rows of grid map, same for right arm
        """
        self.logger = get_logger()

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
        self.flag_map = np.zeros(
            (self.grid_map.shape[0], self.grid_map.shape[1]))
        self.left_arm_traj, self.right_arm_traj = left_arm_traj, right_arm_traj
        self.left_idx_list, self.right_idx_list = left_idx_list, right_idx_list

        self.collision_checker = collision_checker
        self.coordination_planner_config = coordination_planner_config

        self.left_ee_trigger_index, self.right_ee_trigger_index = [], []
        self.left_ee_trigger_pulse, self.right_ee_trigger_pulse = [], []
        if left_ee_trigger_list is not None and left_ee_trigger_list != []:
            if left_ee_trigger_list[0][0] != 0:
                raise ValueError(
                    f"The first trigger of left_ee_trigger_list should be the start ee pulse with index 0"
                )
            for idx, pulse in left_ee_trigger_list:
                self.left_ee_trigger_index.append(idx)
                self.left_ee_trigger_pulse.append(pulse)
        if right_ee_trigger_list is not None and right_ee_trigger_list != []:
            if right_ee_trigger_list[0][0] != 0:
                raise ValueError(
                    f"The first trigger of right_ee_trigger_list should be the start ee pulse with index 0"
                )
            for idx, pulse in right_ee_trigger_list:
                self.right_ee_trigger_index.append(idx)
                self.right_ee_trigger_pulse.append(pulse)

    def _sample_state_to_grid_idx(self, state):
        """
        Transfer a RRT sample state to the idx of grid point
        """
        state = np.array([state[i] for i in range(2)], dtype=np.float64)
        state = np.floor(state + 0.5).astype(int)
        return state[0], state[1]

    def _query_ee_pulse(self, index: int, is_left: bool):
        """
        Query the end effector pulse of provided trajectory index
        """
        if is_left:
            if not self.left_ee_trigger_index:
                raise ValueError(
                    f"unknown left ee trigger list but query ee pulse.")
            for i in range(len(self.left_ee_trigger_index)):
                if index <= self.left_ee_trigger_index[i]:
                    if i == 0:
                        return self.left_ee_trigger_pulse[0]
                    else:
                        return self.left_ee_trigger_pulse[i - 1]
            return self.left_ee_trigger_pulse[-1]
        else:
            if not self.right_ee_trigger_index:
                raise ValueError(
                    f"unknown right ee trigger list but query ee pulse")
            for i in range(len(self.right_ee_trigger_index)):
                if index <= self.right_ee_trigger_index[i]:
                    if i == 0:
                        return self.right_ee_trigger_pulse[0]
                    else:
                        return self.right_ee_trigger_pulse[i - 1]
            return self.right_ee_trigger_pulse[-1]

    def _is_state_valid(self, state):
        """
        Check whether the state is valid by checking collision between arms
        """
        row, col = self._sample_state_to_grid_idx(state)
        if self.left_ee_trigger_index:
            left_ee_pulse = self._query_ee_pulse(self.left_idx_list[row], True)
        else:
            left_ee_pulse = None
        if self.right_ee_trigger_index:
            right_ee_pulse = self._query_ee_pulse(self.right_idx_list[col],
                                                  False)
        else:
            right_ee_pulse = None

        if self.flag_map[row, col] == 0:
            self.flag_map[row, col] = 1
            left_arm_qpos = self.left_arm_traj[self.left_idx_list[row], :]
            right_arm_qpos = self.right_arm_traj[self.right_idx_list[col], :]
            res = self.collision_checker.check_collision_between_arms(
                left_arm_qpos=left_arm_qpos,
                right_arm_qpos=right_arm_qpos,
                left_ee_pulse=left_ee_pulse,
                right_ee_pulse=right_ee_pulse)
            if res != StatusCode.SUCCESS:
                self.grid_map[row, col] = 0
                return False
            else:
                return True
        else:
            if self.grid_map[row, col] == 0:
                return False
            else:
                return True

    def plan(self, planning_time: float = 1.0):
        si = ob.SpaceInformation(self.space)
        si.setStateValidityChecker(self._is_state_valid)
        motion_validator = ForwardMotionValidator(
            si, self.coordination_planner_config.checking_resolution,
            self._is_state_valid)
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
        planner.setRange(self.coordination_planner_config.sample_step)
        planner.setProblemDefinition(pdef)
        planner.setup()

        for attempt in range(self.coordination_planner_config.max_retries + 1):
            solved = planner.solve(planning_time)
            # reuse cache RRT tree to continue search, if we need construct a new RRT tree, it should not have local attempt count, use global retry count and use plan() for replan.
            if solved.getStatus() == ob.PlannerStatus.TIMEOUT:
                self.logger.warning(
                    f"Warning: Planning took longer than the specified time limit of {planning_time} seconds."
                )
                self.logger.info(
                    f"Continue planning for {attempt + 1} times...")
                continue

            if solved.getStatus() == ob.PlannerStatus.EXACT_SOLUTION:
                path = pdef.getSolutionPath()
                original_path = []
                for i in range(path.getStateCount()):
                    state = path.getState(i)
                    original_path.append((state[0], state[1]))
                path_simplifier = og.PathSimplifier(si)
                path_simplifier.reduceVertices(path)
                return path, original_path

            self.logger.error(f"Planning failed: {solved.asString()}")
            break
        self.logger.error(
            f"Planning failed since reached maximum retry times or broke for some other reason."
        )
        return None, None


class DualArmCoordination:

    def __init__(self,
                 left_arm_traj: np.ndarray,
                 right_arm_traj: np.ndarray,
                 coordination_space_config: CoordinationSpaceConfig,
                 coordination_planner_config: CoordinationPlannerConfig,
                 collision_checker: CollisionCheckerWrapper,
                 left_ee_start_pulse: np.ndarray = None,
                 right_ee_start_pulse: np.ndarray = None,
                 left_ee_trigger_list: list[tuple[int, list]] = None,
                 right_ee_trigger_list: list[tuple[int, list]] = None):
        self.logger = get_logger()
        self.left_arm_traj = left_arm_traj
        self.right_arm_traj = right_arm_traj
        self.coordination_space_config = coordination_space_config
        self.coordination_planner_config = coordination_planner_config
        self.collision_checker = collision_checker

        if coordination_space_config.space_resolution < coordination_space_config.trajectory_resolution:
            raise ValueError(
                f"In coordination space, the space resolution {coordination_space_config.space_resolution} should be larger than trajectory resolution {coordination_space_config.trajectory_resolution}."
            )

        self.idx_increment = round(
            coordination_space_config.space_resolution /
            coordination_space_config.trajectory_resolution)
        self.left_arm_idx_list = np.r_[
            np.arange(0, self.left_arm_traj.shape[0] - 1, self.idx_increment),
            self.left_arm_traj.shape[0] - 1]
        self.right_arm_idx_list = np.r_[
            np.arange(0, self.right_arm_traj.shape[0] - 1, self.idx_increment),
            self.right_arm_traj.shape[0] - 1]
        self.map_verticle_len = len(self.left_arm_idx_list)
        self.map_horizontal_len = len(self.right_arm_idx_list)

        self.grid_map = np.ones(
            (self.map_verticle_len, self.map_horizontal_len), dtype=np.int8)

        self.left_ee_trigger_list, self.right_ee_trigger_list = [], []
        if left_ee_start_pulse is not None:
            left_ee_start_trigger = (0, left_ee_start_pulse.tolist())
            if left_ee_trigger_list is not None:
                self.left_ee_trigger_list.append(left_ee_start_trigger)
                self.left_ee_trigger_list.extend(left_ee_trigger_list)
            else:
                self.left_ee_trigger_list.append(left_ee_start_trigger)
        if right_ee_start_pulse is not None:
            right_ee_start_trigger = (0, right_ee_start_pulse.tolist())
            if right_ee_trigger_list is not None:
                self.right_ee_trigger_list.append(right_ee_start_trigger)
                self.right_ee_trigger_list.extend(right_ee_trigger_list)
            else:
                self.right_ee_trigger_list.append(right_ee_start_trigger)

        self.left_ee_trigger_index, self.right_ee_trigger_index = [], []
        self.left_ee_trigger_pulse, self.right_ee_trigger_pulse = [], []
        if self.left_ee_trigger_list:
            for idx, pulse in self.left_ee_trigger_list:
                self.left_ee_trigger_index.append(idx)
                self.left_ee_trigger_pulse.append(pulse)
        if self.right_ee_trigger_list:
            for idx, pulse in self.right_ee_trigger_list:
                self.right_ee_trigger_index.append(idx)
                self.right_ee_trigger_pulse.append(pulse)

    def coordinate(self, planning_time: float = 1.0):
        coordination_planner = CoordinationSpaceRRT(
            self.grid_map, self.left_arm_traj, self.right_arm_traj,
            self.left_arm_idx_list, self.right_arm_idx_list,
            self.collision_checker, self.coordination_planner_config,
            self.left_ee_trigger_list, self.right_ee_trigger_list)

        simp_coord_path, origin_coord_path = coordination_planner.plan(
            planning_time)
        left_simp_coord_nodes, right_simp_coord_nodes, left_origin_coord_nodes, right_origin_coord_nodes = [], [], [], []
        for i in range(len(origin_coord_path)):
            left_origin_coord_nodes.append(origin_coord_path[i][0])
            right_origin_coord_nodes.append(origin_coord_path[i][1])
        for i in range(simp_coord_path.getStateCount()):
            left_simp_coord_nodes.append(simp_coord_path.getState(i)[0])
            right_simp_coord_nodes.append(simp_coord_path.getState(i)[1])

        modified_left_traj, modified_right_traj, modified_left_ee_trigger_list, modified_right_ee_trigger_list = \
            self._modify_trajectory(left_simp_coord_nodes, right_simp_coord_nodes)

        return modified_left_traj, modified_right_traj, modified_left_ee_trigger_list, modified_right_ee_trigger_list

    def _modify_trajectory(self, left_coord_nodes: list[int],
                           right_coord_nodes: list[int]):
        left_coord_nodes = np.floor(np.asarray(left_coord_nodes) +
                                    0.5).astype(int)
        right_coord_nodes = np.floor(np.asarray(right_coord_nodes) +
                                     0.5).astype(int)

        # TODO: whether to add some dimenson checking?

        # segment_cnt = left_coord_nodes.shape[0] - 1
        # segment_duration = np.zeros(segment_cnt)
        # for i in range(segment_cnt):
        #     left_duration = left_coord_nodes[i+1] - left_coord_nodes[i]
        #     right_duration = right_coord_nodes[i+1] - right_coord_nodes[i]
        #     coord_duration = np.maximum(left_duration, right_duration)
        #     segment_duration[i] = coord_duration

        left_arm_traj = self.left_arm_traj.copy()
        right_arm_traj = self.right_arm_traj.copy()

        # left_coord_traj = [np.atleast_2d(left_arm_traj[0, :])]
        # right_coord_traj = [np.atleast_2d(right_arm_traj[0, :])]

        # left_origin_idx_section, right_origin_idx_section = [], []
        # left_new_idx_section, right_new_idx_section = [], []

        def resample_segment(trajectory: np.ndarray, start_idx: int,
                             end_idx: int, target_intervals: int):
            if target_intervals <= 0:
                self.logger.debug(
                    "Unpositve coordination intervals occured, find the reason."
                )
                return np.empty((0, trajectory.shape[1]),
                                dtype=trajectory.dtype)
            if start_idx == end_idx:
                return np.repeat(trajectory[start_idx:start_idx + 1],
                                 target_intervals,
                                 axis=0)
            source_traj = trajectory[start_idx:end_idx + 1]
            source_time = np.linspace(0.0, 1.0, source_traj.shape[0])
            target_time = np.linspace(0.0, 1.0, target_intervals + 1)[1:]
            interp_func = interp1d(source_time,
                                   source_traj,
                                   kind="linear",
                                   axis=0)

            result_traj = np.asarray(interp_func(target_time))
            return result_traj

        def add_index_mapping(index_map: dict[int, int], original_start: int,
                              original_end: int, new_start: int, new_end: int):
            """
            Both spans are inclusive
            """
            if original_start == original_end:
                index_map.setdefault(original_start, new_start)
                return

            original_intervals = original_end - original_start
            new_intervals = new_end - new_start

            for original_idx in range(original_start, original_end + 1):
                alpha = (original_idx - original_start) / original_intervals
                new_idx = new_start + int(np.ceil(alpha * new_intervals))
                index_map.setdefault(original_idx, new_idx)

        def remap_ee_triggers(trigger_list: list[tuple[int, list]] | None,
                              index_map: dict[int, int]):
            if trigger_list is None:
                return None

            remapped = []
            for original_idx, pulse in trigger_list:
                if original_idx not in index_map:
                    raise ValueError(
                        f"EE trigger index {original_idx} is not covered by the coordination path"
                    )
                remapped.append((index_map[original_idx], pulse))

            return remapped

        left_segments = [left_arm_traj[0:1]]
        right_segments = [right_arm_traj[0:1]]

        left_old_to_new = {0: 0}
        right_old_to_new = {0: 0}

        new_idx = 0

        for i in range(left_coord_nodes.size - 1):
            left_grid_start = left_coord_nodes[i]
            left_grid_end = left_coord_nodes[i + 1]
            right_grid_start = right_coord_nodes[i]
            right_grid_end = right_coord_nodes[i + 1]

            if (left_grid_start == left_grid_end and
                    right_grid_start == right_grid_end):
                continue
            left_start_idx = int(self.left_arm_idx_list[left_grid_start])
            left_end_idx = int(self.left_arm_idx_list[left_grid_end])
            right_start_idx = int(self.right_arm_idx_list[right_grid_start])
            right_end_idx = int(self.right_arm_idx_list[right_grid_end])

            left_original_intervals = left_end_idx - left_start_idx
            right_original_intervals = right_end_idx - right_start_idx

            target_intervals = max(left_original_intervals,
                                   right_original_intervals)

            if target_intervals <= 0:
                raise ValueError(
                    f"Coordination segment {i} has no forward progress")

            left_segment = resample_segment(left_arm_traj, left_start_idx,
                                            left_end_idx, target_intervals)
            right_segment = resample_segment(right_arm_traj, right_start_idx,
                                             right_end_idx, target_intervals)
            new_start_idx = new_idx
            new_end_idx = new_idx + target_intervals

            add_index_mapping(left_old_to_new, left_start_idx, left_end_idx,
                              new_start_idx, new_end_idx)
            add_index_mapping(right_old_to_new, right_start_idx, right_end_idx,
                              new_start_idx, new_end_idx)

            left_segments.append(left_segment)
            right_segments.append(right_segment)

            new_idx = new_end_idx

        modified_left_traj = np.concatenate(left_segments, axis=0)
        modified_right_traj = np.concatenate(right_segments, axis=0)

        if modified_left_traj.shape[0] != modified_right_traj.shape[0]:
            raise RuntimeError(
                f"Modified left and right trajectories have different lengths: {modified_left_traj.shape[0]}, {modified_right_traj.shape[0]}"
            )
        if not np.allclose(modified_left_traj[-1], left_arm_traj[-1]):
            raise RuntimeError(
                "Modified left trajectory does not reach its goal.")

        if not np.allclose(modified_right_traj[-1], right_arm_traj[-1]):
            raise RuntimeError(
                "Modified right trajectory does not reach its goal.")

        modified_left_ee_trigger_list = remap_ee_triggers(
            self.left_ee_trigger_list, left_old_to_new)
        modified_right_ee_trigger_list = remap_ee_triggers(
            self.right_ee_trigger_list, right_old_to_new)

        return modified_left_traj, modified_right_traj, modified_left_ee_trigger_list, modified_right_ee_trigger_list

        # for i in range(segment_cnt):
        #     left_duration_ratio = segment_duration[i] / (left_coord_nodes[i+1] - left_coord_nodes[i])
        #     right_duration_ratio = segment_duration[i] / (right_coord_nodes[i+1] - right_coord_nodes[i])

        #     left_start_idx = self.left_arm_idx_list[left_coord_nodes[i]]
        #     left_end_idx = self.left_arm_idx_list[left_coord_nodes[i+1]]
        #     left_idx_length = left_end_idx - left_start_idx
        #     right_start_idx = self.right_arm_idx_list[right_coord_nodes[i]]
        #     right_end_idx = self.right_arm_idx_list[right_coord_nodes[i+1]]
        #     right_idx_length = right_end_idx - right_start_idx

        #     if not np.isclose(left_duration_ratio, 1.0):
        #         if left_idx_length == 0:
        #             insert_jps = left_arm_traj[left_start_idx, :]
        #             left_interpolated_traj = np.tile(
        #                 insert_jps,
        #                 (int(self.idx_increment * segment_duration[i]))
        #             )
        #         elif
