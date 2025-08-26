import ompl.base as ob
import ompl.geometric as og
import numpy as np
from dexe_robot_client.w1.path_planner.fcl_checker import FCLChecker
from dexe_robot_client.w1.trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from dexe_robot_client.w1.path_planner.traj_coordination import Coordination
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
import pinocchio
import time
import os
import csv

CHECK_RES = 0.005
SAMPLE_RANGE = 0.1

"""
Single Arm Plan Strategy
init -> set_head_body -> plan
keep which arm is used but change the start and goal, or change the other arm qpos:
set_start_goal -> plan  set_other_arm_qpos -> plan
switch arm:
switch_arm -> set_start_goal -> plan
"""

class OMPLPlanSingle:
    def __init__(self, space_dimension, bounds, start_state, goal_state, fcl_checker: FCLChecker, planner_pin: Planner_Pin, which_arm, other_arm_qpos, use_ee: bool = False):
        if bounds.shape[1] != space_dimension or bounds.shape[0] != 2:
            raise ValueError(f"Bounds must be a 2x{space_dimension} array, got {bounds.shape}")
        if start_state.shape[0] != space_dimension or goal_state.shape[0] != space_dimension:
            raise ValueError(f"Start and goal state must have {space_dimension} elements, got {start_state.shape} and {goal_state.shape}")
        
        self.space = ob.RealVectorStateSpace(space_dimension)
        self.bounds = ob.RealVectorBounds(space_dimension)
        for i in range(space_dimension):
            self.bounds.setLow(i, bounds[0,i])
            self.bounds.setHigh(i, bounds[1,i])
        self.space.setBounds(self.bounds)

        self.which_arm = which_arm
        self.is_left = None
        if which_arm == "left":
            self.is_left = True
        elif which_arm == "right":
            self.is_left = False
        else:
            raise ValueError(f"which_arm must be 'left' or 'right', got {which_arm}")
        
        self.start_state = start_state
        self.goal_state = goal_state
        self.fcl_checker = fcl_checker
        self.other_arm_qpos = other_arm_qpos
        self.result_path = None
        self.result_traj = None

        self.head_qpos = None
        self.body_qpos = None

        # self.pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        # self.pin_data = self.pin_model.createData()
        self.planner_pin = planner_pin

        self.use_ee = use_ee
        
        self.traj_max_vel = 0.3


    # def _get_Kinematics(self, body_qpos, arml_qpos, armr_qpos, head_qpos):
    #     # NOTE: 
    #     # q's order depends on urdf, currently is [body(4), left arm(7), right arm(7), head(2)]
    #     # pinocchio will output 21 transformation matrices include universe as the first one
    #     if body_qpos is None or head_qpos is None:
    #         raise ValueError("body_qpos and head_qpos must be set before calling _get_Kinematics")
    #     q = np.concatenate((body_qpos, arml_qpos, head_qpos, armr_qpos))
    #     pinocchio.forwardKinematics(self.pin_model, self.pin_data, q)
    #     pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
    #     transform_chain = [oMi.homogeneous for oMi in self.pin_data.oMi]
    #     left_transform_chain = transform_chain[5:12]  # left arm links
    #     right_transform_chain = transform_chain[14:21]  # right arm links
    #     if self.use_ee:
    #         armlee_ID = self.pin_model.getFrameId("LEFT_EE")  
    #         armree_ID = self.pin_model.getFrameId("RIGHT_EE")  
    #         lee_trans = self.pin_data.oMf[armlee_ID].homogeneous
    #         ree_trans = self.pin_data.oMf[armree_ID].homogeneous
    #         left_transform_chain.append(lee_trans)
    #         right_transform_chain.append(ree_trans)
        
    #     body_transform_chain = transform_chain[1:5]
    #     head_transform_chain = transform_chain[12:14]

    #     return body_transform_chain, left_transform_chain, right_transform_chain, head_transform_chain



    def set_head_body(self, head_qpos, body_qpos):
        if head_qpos.shape[0] != 2:
            raise ValueError(f"head_qpos must be a 2-element array, got {head_qpos.shape}")
        if body_qpos.shape[0] != 4:
            raise ValueError(f"body_qpos must be a 4-element array, got {body_qpos.shape}")
        self.head_qpos = head_qpos
        self.body_qpos = body_qpos
        if self.is_left:
            # body, arml, armr, head = self._get_Kinematics(self.body_qpos, self.start_state, self.other_arm_qpos, self.head_qpos)
            real_qpos = np.concatenate((self.body_qpos, self.head_qpos, self.start_state, self.other_arm_qpos))
            torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(real_qpos)
        else:
            # body, arml, armr, head = self._get_Kinematics(self.body_qpos, self.other_arm_qpos, self.start_state, self.head_qpos)
            real_qpos = np.concatenate((self.body_qpos, self.head_qpos, self.other_arm_qpos, self.start_state))
            torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(real_qpos)
        self.fcl_checker.set_head_body_transform(head_trans, torso_trans)
        

    def set_traj_max_vel(self, max_vel):
        if max_vel <= 0:
            raise ValueError(f"max_vel must be positive, got {max_vel}")
        self.traj_max_vel = max_vel
    
    def set_start_goal(self, start, goal):
        if start.shape[0] != self.space.getDimension():
            raise ValueError(f"Start state must have {self.space.getDimension()} elements, got {start.shape}")
        if goal.shape[0] != self.space.getDimension():
            raise ValueError(f"Goal state must have {self.space.getDimension()} elements, got {goal.shape}")

        self.start_state = start
        self.goal_state = goal

    def set_other_arm_qpos(self, other_arm_qpos):
        if other_arm_qpos.shape[0] != self.space.getDimension():
            raise ValueError(f"Other arm qpos must have {self.space.getDimension()} elements, got {other_arm_qpos.shape}")
        self.other_arm_qpos = other_arm_qpos

    def switch_arm(self, which_arm, bounds, other_arm_qpos):
        if which_arm == "left":
            self.is_left = True
        elif which_arm == "right":
            self.is_left = False
        else:
            raise ValueError(f"which_arm must be 'left' or 'right', got {which_arm}")
        
        if bounds.shape[1] != self.space.getDimension() or bounds.shape[0] != 2:
            raise ValueError(f"Bounds must be a 2x{self.space.getDimension()} array, got {bounds.shape}")
        
        for i in range(self.space.getDimension()):
            self.bounds.setLow(i, bounds[0,i])
            self.bounds.setHigh(i, bounds[1,i])
        self.space.setBounds(self.bounds)
        
        self.other_arm_qpos = other_arm_qpos

    def _is_state_valid(self, state):
        if self.is_left:
            lqpos = np.array([state[i] for i in range(self.space.getDimension())], dtype=np.float32)
            rqpos = self.other_arm_qpos
        else:
            rqpos = np.array([state[i] for i in range(self.space.getDimension())], dtype=np.float32)
            lqpos = self.other_arm_qpos
        real_qpos = np.concatenate((self.body_qpos, self.head_qpos, lqpos, rqpos))
        # body, arml, armr, head = self._get_Kinematics(self.body_qpos, arml_qpos=lqpos, armr_qpos=rqpos, head_qpos=self.head_qpos)
        torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(real_qpos)
        transform = arml_trans + armr_trans
        self.fcl_checker.update_arm_transform(transform)

        return not self.fcl_checker.check_collision()
    
    def save_result_path(self, file_name):
        current_path = os.path.dirname(os.path.abspath(__file__))
        saved_folder_path = os.path.join(current_path, "saved_path")
        if not os.path.exists(saved_folder_path):
            os.makedirs(saved_folder_path)
        saved_file_path = os.path.join(saved_folder_path, file_name)
        with open(saved_file_path, "w", newline='') as f:
            writer = csv.writer(f)
            if self.is_left:
                header = [f"left_q{i}" for i in range(self.space.getDimension())]
            else:
                header = [f"right_q{i}" for i in range(self.space.getDimension())]
            writer.writerow(header)
            for i in range(self.result_path.shape[0]):
                row = self.result_path[i]
                writer.writerow(row)
    def save_result_traj(self, file_name):
        current_path = os.path.dirname(os.path.abspath(__file__))
        saved_folder_path = os.path.join(current_path, "saved_path")
        if not os.path.exists(saved_folder_path):
            os.makedirs(saved_folder_path)
        saved_file_path = os.path.join(saved_folder_path, file_name)
        with open(saved_file_path, "w", newline='') as f:
            writer = csv.writer(f)
            if self.is_left:
                header = [f"left_q{i}" for i in range(self.space.getDimension())]
            else:
                header = [f"right_q{i}" for i in range(self.space.getDimension())]
            writer.writerow(header)
            for i in range(self.result_traj.shape[0]):
                row = self.result_traj[i]
                writer.writerow(row)

    def _np_path(self, path):
        qpos_list = []
        for i in range(path.getStateCount()):
            state = path.getState(i)
            qpos = np.array([state[j] for j in range(self.space.getDimension())], dtype=np.float64)
            qpos_list.append(qpos)
        return np.array(qpos_list, dtype=np.float64)

    def gen_trajectory(self, path, max_v):
        # path_waypoints = []
        # for i in range(path.shape[0]):
        #     point = {"position": path[i, :].tolist()}
        #     path_waypoints.append(point)
        dofs = self.space.getDimension()
        vel_limits = [max_v]*dofs
        acc_limits = [5.0]*dofs
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, pos_array, vel_array, acc_array, duration = traj_planner.plan(path)
        return pos_array, vel_array, acc_array, duration


    def plan(self, planning_time=2.0):
        si = ob.SpaceInformation(self.space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))
        si.setStateValidityCheckingResolution(CHECK_RES)
        si.setup()

        pdef = ob.ProblemDefinition(si)
        si_start = si.allocState()
        si_goal = si.allocState()
        for i in range(self.space.getDimension()):
            si_start[i] = float(self.start_state[i])
            si_goal[i] = float(self.goal_state[i])

        pdef.setStartAndGoalStates(si_start, si_goal)

        planner = og.RRTConnect(si)
        planner.setRange(SAMPLE_RANGE)
        planner.setProblemDefinition(pdef)
        planner.setup()

        start_time = time.time()
        solved = planner.solve(planning_time)
        end_time = time.time()
        if end_time - start_time > planning_time:
            print(f"Planning took longer than {planning_time} seconds. Replan it now, you can stop manually")
            path, total_time = self.plan(planning_time)
            return path, total_time
        if solved:
            print(f"Successfully found a RRT path with {pdef.getSolutionPath().getStateCount()} waypoints and post-pocessing it now")
            path = pdef.getSolutionPath()
            path_simplifier = og.PathSimplifier(si)
            path_simplifier.reduceVertices(path)
            # Interpolate is only used for path visualize, if there is a trajectory generation, it is not needed
            # path.interpolate(50)

            np_path = self._np_path(path)
            self.result_path = np_path

            total_time = time.time() - start_time
            print(f"Finish find a path with {path.getStateCount()} waypoints in {total_time:.6f} seconds")
            result_traj, result_vel, result_acc, duration = self.gen_trajectory(np_path, max_v=self.traj_max_vel)
            self.result_traj = result_traj
            return np_path, self.result_traj, total_time
        else:
            print(f"Failed to find a path for {self.which_arm} arm")
            self.result_path = None
            return None, None, None
        

class OMPLPlanCentralized:
    def __init__(self, space_dimension, arml_bounds, armr_bounds, arml_start, armr_start, arml_goal, armr_goal, fcl_checker: FCLChecker, planner_pin: Planner_Pin, use_ee: bool = False):
        if arml_bounds.shape[1] != int(space_dimension/2) or arml_bounds.shape[0] != 2:
            raise ValueError(f"Left Arm Bounds must be a 2x{space_dimension} array, got {arml_bounds.shape}")
        if armr_bounds.shape[1] != int(space_dimension/2) or armr_bounds.shape[0] != 2:
            raise ValueError(f"Right Arm Bounds must be a 2x{space_dimension} array, got {armr_bounds.shape}")
        if arml_start.shape[0] != int(space_dimension/2) or armr_start.shape[0] != int(space_dimension/2):
            raise ValueError(f"Left Arm Start state must have {int(space_dimension/2)} elements, got {arml_start.shape}")
        if arml_goal.shape[0] != int(space_dimension/2) or armr_goal.shape[0] != int(space_dimension/2):
            raise ValueError(f"Left Arm Goal state must have {int(space_dimension/2)} elements, got {arml_goal.shape}")
        
        self.space = ob.RealVectorStateSpace(space_dimension)
        self.bounds = ob.RealVectorBounds(space_dimension)
        for i in range(space_dimension):
            if i < int(space_dimension/2):
                self.bounds.setLow(i, arml_bounds[0,i])
                self.bounds.setHigh(i, arml_bounds[1,i])
            else:
                self.bounds.setLow(i, armr_bounds[0,i - int(space_dimension/2)])
                self.bounds.setHigh(i, armr_bounds[1,i - int(space_dimension/2)])
        self.space.setBounds(self.bounds)

        self.start_state = np.concatenate((arml_start, armr_start))
        self.goal_state = np.concatenate((arml_goal, armr_goal))
        self.fcl_checker = fcl_checker
        self.result_path = None
        self.result_traj = None

        self.head_qpos = None
        self.body_qpos = None

        # self.pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        # self.pin_data = self.pin_model.createData()
        self.planner_pin = planner_pin

        self.use_ee = use_ee

        self.traj_max_vel = 0.3

    # def _get_Kinematics(self, body_qpos, arml_qpos, armr_qpos, head_qpos):
    #     # NOTE: 
    #     # q's order depends on urdf, currently is [body(4), left arm(7), right arm(7), head(2)]
    #     # pinocchio will output 21 transformation matrices include universe as the first one
    #     if body_qpos is None or head_qpos is None:
    #         raise ValueError("body_qpos and head_qpos must be set before calling _get_Kinematics")
    #     q = np.concatenate((body_qpos, arml_qpos, head_qpos, armr_qpos))
    #     pinocchio.forwardKinematics(self.pin_model, self.pin_data, q)
    #     pinocchio.updateFramePlacements(self.pin_model, self.pin_data)
    #     transform_chain = [oMi.homogeneous for oMi in self.pin_data.oMi]
    #     left_transform_chain = transform_chain[5:12]  # left arm links
    #     right_transform_chain = transform_chain[14:21]  # right arm links
    #     if self.use_ee:
    #         armlee_ID = self.pin_model.getFrameId("LEFT_EE")  
    #         armree_ID = self.pin_model.getFrameId("RIGHT_EE")  
    #         lee_trans = self.pin_data.oMf[armlee_ID].homogeneous
    #         ree_trans = self.pin_data.oMf[armree_ID].homogeneous
    #         left_transform_chain.append(lee_trans)
    #         right_transform_chain.append(ree_trans)

    #     body_transform_chain = transform_chain[1:5]
    #     head_transform_chain = transform_chain[12:14]

    #     return body_transform_chain, left_transform_chain, right_transform_chain, head_transform_chain

    def set_traj_max_vel(self, max_vel):
        if max_vel <= 0:
            raise ValueError(f"max_vel must be positive, got {max_vel}")
        self.traj_max_vel = max_vel
        
    def set_head_body(self, head_qpos, body_qpos):
        if head_qpos.shape[0] != 2:
            raise ValueError(f"head_qpos must be a 2-element array, got {head_qpos.shape}")
        if body_qpos.shape[0] != 4:
            raise ValueError(f"body_qpos must be a 4-element array, got {body_qpos.shape}")
        self.head_qpos = head_qpos
        self.body_qpos = body_qpos
        real_qpos = np.concatenate((self.body_qpos, self.head_qpos, 
                                    self.start_state[:int(self.space.getDimension()/2)], 
                                    self.start_state[int(self.space.getDimension()/2):]))
        torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(real_qpos)
        # body, arml, armr, head = self._get_Kinematics(self.body_qpos, self.start_state[:int(self.space.getDimension()/2)], 
        #                                               self.start_state[int(self.space.getDimension()/2):], self.head_qpos)
        self.fcl_checker.set_head_body_transform(head_trans, torso_trans)

    def set_start_goal(self, arml_start, armr_start, arml_goal, armr_goal):
        if arml_start.shape[0] != int(self.space.getDimension()/2):
            raise ValueError(f"Left Arm Start state must have {int(self.space.getDimension()/2)} elements, got {arml_start.shape}")
        if armr_start.shape[0] != int(self.space.getDimension()/2):
            raise ValueError(f"Right Arm Start state must have {int(self.space.getDimension()/2)} elements, got {armr_start.shape}")
        if arml_goal.shape[0] != int(self.space.getDimension()/2):
            raise ValueError(f"Left Arm Goal state must have {int(self.space.getDimension()/2)} elements, got {arml_goal.shape}")
        if armr_goal.shape[0] != int(self.space.getDimension()/2):
            raise ValueError(f"Right Arm Goal state must have {int(self.space.getDimension()/2)} elements, got {armr_goal.shape}")

        self.start_state = np.concatenate((arml_start, armr_start))
        self.goal_state = np.concatenate((arml_goal, armr_goal))

    def _is_state_valid(self, state):
        arml_qpos = np.array([state[i] for i in range(int(self.space.getDimension()/2))], dtype=np.float32)
        armr_qpos = np.array([state[i] for i in range(int(self.space.getDimension()/2), self.space.getDimension())], dtype=np.float32)
        real_qpos = np.concatenate((self.body_qpos, self.head_qpos, arml_qpos, armr_qpos))
        torso_trans, head_trans, arml_trans, armr_trans = self.planner_pin.get_Kinematics(real_qpos)
        # body, arml, armr, head = self._get_Kinematics(self.body_qpos, arml_qpos=arml_qpos, armr_qpos=armr_qpos, head_qpos=self.head_qpos)
        transform = arml_trans + armr_trans
        self.fcl_checker.update_arm_transform(transform)

        return not self.fcl_checker.check_collision()
    
    def save_result_path(self, file_name):
        current_path = os.path.dirname(os.path.abspath(__file__))
        saved_folder_path = os.path.join(current_path, "saved_path")
        if not os.path.exists(saved_folder_path):
            os.makedirs(saved_folder_path)
        saved_file_path = os.path.join(saved_folder_path, file_name)
        with open(saved_file_path, "w", newline='') as f:
            writer = csv.writer(f)
            header = [f"left_q{i}" for i in range(int(self.space.getDimension()/2))] + \
                     [f"right_q{i}" for i in range(int(self.space.getDimension()/2))]
            writer.writerow(header)
            for i in range(self.result_path.shape[0]):
                row = self.result_path[i]
                writer.writerow(row)
    def _np_path(self, path):
        qpos_list = []
        for i in range(path.getStateCount()):
            state = path.getState(i)
            qpos = np.array([state[j] for j in range(self.space.getDimension())], dtype=np.float64)
            qpos_list.append(qpos)
        return np.array(qpos_list, dtype=np.float64)
    
    def gen_trajectory(self, path, max_v):
        # path_waypoints = []
        # for i in range(path.shape[0]):
        #     point = {"position": path[i, :].tolist()}
        #     path_waypoints.append(point)
        dofs = self.space.getDimension()
        vel_limits = [max_v]*dofs
        acc_limits = [5.0]*dofs
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, pos_array, vel_array, acc_array, duration = traj_planner.plan(path)
        return pos_array, vel_array, acc_array, duration

    def plan(self, planning_time=5.0):
        si = ob.SpaceInformation(self.space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))
        si.setStateValidityCheckingResolution(CHECK_RES)
        si.setup()

        pdef = ob.ProblemDefinition(si)
        si_start = si.allocState()
        si_goal = si.allocState()
        for i in range(self.space.getDimension()):
            si_start[i] = float(self.start_state[i])
            si_goal[i] = float(self.goal_state[i])

        pdef.setStartAndGoalStates(si_start, si_goal)

        planner = og.RRTConnect(si)
        planner.setRange(SAMPLE_RANGE)
        planner.setProblemDefinition(pdef)
        planner.setup()

        start_time = time.time()
        solved = planner.solve(planning_time)
        end_time = time.time()
        if end_time - start_time > planning_time:
            print(f"Planning took longer than {planning_time} seconds. Replan it now, you can stop manually")
            path, total_time = self.plan(planning_time)
            return path, total_time
        if solved:
            print(f"Successfully found a RRT path with {pdef.getSolutionPath().getStateCount()} waypoints and post-pocessing it now")
            path = pdef.getSolutionPath()
            path_simplifier = og.PathSimplifier(si)
            path_simplifier.reduceVertices(path)
            # Interpolate is only used for path visualize, if there is a trajectory generation, it is not needed
            # path.interpolate(50)

            np_path = self._np_path(path)
            self.result_path = np_path

            total_time = time.time() - start_time
            print(f"Finish find a path with {path.getStateCount()} waypoints in {total_time:.6f} seconds")
            result_traj, result_vel, result_acc, duration = self.gen_trajectory(np_path, max_v=self.traj_max_vel)
            self.result_traj = result_traj

            return np_path, self.result_traj, total_time
        else:
            print("Failed to find a path for dual arm Centralized planning")
            self.result_path = None
            return None, None, None
        
class OMPLPlanDecoupled:
    def __init__(self, arml_dim, armr_dim,
                 arml_bounds, armr_bounds,
                 arml_start, armr_start,
                 arml_goal, armr_goal,
                 fcl_checker: FCLChecker,
                 planner_pin: Planner_Pin,
                 use_ee: bool = False):
        if arml_bounds.shape[1] != arml_dim or arml_bounds.shape[0] != 2:
            raise ValueError(f"Left Arm Bounds must be a 2x{arml_dim} array, got {arml_bounds.shape}")
        if armr_bounds.shape[1] != armr_dim or armr_bounds.shape[0] != 2:
            raise ValueError(f"Right Arm Bounds must be a 2x{armr_dim} array, got {armr_bounds.shape}")
        if arml_start.shape[0] != arml_dim or armr_start.shape[0] != armr_dim:
            raise ValueError(f"Left Arm Start state must have {arml_dim} elements, got {arml_start.shape}")
        if arml_goal.shape[0] != arml_dim or armr_goal.shape[0] != armr_dim:
            raise ValueError(f"Left Arm Goal state must have {arml_dim} elements, got {arml_goal.shape}")
        
        self.use_ee = use_ee
        self.fcl_checker = fcl_checker
        # self.urdf_path = urdf_path
        self.planner_pin = planner_pin
        self.arml_planner = OMPLPlanSingle(arml_dim, arml_bounds, arml_start, arml_goal, self.fcl_checker, self.planner_pin, which_arm="left", other_arm_qpos=armr_start, use_ee=self.use_ee)
        self.armr_planner = OMPLPlanSingle(armr_dim, armr_bounds, armr_start, armr_goal, self.fcl_checker, self.planner_pin, which_arm="right", other_arm_qpos=arml_start, use_ee=self.use_ee)

        self.arml_dim = arml_dim
        self.armr_dim = armr_dim
        self.arml_start = arml_start
        self.armr_start = armr_start
        self.arml_goal = arml_goal
        self.armr_goal = armr_goal

        self.head_qpos = None
        self.body_qpos = None

        self.arml_result_trajectory = None
        self.armr_result_trajectory = None

        self.left_traj_max_vel = 0.3
        self.right_traj_max_vel = 0.3

    def set_traj_max_vel(self, left_max_vel, right_max_vel):
        if left_max_vel <= 0 or right_max_vel <= 0:
            raise ValueError(f"max_vel must be positive, got {left_max_vel} and {right_max_vel}")
        self.left_traj_max_vel = left_max_vel
        self.right_traj_max_vel = right_max_vel

    def set_head_body(self, head_qpos, body_qpos):
        if head_qpos.shape[0] != 2:
            raise ValueError(f"head_qpos must be a 2-element array, got {head_qpos.shape}")
        if body_qpos.shape[0] != 4:
            raise ValueError(f"body_qpos must be a 4-element array, got {body_qpos.shape}")
        self.head_qpos = head_qpos
        self.body_qpos = body_qpos
        self.arml_planner.set_head_body(head_qpos, body_qpos)
        self.armr_planner.set_head_body(head_qpos, body_qpos)
    
    def gen_trajectory(self, path, max_v, is_left=True):
        # path_waypoints = []
        # for i in range(path.shape[0]):
        #     point = {"position": path[i, :].tolist()}
        #     path_waypoints.append(point)
        if is_left:
            dofs = self.arml_dim
        else:
            dofs = self.armr_dim
        vel_limits = [max_v]*dofs
        acc_limits = [5.0]*dofs
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, pos_array, vel_array, acc_array, duration = traj_planner.plan(path)
        return pos_array, vel_array, acc_array, duration
    
    def save_result_path(self, file_name, is_left=True):
        current_path = os.path.dirname(os.path.abspath(__file__))
        saved_folder_path = os.path.join(current_path, "saved_path")
        if not os.path.exists(saved_folder_path):
            os.makedirs(saved_folder_path)
        saved_file_path = os.path.join(saved_folder_path, file_name)
        with open(saved_file_path, "w", newline='') as f:
            writer = csv.writer(f)
            if is_left:
                header = [f"left_q{i}" for i in range(self.space.getDimension())]
                trajectory = self.arml_result_trajectory
            else:
                header = [f"right_q{i}" for i in range(self.space.getDimension())]
                trajectory = self.armr_result_trajectory
            writer.writerow(header)
            for i in range(trajectory.shape[0]):
                row = self.trajectory[i]
                writer.writerow(row)

    def plan(self):
        arml_path, arml_trajectory, arml_plan_time = self.arml_planner.plan(planning_time=2.0)
        armr_path, armr_trajectory, armr_plan_time = self.armr_planner.plan(planning_time=2.0)
        if arml_path is None:
            # print("Failed to find a path for left arm")
            return None, None, None
        if armr_path is None:
            # print("Failed to find a path for right arm")
            return None, None, None
        
        start_time = time.time()
        arml_traj, arml_vel, arml_acc, arml_duration = self.gen_trajectory(arml_path, max_v=self.left_traj_max_vel, is_left=True)
        armr_traj, armr_vel, armr_acc, armr_duration = self.gen_trajectory(armr_path, max_v=self.right_traj_max_vel, is_left=False)

        coordinator = Coordination(arml_traj, armr_traj, self.fcl_checker, self.planner_pin, coordination_res=0.1, traj_res=0.01, use_ee=self.use_ee)
        coordinator.set_head_body(self.head_qpos, self.body_qpos)
        # coordinated_left, coordinated_right = coordinator.Coordinate_Astar()
        coordinated_left, coordinated_right = coordinator.Coordinate_RRT()
        end_time = time.time()

        self.arml_result_trajectory = coordinated_left
        self.armr_result_trajectory = coordinated_right

        total_time = arml_plan_time + armr_plan_time + (end_time - start_time)

        return coordinated_left, coordinated_right, total_time


