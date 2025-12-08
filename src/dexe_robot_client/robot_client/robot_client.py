# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------
from abc import ABC, abstractmethod
from .data_structure import *


class RobotClient(ABC):

    def __init__(self, is_sim: bool = False):
        self.is_sim = is_sim

        self.status = None
        self.robot_statusword = None
        self.robot_controlword = None
        self.run_mode = None

        self.latest_sent_id = -1
        self.latest_queued_id = -1
        self.latest_finished_id = -1
        self.latest_errordrop_id = -1

        self.joint_position_cmd = list[float]()
        self.joint_position = list[float]()
        self.joint_velocity = list[float]()
        self.joint_torque = list[float]()

        self.motor_status = list[int]()
        self.motor_error_code = list[int]()
        self.motor_temperature = list[int]()
        self.force_sensor = list[float]()

        self.torque_limit_negative = list[float]()
        self.torque_limit_positive = list[float]()

        self.ee_pose = list[float]()
        self.torso_pose_X_Z_RY = list[float]()

        self._move_request_id_returned = False

    """---------------------------------------------------------------------------
    User interfaces - setting/getting parameters, mode or state
    用户接口 - 设置/获取机器人参数, 模式或状态
    ----------------------------------------------------------------------------"""

    def get_info(self) -> RobotInfo:
        """Get robot state information.
        获取机器人状态信息
        Returns:
            RobotState: 机器人状态集合
        """
        return RobotInfo(
            status=self.status,
            robot_statusword=self.robot_statusword,
            robot_controlword=self.robot_controlword,
            run_mode=self.run_mode,
            latest_sent_id=self.latest_sent_id,
            latest_queued_id=self.latest_queued_id,
            latest_finished_id=self.latest_finished_id,
            latest_errordrop_id=self.latest_errordrop_id,
            joint_position_cmd=self.joint_position_cmd,
            joint_position=self.joint_position,
            joint_velocity=self.joint_velocity,
            joint_torque=self.joint_torque,
            motor_status=self.motor_status,
            motor_error_code=self.motor_error_code,
            motor_temperature=self.motor_temperature,
            force_sensor=self.force_sensor,
            torque_limit_negative=self.torque_limit_negative,
            torque_limit_positive=self.torque_limit_positive,
            ee_pose=self.ee_pose,
            torso_pose_X_Z_RY=self.torso_pose_X_Z_RY,
        )

    def set_tcp(
        self,
        tcp: list[float],
        arm: MotionGroup = MotionGroup.Primary,
    ) -> int:
        """async interface - setting robot TCP
        异步接口 - 设置机器人工具坐标系, 默认设置左臂tcp坐标系(机器人为双臂时), 单臂时即为机器人tcp坐标系
        Args:
        - tcp: 四元数形式位姿list[float]: [x, y, z, qw, qx, qy, qz]
        - arm: 可选项:
            - MotionGroup.PrimaryArm: 机器人为双臂时设置左臂tcp坐标系, 单臂时即为机器人tcp坐标系
            - MotionGroup.SecondaryArm: 机器人为双臂时设置右臂tcp坐标系, 单臂时无效返回-1
        Returns:
        - int: request id 指令号, 可用于等待对应运动指令完成等
        """

        if arm == MotionGroup.Primary:
            return self.send_set_property_request(PropertyType.TcpPrimary, tcp)
        elif arm == MotionGroup.SecondaryArm:
            return self.send_set_property_request(PropertyType.TcpSecondary,
                                                  tcp)
        return -1

    def set_global_speed(self, ratio: float) -> int:
        """async interface - setting robot speed ratio
        异步接口 - 设置机器人速度百分比
        Args:
            - ratio: float: 0.0 ~ 1.0
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        return self.send_set_property_request(PropertyType.SpeedRatio, ratio)

    def clear_error(self) -> int:
        """async interface - clearing robot error
        异步接口 - 清除机器人错误
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        return self.send_set_property_request(PropertyType.ClearError, None)

    def set_drag_mode(self, set_enable: bool) -> int:
        """async interface - setting robot drag mode
        异步接口 - 开关机器人拖动示教模式, 开启后仅可拖拽, 无法通过运动指令控制机器人运动, 仅真实机器人生效
        Args:
            - set_enable: bool: 是否开启拖动示教模式, True: 开启拖动示教模式, False: 关闭拖动示教模式
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        return self.send_set_property_request(PropertyType.EnableDrag,
                                              set_enable)

    def set_safe_level(self, safe_level: float) -> int:
        """async interface - setting robot safe level
        异步接口 - 设置机器人安全级别, 仅真实机器人生效
        Args:
            - safe_level: float: 安全级别, 取值范围 [0.1, 1], 默认为0.5,
                值越小机械臂柔性越大, 0.1时机械臂柔性最大, 1.0时机械臂刚性最大, 刚性越大运动跟踪性能越好, 但碰撞时容易造成机械臂损坏
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        return self.send_set_property_request(PropertyType.SafeLevel,
                                              safe_level)

    def set_impedance_mode(self, set_enable: bool) -> int:
        """async interface - setting robot impedance mode
        异步接口 - 开关机器人阻抗模式, 开启后机器人处于阻抗控制状态, 仅真实机器人生效
        Args:
            - set_enable: bool: 是否开启阻抗模式, True: 开启阻抗模式, False: 关闭阻抗模式
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        return self.send_set_property_request(PropertyType.EnableImpedance,
                                              set_enable)

    def set_impedance_mode(self, set_enable: bool) -> int:
        """async interface - setting robot impedance mode
        异步接口 - 开关机器人阻抗模式, 开启后机器人处于阻抗控制状态, 仅真实机器人生效
        Args:
            - set_enable: bool: 是否开启阻抗模式, True: 开启阻抗模式, False: 关闭阻抗模式
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        return self.send_set_property_request(PropertyType.EnableImpedance,
                                              set_enable)

    """ ---------------------------------------------------------------------------
    User interfaces - Move requests
    用户接口 - 运动指令
    ----------------------------------------------------------------------------"""

    def stop_move(self) -> int:
        """async interface - stop robot move
        异步接口 - 停止机器人运动, 会立即停止机器人当前运动并清除运动队列中未执行的运动指令
        Returns:
            - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        stop_move_request = MoveRequest(type=MotionType.StopMove)
        return self.send_move_robot_request(stop_move_request)

    def move_joint_async(
        self,
        motion_target: list[float],
        motion_group: MotionGroup = MotionGroup.Primary,
        speed: float = 1.0,
        **kwargs,
    ) -> int:
        """async interface - joint move to target joint angles
        异步接口 - 关节运动到目标关节角度, 如果机器人为双臂时默认设置MotionGroup.Primary为左臂运动
        Args:
        - target:
            - 关节角度list[float]: [j1, j2, j3, ...], 数据长度要与指定的motion_group一致

        - arm: 可选项:
            - MotionGroup.PrimaryArm: 机器人为双臂时左臂运动, target长度要与左臂关节数一致
            - MotionGroup.SecondaryArm: 机器人为双臂时右臂运动, target长度要与右臂关节数一致
            - MotionGroup.BothArms: 机器人为双臂时双臂运动, target长度要与双臂关节数相加一致, 左臂在前右臂在后:
                [j1_left, j2_left, ..., jn_left, j1_right, j2_right, ..., jn_right]
            - MotionGroup.WholeBody: 机器人为双臂时全身运动(包括腿和头), target长度要与全身关节数一致

        - speed: 速度百分比 0.0 ~ 1.0

        - kwargs: 可选参数
            - blend_tolerance: 平滑容忍度, 单位m
            - todo: 其他参数, 如触发拍照, 夹爪, 语音等动作

        Returns:
            int: request id 指令号, 可用于等待对应运动指令完成等
        """
        move_joint_request = self.make_move_request(
            motion_type=MotionType.MoveJoint,
            motion_group=motion_group,
            motion_target=motion_target,
            speed=speed,
            kwargs=kwargs,
        )

        return self.send_move_robot_request(move_joint_request)

    def move_joint_sync(
        self,
        motion_target: list[float],
        motion_group: MotionGroup = MotionGroup.Primary,
        speed: float = 1.0,
        **kwargs,
    ) -> int:
        """sync interface - joint move to target joint angles
        同步接口 - move_joint_async的同步版本接口, 会等待此request完成后再返回
        """
        request_id = self.move_joint_async(
            motion_target=motion_target,
            motion_group=motion_group,
            speed=speed,
            kwargs=kwargs,
        )
        self.wait_request_finish(request_id)
        return request_id

    def move_line_async(
        self,
        motion_target: Union[list[float], np.ndarray],
        motion_group: MotionGroup = MotionGroup.Primary,
        reference_frame: ReferenceFrame = ReferenceFrame.ArmBase,
        speed: float = 1.0,
        **kwargs,
    ) -> int:
        """async interface - line move to target cartesian pose
        异步接口 - 直线运动到目标笛卡尔位姿, 默认设置左臂运动(机器人为双臂时), 单臂时即为机器人当前运动
        Args:
        - target: list[float]: 四元数形式位姿: [x, y, z, qw, qx, qy, qz]
        - arm 可选项:
            - MotionGroup.PrimaryArm: 机器人为双臂时左臂运动, 单臂时即为机器人当前运动
            - MotionGroup.SecondaryArm: 机器人为双臂时右臂运动, 单臂时指令无效返回false
        - reference_frame: 参考坐标系可选项:
            - ReferenceFrame.ArmBase: 以臂基坐标系为参考坐标系
            - ReferenceFrame.WorldBase: 以世界坐标系为参考坐标系
        - speed: 速度百分比 0.0 ~ 1.0
        - kwargs: 可选参数
            - blend_tolerance: 平滑容忍度, 单位m
            - todo: 其他参数, 如触发拍照, 夹爪, 语音等动作
        Returns:
        - int: request id 指令号, 可用于等待对应运动指令完成等
        """
        move_line_request = self.make_move_request(
            motion_type=MotionType.MoveLine,
            motion_group=motion_group,
            motion_target=motion_target,
            reference_frame=reference_frame,
            speed=speed,
            kwargs=kwargs,
        )

        return self.send_move_robot_request(move_line_request)

    def move_line_sync(
        self,
        motion_target: Union[list[float], np.ndarray],
        motion_group: MotionGroup = MotionGroup.Primary,
        reference_frame: ReferenceFrame = ReferenceFrame.ArmBase,
        speed: float = 1.0,
        **kwargs,
    ) -> int:
        """sync interface - line move to target cartesian pose
        同步接口 - move_line_async的同步版本接口, 会等待此request完成后再返回
        """
        request_id = self.move_line_async(
            motion_group=motion_group,
            motion_target=motion_target,
            reference_frame=reference_frame,
            speed=speed,
            kwargs=kwargs,
        )
        self.wait_request_finish(request_id)
        return request_id

    def move_traj_async(
        self,
        motion_target: list[float],
        motion_group: MotionGroup,
        interval: float,
        endpoint_index: list[int] = None,
        **kwargs,
    ) -> int:
        """async interface - joint move to target joint angles
        异步接口 - 多点位轨迹运动, 每运动到指定的endpoint_index会更新self.latest_finished_id, 如果机器人为双臂时默认设置MotionGroup.Primary为左臂运动
        Args:
        - target:
            - 平铺多个点位的关节角度list[float]: [j1, j2, ..., j1, j2, ...], 数据长度为点位数乘以指定的motion_group关节数

        - arm: 可选项:
            - MotionGroup.PrimaryArm: 机器人为双臂时左臂运动, target长度要与左臂关节数一致
            - MotionGroup.SecondaryArm: 机器人为双臂时右臂运动, target长度要与右臂关节数一致
            - MotionGroup.BothArms: 机器人为双臂时双臂运动, target长度要与双臂关节数相加一致, 左臂在前右臂在后:
                [j1_left, j2_left, ..., jn_left, j1_right, j2_right, ..., jn_right]
            - MotionGroup.Torso: 仅躯干运动, target长度要与躯干关节数一致
            - MotionGroup.WholeBody: 机器人为双臂时全身运动(包括腿和头), target长度要与全身关节数一致

        - endpoint_index: 轨迹点中指定每段末尾的点位id, 如[4, 9]表示第4个点和第9个点为对应段的终点, 会在运行到这些点位时更新self.latest_finished_id

        - interval: 轨迹点时间间隔, 单位秒

        - kwargs: 可选参数
            - blend_tolerance: 平滑容忍度, 单位m

        Returns:
            int: request id 本次move_traj结束指令号, 可用于等待对应运动指令完成等, -1时为请求被拒绝
        """
        move_joint_request = self.make_move_request(
            motion_type=MotionType.MoveTraj,
            motion_group=motion_group,
            motion_target=motion_target,
            endpoint_index=endpoint_index,
            interval=interval,
            kwargs=kwargs,
        )

        return self.send_move_robot_request(move_joint_request)

    def move_joint_sync(
        self,
        motion_target: list[float],
        motion_group: MotionGroup = MotionGroup.Primary,
        speed: float = 1.0,
        **kwargs,
    ) -> int:
        """sync interface - joint move to target joint angles
        同步接口 - move_joint_async的同步版本接口, 会等待此request完成后再返回
        """
        request_id = self.move_joint_async(
            motion_target=motion_target,
            motion_group=motion_group,
            speed=speed,
            kwargs=kwargs,
        )
        self.wait_request_finish(request_id)
        return request_id

    """ ---------------------------------------------------------------------------
    User interfaces - streaming requests
    用户接口 - 流控制指令
    ----------------------------------------------------------------------------"""

    def start_joint_streaming_request(
        self,
        motion_group: MotionGroup = MotionGroup.Primary,
    ) -> int:
        """sync interface - start streaming control
        同步接口 - 开启一个流控制request, 运行流控制指令前需先调用此命令, 仅可在机器人Idle状态下调用, 可先调用wait_move_finish等待之前发送的指令完成让机器人进入Idle状态
        Returns:
            - int: request id 指令号
        """
        self.wait_request_finish()

        request = MoveRequest(type=MotionType.StartJointStreaming,
                              motion_group=motion_group)
        return self.send_move_robot_request(request)

    def start_ee_pose_streaming_request(
        self,
        motion_group: MotionGroup = MotionGroup.Primary,
    ) -> int:
        """sync interface - start streaming control
        同步接口 - 开启一个流控制request, 运行流控制指令前需先调用此命令, 仅可在机器人Idle状态下调用, 可先调用wait_move_finish等待之前发送的指令完成让机器人进入Idle状态
        Returns:
            - int: request id 指令号
        """
        self.wait_request_finish()

        request = MoveRequest(type=MotionType.StartEEPoseStreaming,
                              motion_group=motion_group)
        return self.send_move_robot_request(request)

    def send_streaming_command(
        self,
        position: list[float],
        streaming_finished: bool = False,
    ) -> bool:
        """async interface - send joint streaming control point, need to switch to joint streaming mode first
        异步接口 - 关节流控制, 最新下发的指令会覆盖之前的指令, 需先调用start_streaming_mode
        Args:
        - position:
            JointPosition: [j1, j2, j3, ...], 长度与机器人关节数一致, 对应MotionType.StartJointStreaming
        - streaming_finished: 是否结束streaming, true - 流控制结束, false - 流控制继续
        Returns:
            - bool: true if success, false if fail
        """
        return self.send_streaming_command(position, streaming_finished)

    """ ---------------------------------------------------------------------------
    User interfaces - make move requests functions
    用户接口 - 运动指令生成
    ----------------------------------------------------------------------------"""

    def make_move_request(
        self,
        motion_type: MotionType,
        motion_group: MotionGroup = MotionGroup.Primary,
        motion_target: Optional[list[float]] = None,
        speed: Optional[float] = None,
        reference_frame: Optional[ReferenceFrame] = None,
        **kwargs,
    ) -> MoveRequest:
        """create a move joint request
        创建一个关节运动指令
        Args:
            - type: MotionType: 运动类型
            - motion_group: 运动部位
            - value: 目标位置
            - reference_frame: 参考坐标系
            - speed: 运动速度

        - kwargs 可选项
            - blend_tolerance: 平滑容忍度, 单位m
        Returns:
            MoveRequest: 运动指令
        """
        move_joint_request = MoveRequest(
            type=motion_type,
            motion_group=motion_group,
            value=motion_target,
            reference_frame=reference_frame,
            speed=speed,
            kwargs=kwargs,
        )
        return move_joint_request

    def send_move_request_list(self,
                               request_list: list[MoveRequest]) -> list[int]:
        """async interface - execute request list
        异步接口 - 执行请求队列
        Args:
            - request_list: list[MoveRequest] 运动请求队列下发
        Returns:
            - list[int]: request id 请求号队列
        """
        request_ids = []
        for request in request_list:
            request_id = self.send_move_robot_request(request)
            request_ids.append(request_id)
        return request_ids

    """---------------------------------------------------------------------------
    Low-level abstract interfaces, need to be implemented by derived classes
    底层抽象接口, 需由派生类实现
    ----------------------------------------------------------------------------"""

    @abstractmethod
    def connect_robot(self, timeout_sec: float = 0.5, ip: str = None) -> bool:
        """Connect to the robot.
        连接机器人
        Args:
            - ip: str: 机器人IP地址 (Ros2连接无需指定)
        Returns:
            - bool: True if connected successfully, False otherwise
        """
        pass

    @abstractmethod
    def wait_second(self, wait_second: float):
        """sync interface - wait for a specified time in seconds
        同步接口 - 等待指定时间, 单位秒
        Args:
            wait_second: float: 等待时间, 单位秒
        """
        pass

    @abstractmethod
    def wait_request_finish(self,
                            request_id: int,
                            timeout: float = 20.0) -> bool:
        """sync interface - wait for move request to finish
        同步接口 - 等待运动请求完成
        Args:
            request_id: int: 运动请求号
            timeout: float: 超时时间, 默认20秒
        Returns:
            bool: true if success, false if fail or timeout
        """
        pass

    @abstractmethod
    def send_set_property_request(self, type: PropertyType, value) -> bool:
        """async interface - setting interfaces for all types of properties
        异步接口 - 设置机器人属性底层通用接口
        Args:
          type 可选项:                 对应 value 类型:
            PropertyType.SpeedRatio          ->  float: 0.0 ~ 1.0
            PropertyType.TcpPrimary          ->  PoseQuaternion
            PropertyType.TcpSecondary        ->  PoseQuaternion
            PropertyType.ClearError          ->  None
        Returns:
            bool: true if success, false if fail
        """
        pass

    @abstractmethod
    def send_move_robot_request(self, request: MoveRequest) -> int:
        """async interface - execute request
        异步接口 - 执行指令通用接口
        Args:
            request: MoveRequest 运动指令下发
        Returns:
            int: request id 指令号
        """
        pass

    @abstractmethod
    def send_streaming_command(self,
                               position: np.ndarray,
                               streaming_finished: bool = False) -> bool:
        """async interface - send streaming control point, need to start streaming mode first
        异步接口 - 发送流控制点位, 需先调用start_streaming_request
        Args:
            position: np.ndarray, 包含目标位置
            streaming_finished: 是否是本次流控制最后的点位, true - 停止流控制, false - 继续流控制
        Returns:
            bool: true - send success, false - send fail
        """
        pass

    @abstractmethod
    def close_client(self):
        """sync interface - stop client from robot server
        同步接口 - 关闭client, 不会影响server正在进行的动作
        """
        pass
