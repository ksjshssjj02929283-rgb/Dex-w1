# -*- coding: utf-8 -*-
import socket
import fcntl
import ast
import asyncio
import json
import logging
import os
import re
import sys
import threading
import websockets
import time
import copy
import numpy as np
from datetime import datetime
from pathlib import Path
from PySide6.QtCore import Qt,QThread, Signal,QTimer, QObject, Slot
from PySide6.QtWidgets import QFileDialog,QMainWindow,QMessageBox,QDialog,QApplication,QProgressDialog, QLabel, QVBoxLayout
from ui_display import W1Displayui
from IPython import embed
import subprocess
from concurrent.futures import ThreadPoolExecutor
import queue
app_dir = Path(__file__).parent.parent 
sys.path.insert(0, str(app_dir)) 
from application.initialize import Initialize, ColorLogger
from application.basic_function import VisionApp, RobotApp, Function, set_ws
import asyncio
import concurrent.futures
from json_editor import show_json_editor  
from scipy.spatial.transform import Rotation as R
from w1_flaskServer import FlaskServerThread

import requests
import yaml
_global_lock = threading.Lock()
from operator import index
import order_detail
import netifaces
from rclpy.qos import qos_profile_sensor_data
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor


class PC1LogSaveSimpleThread(QThread):
    finished_signal = Signal(bool, str) # 是否成功, 消息

    def __init__(self, script_path):
        super().__init__()
        self.script_path = script_path
        self.process = None

    def run(self):
        try:
            # 直接在新进程中全速运行脚本
            self.process = subprocess.Popen([sys.executable, self.script_path])
            self.process.wait() # 阻塞当前子线程，直到脚本运行完毕
            if self.process.returncode == 0:
                self.finished_signal.emit(True, "🎉 PC1 日志传输全部完成！已保存至本地。")
            else:
                self.finished_signal.emit(False, f"❌ 传输异常中断，返回码: {self.process.returncode}")
        except Exception as e:
            self.finished_signal.emit(False, f"❌ 启动传输脚本失败: {str(e)}")

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()

class LoadingDialog(QDialog): 
    """自定义加载对话框"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("加载配置")
        self.setModal(True)
        self.setFixedSize(300, 120)
        self.setWindowFlags(Qt.Dialog | Qt.CustomizeWindowHint | Qt.WindowTitleHint)
        layout = QVBoxLayout(self)
        self.status_label = QLabel("初始化中...")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        self.progress_label = QLabel("进度: 0%")
        self.progress_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.progress_label)

    def update_status(self, status, progress=None):
        """更新状态和进度"""
        self.status_label.setText(status)
        if progress is not None:
            self.progress_label.setText(f"进度: {progress}%")
        QApplication.processEvents()  # 确保UI更新

class ExecuteTaskThread(QThread):

    def __init__(self, w1_controller=None, voice_msg="", module_name="",
                 task_config=None, policy_need_resume=None,
                 task_request_serial=None):
        super().__init__()
        self.voice_msg = voice_msg
        self.module_name = module_name
        self.w1_controller = w1_controller
        self._stop_event = threading.Event()
        self.task_config = task_config
        self.policy_need_resume = policy_need_resume
        self.task_request_serial = (
            task_request_serial if task_request_serial is not None else
            getattr(w1_controller, "_task_request_serial", 0))


    def stop(self):
        self._stop_event.set()

    def run(self):
        actual_module_name = self.module_name if self.module_name != "" else "语音连续运行"
        
        # 退桩前若已由 on_execute 挂起 Policy，这里继承恢复责任，不能再次
        # 调用后得到 False，否则任务结束后将无人恢复 Policy。
        if self.policy_need_resume is None:
            need_resume = self.w1_controller.try_suspend_policy_service()
        else:
            need_resume = bool(self.policy_need_resume)
        
        task_success = False 
        
        try:
            # 3. 执行主任务
            future = self.w1_controller.model_manager.submit_inference(
                self._execute_task_with_vision, actual_module_name
            )
            
            # 获取结果
            result = future.result(timeout=300)
            if result is True:
                task_success = True
            
        except Exception as e:
            logging.error(f"任务执行异常: {str(e)}")
            task_success = False
            
        finally:
            # 底盘占用状态必须在所有任务退出路径中释放。该调用放在 Policy
            # 恢复逻辑之前，避免下面的提前 return 使低电量待回充永远不再执行。
            boundary_success = (
                self.w1_controller._on_execute_task_finished(self))
            if boundary_success is False:
                task_success = False

            # 只有当：1. 原本处于 ACT 模式 (need_resume) 且 2. 任务执行成功 (task_success) 时，才恢复
            if need_resume:
                # 判断自己是否还是主控制器的最新活动线程
                is_latest_thread = True
                if hasattr(self.w1_controller, 'thread') and self.w1_controller.thread is not None:
                    if self.w1_controller.thread is not self:
                        is_latest_thread = False
                if (getattr(self.w1_controller, "_task_request_serial", 0) !=
                        self.task_request_serial):
                    is_latest_thread = False

                # 检查是否被硬打断 只要 stop_squence 为 True
                is_interrupted = getattr(self.w1_controller.function, "stop_squence", False)

                # 如果不是最新线程，说明被新任务插队了，回到任务
                if not is_latest_thread:
                    logging.info("⚠️ 检测到有新任务插队，当前旧任务静默退出，移交 Policy 恢复权")
                    return 

                # 是最新线程，但是中途被打断/报错，不恢复
                if is_interrupted or not task_success:
                    logging.info("❌ 任务异常、失败或被中止，保持静止，不恢复 Policy")
                    return 

                # 最新且完成的线程，才进入等待期
                wait_time = 5.0  
                check_interval = 0.1
                elapsed = 0.0
                cancel_resume = False
                
                logging.info(f"✅ 任务圆满完成，进入 Policy 恢复等待期 ({wait_time} 秒)...")
                
                while elapsed < wait_time:
                    # 1. 检查是否在等待期间被新任务顶替
                    if ((hasattr(self.w1_controller, 'thread') and
                         self.w1_controller.thread is not None and
                         self.w1_controller.thread is not self) or
                            getattr(self.w1_controller,
                                    "_task_request_serial", 0) !=
                            self.task_request_serial):
                        logging.info("🛑 在等待期间检测到新任务触发，取消当前 Policy 恢复！")
                        cancel_resume = True
                        break

                    # 2. 检查是否在等待期间被硬停止
                    if getattr(self.w1_controller.function, "stop_squence", False):
                        logging.info("🛑 在等待期间检测到全局停止指令，取消当前 Policy 恢复！")
                        cancel_resume = True
                        break

                    time.sleep(check_interval)
                    elapsed += check_interval
                
                # 过等待期，执行恢复
                if not cancel_resume:
                    logging.info("✅ 等待期平稳结束，正式请求恢复 Policy...")
                    self.w1_controller.try_resume_policy_for_task(
                        self.task_request_serial)
    
    def _execute_task_with_vision(self, actual_module_name):
        self.w1_controller.config['parameter_cfg']["workspace"] = self.task_config["workspace"]
        self.w1_controller.function.set_config(self.w1_controller.config)

        if self.w1_controller.parameter_cfg["use_picklight_vision"]["use"]:
            picklight_vision_config = [
                self.w1_controller.vision_logger,
                self.w1_controller.vision_param,
            ]
            return self.w1_controller.function.execute_origin_tasks(
                actual_module_name,
                text=self.voice_msg,
                picklight_vision_cfg=picklight_vision_config,
                is_local=self.w1_controller.is_local,
            )
        else:
            return self.w1_controller.function.execute_origin_tasks(
                actual_module_name,
                text=self.voice_msg,
                is_local=self.w1_controller.is_local,
            )
        # finally:
        # self.function.robot_status='idle'
        # self.w1_controller.voice_task_running = "finished"
        # self.w1_controller.update_light(self.w1_controller.G4, False)

class FaceTrackingThread(QThread):
    finished_signal = Signal(bool, str)
    
    def __init__(self, picklight_vision, w1_manager, vision_cfg,w1_controller):
        super().__init__()
        self.picklight_vision = picklight_vision
        self.w1_manager = w1_manager
        self.vision_cfg = vision_cfg
        self.w1_controller = w1_controller      
    def run(self):
        try:
            # 设置人脸跟踪状态
            self.picklight_vision.face_tracking_status = True
            self.picklight_vision.w1_manager = self.w1_manager
            
            # 启动人脸检测（在后台线程中运行）
            self.picklight_vision.face_detect(self.vision_cfg,w1_controller=self.w1_controller,face_tracking_status=True)
            
            # self.finished_signal.emit(True, "人脸跟踪已启动")
        except Exception as e:
            self.finished_signal.emit(False, f"人脸跟踪启动失败: {str(e)}")

class PolicyControlThread(QThread):
    """Policy 冷启动线程 — 逻辑已委托 PolicyBridge"""
    finished_signal = Signal(bool, str)

    def __init__(self, function, work_name="Auto_Policy_Start"):
        super().__init__()
        self.function = function
        self.work_name = work_name

    def run(self):
        try:
            if not self.function:
                self.finished_signal.emit(False, f"[{self.work_name}] 失败: Function 实例未加载")
                return
            # 方案C: basic_function 未接线 policy_bridge 时懒实例化
            if getattr(self.function, 'policy_bridge', None) is None:
                from application.policy_function import PolicyBridge
                self.function.policy_bridge = PolicyBridge(self.function)
            logging.info(f"[{self.work_name}] 开始异步初始化 Policy 服务...")
            success, message = self.function.policy_bridge.init_and_run_motion_from_origin(self.work_name)
            self.finished_signal.emit(success, message)
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            logging.error(f"[{self.work_name}] 线程异常:\n{error_details}")
            self.finished_signal.emit(False, f"[{self.work_name}] 运行异常: {str(e)}")

class RosBatteryMonitorThread(QThread):
    """基于ROS2诊断信息的电量监控线程"""
    update_signal = Signal(str)
    refresh_signal = Signal(str)
    charging_state_signal = Signal(bool, bool, bool, float)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.running = True
        self.node = None
        self.executor = None
        self._battery_sample_condition = threading.Condition()
        self._latest_battery_text = None
        self._latest_battery_sample_time = None

    def _store_battery_sample(self, battery_text: str,
                              sample_time: float) -> None:
        """缓存每帧完整电量数据，供任务边界主动刷新显示。"""
        with self._battery_sample_condition:
            self._latest_battery_text = battery_text
            self._latest_battery_sample_time = sample_time
            self._battery_sample_condition.notify_all()

    def refresh_latest_battery(self, reason: str,
                               timeout: float = 3.0) -> bool:
        """等待一帧新诊断后刷新UI；超时则使用最近一次有效样本。"""
        request_time = time.monotonic()
        with self._battery_sample_condition:
            deadline = request_time + timeout
            while (self.running and
                   (self._latest_battery_sample_time is None or
                    self._latest_battery_sample_time <= request_time)):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._battery_sample_condition.wait(timeout=remaining)

            battery_text = self._latest_battery_text
            sample_time = self._latest_battery_sample_time

        if battery_text is None or sample_time is None:
            logging.warning(
                f"[电量刷新][{reason}] 等待 {timeout:.0f} 秒仍无有效电量样本。")
            return False

        sample_age = max(0.0, time.monotonic() - sample_time)
        self.refresh_signal.emit(battery_text)
        compact_text = battery_text.replace('\n', '，')
        logging.info(
            f"[电量刷新][{reason}] {compact_text}，"
            f"样本年龄={sample_age:.1f}秒。")
        return True

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from diagnostic_msgs.msg import DiagnosticArray
        except ImportError:
            print("未找到ROS2环境或 diagnostic_msgs，电量监控已禁用")
            return

        class BatteryNode(Node):
            def __init__(self, update_sig, charging_state_sig,
                         sample_callback, context=None):
                # 🌟 修改点 2：必须把 context 透传给父类 Node
                super().__init__('ui_battery_monitor', context=context)
                self.update_sig = update_sig
                self.charging_state_sig = charging_state_sig
                self.sample_callback = sample_callback
                self.subscription = self.create_subscription(
                    DiagnosticArray,
                    '/diagnostics',
                    self.battery_callback,
                    qos_profile_sensor_data)
                self._last_battery_percentage = -1.0
                self._last_update_time = time.time()
                self._battery_charging = None
                self._charger_connected = None
                self._charging_current_detected = None

            @staticmethod
            def parse_bool(value):
                return str(value).strip().lower() in {
                    'true', '1', 'yes', 'on'
                }
            ####################################原来的代码逻辑 ###################
            # def battery_callback(self, msg):
            #     try:
            #         charging_state_changed = False
            #         for status in msg.status:
            #             # 放宽匹配条件，只要包含 PowerBoard 就行
            #             if "PowerBoard" in status.name:
            #                 for value in status.values:
            #                     if value.key == "battery_percentage":
            #                         # 过滤掉可能的 "%" 等非数字字符，提升稳定性
            #                         val_str = str(value.value).replace('%', '').strip()
            #                         percentage = float(val_str)
                                    
            #                         current_time = time.time()

            #                         # --- 新增：独立的日志定时器 ---

            #                         # last_log_time = getattr(self, '_last_log_time', 0.0)
            #                         # if current_time - last_log_time >= 2.0:
            #                         #     logging.warning(f"[Battery Log] 当前电量: {percentage}%")
            #                         #     self._last_log_time = current_time

            #                         # ------------------------------ 

            #                         # 防抖：2秒内或电量改变才更新一次UI
            #                         if current_time - self._last_update_time > 2.0 or percentage != self._last_battery_percentage:
            #                             self._last_battery_percentage = percentage
            #                             self._last_update_time = current_time
            #                             # 格式需要符合原UI的切割要求：包含“电量：XX”
            #                             self.update_sig.emit(f"电量：{int(percentage)}")
            #                     elif value.key == "battery_charging":
            #                         battery_charging = self.parse_bool(value.value)
            #                         if battery_charging != self._battery_charging:
            #                             self._battery_charging = battery_charging
            #                             charging_state_changed = True
            #                     elif value.key == "charger_connect_state":
            #                         charger_connected = self.parse_bool(value.value)
            #                         if charger_connected != self._charger_connected:
            #                             self._charger_connected = charger_connected
            #                             charging_state_changed = True

            #         if charging_state_changed:
            #             self.charging_state_sig.emit(
            #                 bool(self._battery_charging),
            #                 bool(self._charger_connected))
            #     except Exception as e:
            #         # 🚨 把报错打印出来，别直接 pass 被吞掉
            #         print(f"ROS 解析电量回调发生异常: {e}")

            def battery_callback(self, msg):
                try:
                    charging_state_observed = False

                    for status in msg.status:
                        if "PowerBoard" not in status.name:
                            continue

                        battery_info = {
                            value.key: str(value.value).strip()
                            for value in status.values
                        }

                        if "battery_percentage" in battery_info:
                            percentage_str = battery_info["battery_percentage"].replace('%', '').strip()
                            percentage = float(percentage_str)

                            voltage = battery_info.get("battery_voltage", "--")
                            current = battery_info.get("battery_current", "--")
                            battery_text = (
                                f"电量：{int(percentage)}\n"
                                f"电压：{voltage}V\n"
                                f"电流：{current}A")
                            self.sample_callback(
                                battery_text, time.monotonic())

                            current_time = time.time()

                            if percentage != self._last_battery_percentage:
                                self._last_battery_percentage = percentage
                                self._last_update_time = current_time

                                print(
                                    f"[Battery] 电量: {int(percentage)}%, "
                                    f"电压: {voltage}V, "
                                    f"电流: {current}A"
                                )

                                self.update_sig.emit(battery_text)

                        if "battery_charging" in battery_info:
                            battery_charging = self.parse_bool(battery_info["battery_charging"])
                            self._battery_charging = battery_charging

                        if "charger_connect_state" in battery_info:
                            charger_connected = self.parse_bool(battery_info["charger_connect_state"])
                            self._charger_connected = charger_connected

                        # 实机日志已确认：放电约为负电流，插桩充电约为 +35A。
                        # 当部分固件缺少充电布尔字段时，用明显正向电流作为辅助证据。
                        charging_current_observed = False
                        if "battery_current" in battery_info:
                            try:
                                current_value = float(
                                    battery_info["battery_current"]
                                    .replace('A', '').replace('a', '').strip())
                                self._charging_current_detected = (
                                    current_value > 1.0)
                                charging_current_observed = True
                            except ValueError:
                                pass

                        charging_state_observed = (
                            charging_state_observed or
                            "battery_charging" in battery_info or
                            "charger_connect_state" in battery_info or
                            charging_current_observed)

                    # 连续稳定判断需要每一帧诊断样本，不能只在状态变化时发送。
                    # 同时携带采样时间，防止主线程阻塞后把排队的旧样本当成新状态。
                    if charging_state_observed:
                        self.charging_state_sig.emit(
                            bool(self._battery_charging),
                            bool(self._charger_connected),
                            bool(self._charging_current_detected),
                            time.monotonic()
                        )

                except Exception as e:
                    print(f"ROS 解析电量回调发生异常: {e}")
        while self.running:
            # 电量监控独占一套 Context + Executor，不能通过 rclpy.spin_once()
            # 借用全局 Executor，否则会与 Policy 的 ROS 订阅争用 WaitSet。
            isolated_context = Context()
            executor = None
            try:
                rclpy.init(context=isolated_context)
                self.node = BatteryNode(
                    self.update_signal,
                    self.charging_state_signal,
                    self._store_battery_sample,
                    context=isolated_context)
                executor = SingleThreadedExecutor(context=isolated_context)
                self.executor = executor
                executor.add_node(self.node)

                while self.running and rclpy.ok(context=isolated_context):
                    executor.spin_once(timeout_sec=0.5)

            except Exception as e:
                if self.running:
                    logging.error(
                        f"🚨 ROS电量监控节点出错: {e}，2秒后尝试重启节点...")
                    # 分段等待，关闭程序时不必固定再等2秒。
                    for _ in range(20):
                        if not self.running:
                            break
                        time.sleep(0.1)

            finally:
                if executor is not None:
                    if self.node is not None:
                        try:
                            executor.remove_node(self.node)
                        except Exception:
                            pass
                    try:
                        executor.shutdown()
                    except Exception as e:
                        if self.running:
                            logging.warning(
                                f"ROS电量监控 Executor 关闭异常: {str(e)}")
                self.executor = None

                if self.node is not None:
                    self.node.destroy_node()
                    self.node = None

                if rclpy.ok(context=isolated_context):
                    rclpy.shutdown(context=isolated_context)

    def stop_monitoring(self):
        self.running = False
        with self._battery_sample_condition:
            self._battery_sample_condition.notify_all()
        if self.executor is not None:
            try:
                self.executor.wake()
            except Exception:
                pass

class StatusMonitorThread(QThread):  
    """仅保留TCP命令连接功能，不再轮询OP和电量"""
    def __init__(self, controller, device_cfg, is_sim=False):
        super().__init__()
        self.controller = controller
        self.device_cfg = device_cfg
        self.is_sim = is_sim
        self.running = True
        self.client_socket = None
        self.client_connected = False
        self.tcp_monitoring_enabled = True

    def run(self):
        while self.running:
            try:
                # 仅维持连接状态，不再发送 getbat 和 getop
                if not self.is_sim and not self.client_connected:
                    self._try_reconnect()
            except Exception as e:
                self.client_connected = False
            time.sleep(2)

    def _try_reconnect(self):
        try:
            reconnect_msg = self._start_client()
            if self.client_connected:
                logging.info(f"TCP通道连接成功: {reconnect_msg}")
        except Exception:
            pass

    def _start_client(self):
        try:
            server_host = self.device_cfg["w1_tcp_config"]["host"]
            server_port = self.device_cfg["w1_tcp_config"]["port"]

            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(1.0)
            self.client_socket.connect((server_host, server_port))
            self.client_connected = True
            self.client_socket.settimeout(None)
            return f"已连接到服务器 {server_host}:{server_port}"
        except socket.timeout:
            self.client_connected = False
            return "连接超时"
        except ConnectionRefusedError:
            self.client_connected = False
            return "连接被拒绝"
        except Exception as e:
            self.client_connected = False
            return f"发生错误: {e}"

    def stop_monitoring(self):
        self.running = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
        self.client_connected = False

class VoiceServiceWorker(QObject):
    # 给主线程回传的三类信号
    log_sig      = Signal(str)          # 普通日志
    error_sig    = Signal(str)          # 异常日志
    started_sig  = Signal(bool)         # True=启动成功  False=启动失败
    finished_sig = Signal()             # 停止完成

    def __init__(self, user_config_path, main_controller):
        super().__init__()
        self.user_config_path = user_config_path
        self.main_controller  = main_controller
        self.voice_controller = None

    @Slot()
    def start_server(self):
        try:
            # —— 1. 音频设备脚本（子线程里执行无妨）
            subprocess.run(
                ["bash", "/home/dexforce/workspace/dexe_application/qt/sound.sh"],
                check=True, timeout=10
            )
            self.log_sig.emit("音频设备设置完成")

            # —— 2. 导入并实例化 VoiceController（第一次导入略慢）
            from tools.voice_controller import VoiceController
            self.voice_controller = VoiceController(
                config_path=self.user_config_path,
                controller=self.main_controller,
                log_callback=lambda msg: self.log_sig.emit(msg)
            )

            # —— 3. 真正耗时的地方
            self.voice_controller.start_server()
            self.log_sig.emit("语音服务启动成功！")
            self.started_sig.emit(True)

        except ImportError as e:
            self.error_sig.emit(f"导入语音控制器失败: {e}")
            self.started_sig.emit(False)
        except Exception as e:
            self.error_sig.emit(f"语音服务启动失败: {e}")
            self.started_sig.emit(False)

    @Slot()
    def stop_server(self):
        """子线程入口：停止服务"""
        try:
            if self.voice_controller:
                self.voice_controller.stop_server()
                self.log_sig.emit("语音服务已停止")
        except Exception as e:
            self.error_sig.emit(f"停止语音服务时出错: {e}")
        finally:
            self.finished_sig.emit()

class DexAgentThread(threading.Thread):  # 语音服务线程
    """异步通信线程"""

    task_received_signal = Signal(str)  # 新增：传递任务文本

    def __init__(self, task_handler):
        super().__init__(daemon=True)
        self.loop = asyncio.new_event_loop()
        self.websocket = None
        self.task_handler = task_handler
        set_ws(self)

    def run(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._main())
        except Exception as e:
            pass
        finally:
            self.loop.close()

    async def _stop_async(self):
        """异步停止方法"""
        if self.websocket:
            try:
                await self.websocket.close()
            except:
                pass

    async def _main(self):
        async with websockets.connect("ws://localhost:9001/ws") as ws:
            self.websocket = ws
            handler_task = asyncio.create_task(self.message_handler(ws))

            try:
                while True:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                pass
            finally:
                handler_task.cancel()
                await ws.close()

    async def message_handler(self, websocket):
        """消息处理器"""
        try:
            while True:
                message = await websocket.recv()
                if self.task_handler.global_state == "stopping":
                    if (
                        message.startswith("TASK_REQ:")
                        and message.split(":", 1)[1].strip() == "task_terminate"
                    ):
                        logging.info(
                            f"任务停止中,不要着急: {message}"
                        )
                        if (
                            hasattr(self.task_handler, "function")
                            and self.task_handler.function
                        ):
                            self.task_handler.function.send_voice(
                                "TASK_RES: stopping-success"
                            )
                        continue
                elif message.startswith("TASK_REQ:"):
                    task_text = message.split(":", 1)[1].strip()
                    self.task_handler._write_status_file("agent_msg", task_text)
                    self.task_handler.on_execute(voice_msg=task_text)
                elif message.startswith("HEARTBEAT:"):
                    pass
                # # 新增：处理支付状态消息
                # elif message.startswith("PAYMENT_FEATURE_STATUS:"):
                #     status = message.split(":", 1)[1].strip().lower()
                #     if hasattr(self.task_handler, 'payment_enabled'):
                #         if status == "enabled":
                #             self.task_handler.payment_enabled = True
                #         elif status == "disabled":
                #             self.task_handler.payment_enabled = False
                #         print(f"支付功能状态已更新: {self.task_handler.payment_enabled}")
                else:
                    self.send_msg("TASK_DONE: UNSUPPORTED")
        except websockets.exceptions.ConnectionClosed:
            pass

    def send_msg(self, text):
        async def _send(_text):
            if not self.websocket:
                return
            await self.websocket.send(_text)

        asyncio.run_coroutine_threadsafe(_send(text), self.loop)

    def stop(self):
        """停止线程"""
        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self._stop_async(), self.loop)
            time.sleep(0.5)
            self.loop.call_soon_threadsafe(self.loop.stop)

class HTTPServerThread(QThread):  # HTTP服务线程
    log_signal = Signal(str)
    coffee_signal = Signal(str, str)
    consult_signal = Signal(str, str)
    stop_signal = Signal(str)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.running = False
        self.httpd = None  # 添加对HTTP服务器的引用

    def run(self):
        """运行HTTP服务器"""
        try:
            from http_server import HTTPServer, RequestHandler, add_log_callback
            from http_server import (
                add_coffee_callback,
                add_consult_callback,
                add_stop_callback,
            )
            from http_server import (
                remove_log_callback,
                remove_coffee_callback,
                remove_consult_callback,
                remove_stop_callback,
            )

            add_log_callback(self.handle_log)
            add_coffee_callback(self.handle_coffee_order)
            add_consult_callback(self.handle_consult_message)
            add_stop_callback(self.handle_stop_motion)
            self.running = True
            self.log_signal.emit("启动HTTP服务器...")
            server_address = ("", 8000)
            self.httpd = HTTPServer(server_address, RequestHandler)
            self.httpd.timeout = 1
            local_ip = self.get_local_ip()
            self.log_signal.emit(f"服务器启动在: http://{local_ip}:8000")
            while self.running:
                self.httpd.handle_request()
        except Exception as e:
            self.log_signal.emit(f"HTTP服务器错误: {str(e)}")
            import traceback

            self.log_signal.emit(f"详细错误: {traceback.format_exc()}")
        finally:
            self.running = False
            try:
                from http_server import (
                    remove_log_callback,
                    remove_coffee_callback,
                    remove_consult_callback,
                    remove_stop_callback,
                )

                remove_log_callback(self.handle_log)
                remove_coffee_callback(self.handle_coffee_order)
                remove_consult_callback(self.handle_consult_message)
                remove_stop_callback(self.handle_stop_motion)
            except:
                pass
            self.log_signal.emit("HTTP服务器已停止")

    def get_local_ip(self):
        """获取本机IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"

    def stop(self):
        """停止HTTP服务器"""
        if self.running:
            self.running = False
            if self.httpd:
                try:
                    import socket

                    try:
                        dummy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        dummy_socket.settimeout(1)
                        dummy_socket.connect(("localhost", 8000))
                        dummy_socket.close()
                    except:
                        pass
                except:
                    pass
            self.wait(2000)  # 等待2秒
            if self.isRunning():
                self.terminate()
                self.wait()

    def handle_stop_motion(self, client_ip):
        """处理停止运动"""
        self.stop_signal.emit(client_ip)

    def drag_on(self, client_ip):
        """处理drag_on运动 - 通过信号确保在主线程执行"""
        self.dragON_signal.emit(client_ip)

    def drag_off(self, client_ip):
        """处理drag_off运动 - 通过信号确保在主线程执行"""
        self.dragOFF_signal.emit(client_ip)

    # 修改相关的调用方法
    def on_audio_trigger(self, text):
        """语音触发任务执行"""
        # 通过 payment_trigger=True 标识这是支付相关的触发
        controller.on_execute(voice_msg=text, payment_trigger=True)

    def handle_coffee_order(self, coffee_name, client_ip):
        """处理咖啡订单"""
        try:
            if coffee_name == '已付款':
                logging.info(f"已付款: {coffee_name} (来自: {client_ip})")
            else:
                logging.info(f"收到咖啡订单: {coffee_name} (来自: {client_ip})")

            # 根据支付功能状态决定执行逻辑
            if self.payment_enabled:
                # 支付功能启用时的逻辑 - 等待支付确认
                if coffee_name == '已付款':
                    # 执行已付款任务
                    self.on_execute(voice_msg="已付款", payment_trigger=True)
                else:
                    # 显示付款二维码
                    self.on_execute(voice_msg="route_qr", payment_trigger=True)
                    logging.info("支付功能已启用，请先完成支付")
            else:
                # 支付功能禁用时的逻辑 - 直接执行任务
                self.on_execute(voice_msg=coffee_name, payment_trigger=False)
                logging.info("支付功能已禁用，直接执行任务")

        except Exception as e:
            logging.info(f"处理咖啡订单失败: {str(e)}")


    #新增在任务执行流程中添加支付状态检查
    def execute_task_based_on_payment(self, task_name, task_text):
        """根据支付状态执行任务"""
        if self.payment_enabled:
            # 支付功能启用 - 需要检查支付状态
            if task_text == "已付款":
                # 直接执行制作任务
                self.function.execute_origin_tasks(task_name, task_text)
            else:
                # 显示付款界面
                self.function.execute_origin_tasks("请付款", task_text)
        else:
            # 支付功能禁用 - 直接执行任务
            self.function.execute_origin_tasks(task_name, task_text)

    def handle_consult_message(self, message_content, client_ip):
        """处理咨询消息"""
        self.consult_signal.emit(message_content, client_ip)

    def handle_log(self, message):
        """处理日志消息"""
        self.log_signal.emit(message)

class FlaskServerQtThread(QThread):  # Flask服务
    """把 FlaskServerThread 封装成 QThread，带信号"""

    coffee_signal = Signal(dict)
    consult_signal = Signal(dict)
    stop_signal = Signal(dict)
    money_signal = Signal(dict)
    result1_signal = Signal(dict)
    result2_signal = Signal(dict)
    result3_signal = Signal(dict)
    checkCapsule_signal = Signal(dict)
    trigger_task_signal = Signal(dict)
    pay3_signal = Signal(dict)
    ddone_signal = Signal(dict)
    # trigger_task_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.server = FlaskServerThread(port=8012)
        self._should_stop = False

        self.server.on_coffee(self.handle_coffee)
        self.server.trigger_task_a(self.trigger_task)
        self.server.on_consult(self.handle_consult)
        self.server.on_stop(self.handle_stop)
        self.server.get_money(self.handle_money)
        self.server.result1(self.handle_result1)
        self.server.result2(self.handle_result2)
        self.server.result3(self.handle_result3)
        self.server.checkCapsule(self.checkCapsule)
        self.server.pay3_a(self.pay3)
        self.server.ddone_a(self.ddone)
    
    def trigger_task(self, data):
        self.trigger_task_signal.emit(data)
        logging.info("✅ 信号已发射")
    def pay3(self, data):
        self.pay3_signal.emit(data)
        logging.info("✅ 流程已结束")

    def ddone(self, data):
        self.ddone_signal.emit(data)

    def handle_coffee(self, data):
        self.coffee_signal.emit(data)

    def handle_consult(self, data):
        print(f"[控制程序] 收到咨询消息: {data}")
        self.consult_signal.emit(data)

    def handle_stop(self, data):
        print(f"[控制程序] 收到停止指令: {data}")
        self.stop_signal.emit(data)

    def handle_result1(self, data):
        # controller.function.send_voice("TASK_RES: getCup-success")   
        self.result1_signal.emit(data)

    def handle_result2(self, data):
        # controller.function.send_voice("TASK_RES: getCapsule-success")   
        self.result2_signal.emit(data)
    def handle_result3(self, data):
        # controller.function.send_voice("TASK_RES: getCapsule-success")   
        self.result3_signal.emit(data)

    def checkCapsule(self, data):
        # controller.function.send_voice("TASK_RES: getCapsule-success")   
        self.checkCapsule_signal.emit(data)

    def handle_money(self, data):
        self.money_signal.emit(data)

    def run(self):
        try:
            self.server.start()
            self.exec()
        except Exception as e:
            print(f"[Flask线程] 运行异常: {e}")
        finally:
            print("[Flask线程] 线程结束")

    def stop_server(self):
        self._should_stop = True
        self.server.stop()  
        self.quit() 
        if not self.wait(3000): 
            print("[Flask线程] 线程未正常结束，强制终止")
            self.terminate()
            self.wait()

class VoiceServiceMonitor():
    def __init__(self):
        current_file = __file__
        current_dir = os.path.dirname(os.path.abspath(current_file))
        parent_dir = os.path.dirname(current_dir)
        json_path = os.path.join(parent_dir, "application/parameters/device.json")
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        
        self.server_ip = data.get("server_ip")
        self.server_port = data.get("server_port")
        self.health_interval = data.get("health_interval")
        
        endpoints = data.get("health_endpoints", {})
        self.health_endpoint = endpoints.get("health", "/health_check")
        self.network_endpoint = endpoints.get("network", "/network_check")
        self.microphone_endpoint = endpoints.get("microphone", "/microphone_check")
        self.audio_endpoint = endpoints.get("audio", "/audio_check")
        # print("11111111111111111111111111:",self.audio_endpoint)

        self.monitor_thread = None
        self.is_monitoring = False
        self.callbacks = []  # 存储回调函数
        self.shared_status = {
            'health': False,
            'network': False,
            'microphone': False,
            'audio': False
        }
        self._last_check_time = 0
        self._cached_alive_status = False
        self._check_lock = threading.Lock() 
        self._is_checking = False
        self._stop_checking = False
    
    def is_service_alive(self) -> bool:
        current_time = time.time()
        with self._check_lock:
            if current_time - self._last_check_time < 5:  # 5秒缓存
                return self._cached_alive_status
        
        # 如果没有正在运行的检测线程，启动一个
        if not hasattr(self, '_check_thread') or not self._check_thread.is_alive():
            self._stop_checking = False
            self._check_thread = threading.Thread(target=self._continuous_background_check, daemon=True)
            self._check_thread.start()
        
        return self._cached_alive_status
    
    def stop_continuous_check(self):
        """停止持续检测"""
        self._stop_checking = True
    
    def _continuous_background_check(self):
        """持续后台检测，直到服务恢复或主动停止"""
        self._is_checking = True
        
        while not self._stop_checking:
            try:
                result = self._quick_ping_check()
                
                with self._check_lock:
                    self._cached_alive_status = result
                    self._last_check_time = time.time()
                
                # 如果服务已恢复，停止检测
                if result:
                    break
                
                # 服务未恢复，等待一段时间后再次检测
                time.sleep(10)  # 每10秒检测一次
                
            except Exception as e:
                print(f"检测过程中发生错误: {e}")
                with self._check_lock:
                    self._cached_alive_status = False
                    self._last_check_time = time.time()
                time.sleep(10)  # 出错后等待10秒再重试
        
        self._is_checking = False
    
    def _quick_ping_check(self) -> bool:
        try:
            print("正在检测服务状态...")
            response = requests.get(
                f"http://{self.server_ip}:{self.server_port}",
                timeout=1 
            )
            if response.status_code == 200:
                print("服务检测正常")
                return True
            else:
                print(f"服务返回异常状态码: {response.status_code}")
                # # 触发重启逻辑
                # value = controller.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json",["w1_ui"])
                # if value:
                #     self._restart_service()
                return False
                
        except Exception as e:
            print(f"服务检测失败: {e}")
            # 触发重启逻辑
            value = controller.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json",["w1_ui"])
            if value:
                self._restart_service()
            
            return False
    
    def _restart_service(self):
        """重启服务的逻辑"""
        try:
            print("开始重启服务...")
            time.sleep(1)
            controller.kill_voiceServer()
            time.sleep(3)
            # controller.on_tool_button_action("voice_start")
            controller.B_voice_service_combo.setCurrentText("启用语音服务")
            a = controller.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json",["voice_service_status"])
            time.sleep(7)
            if a:
                time.sleep(6)
                controller.on_tool_button_action("voice_connection")
                time.sleep(1)
            print("服务重启完成")
        except Exception as e:
            print(f"重启服务过程中发生错误: {e}")

        

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def start_monitoring(self):
        if self.is_monitoring:
            print("监控已在运行中")
            return
        
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("语音服务监控已启动")

    def stop_monitoring(self):
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        print("语音服务监控已停止")

    def _monitor_loop(self):
        last_status = {}
        
        while self.is_monitoring:
            try:
                controller.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "health_endpoint", self._check_endpoint(self.health_endpoint))
                controller.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "network_endpoint", self._check_endpoint(self.network_endpoint))
                controller.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "microphone_endpoint", self._check_endpoint(self.microphone_endpoint))
                controller.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "audio_endpoint", self._check_endpoint(self.audio_endpoint))
                current_status = {
                    'health': self._check_endpoint(self.health_endpoint),
                    'network': self._check_endpoint(self.network_endpoint),
                    'microphone': self._check_endpoint(self.microphone_endpoint),
                    'audio': self._check_endpoint(self.audio_endpoint),
                    'timestamp': time.time()
                }
                self.shared_status.update(current_status)
                if current_status != last_status:
                    self._notify_callbacks(current_status)
                    last_status = current_status
                time.sleep(self.health_interval or 5)
                
            except Exception as e:
                print(f"监控循环异常: {e}")
                time.sleep(5)

    def _notify_callbacks(self, status):
        for callback in self.callbacks:
            try:
                callback(status)
            except Exception as e:
                print(f"回调函数执行失败: {e}")

    def get_health_status(self):
        return self._check_endpoint(self.health_endpoint)

    def get_network_status(self):
        return self._check_endpoint(self.network_endpoint)

    def get_microphone_status(self):
        return self._check_endpoint(self.microphone_endpoint)

    def get_audio_status(self):
        return self._check_endpoint(self.audio_endpoint)

    def _check_endpoint(self, endpoint):
        try:
            url = f"http://{self.server_ip}:{self.server_port}/{endpoint}"
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                data = response.json()
                status = data.get("status", False)
                print(f"{endpoint}: {status}")
                return status
            print(f"{endpoint}: 请求失败，状态码 {response.status_code}\n")
            return False
        except requests.exceptions.RequestException as e:
            # print(f"请求 {endpoint} 异常: {e}")
            print(f"请求 {endpoint} 异常")
            return False

class ModelInferenceManager: # 专有线程池执行任务
    def __init__(self, max_workers=1):
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.task_queue = queue.Queue()
        self.is_running = True
        
    def submit_inference(self, model_func, *args, **kwargs):
        """提交推理任务"""
        future = self.executor.submit(model_func, *args, **kwargs)
        return future
        
    def shutdown(self):
        """关闭线程池"""
        self.is_running = False
        self.executor.shutdown(wait=True)

class MovePartThread(QThread):  # UI中单点运动线程
    """运动执行线程"""

    finished_signal = Signal(bool, str)  # 成功标志, 消息

    def __init__(
        self, w1_controller, move_type, part=None, text_field=None, speed_ratio_key=None
    ):
        super().__init__()
        self.w1_controller = w1_controller
        self.move_type = move_type
        self.part = part
        self.text_field = text_field
        self.speed_ratio_key = speed_ratio_key

    def run(self):
        """线程主执行函数"""
        try:
            if self.move_type == "rotate":
                # 旋转关节运动
                status = np.round(
                    np.radians(ast.literal_eval(self.text_field.text())), 4
                ).tolist()
                move_point = {
                    self.part: [status],
                    f"{self.part}_speed_ratio": [
                        self.w1_controller.device_cfg["ui_param"][self.speed_ratio_key]
                    ],
                }
                success = self.w1_controller.w1.w1_manager.move_linear_full_body(
                    waypoints_dict=move_point,
                    speed_ratio=self.w1_controller.device_cfg["ui_param"][
                        self.speed_ratio_key
                    ],
                )
                message = f"{self.part}运动完成" if success else f"{self.part}运动失败"
                self.finished_signal.emit(success, message)

            elif self.move_type == "ee":
                # 末端执行器运动
                hand_state = ast.literal_eval(self.text_field.text())
                self.w1_controller.w1.w1_manager.set_ee_qpos(self.part, hand_state)
                message = f"{self.part}运动到状态: {hand_state}"
                self.finished_signal.emit(True, message)

            elif self.move_type == "linear_right":
                # 右臂线性运动
                arm_l_pose = self.w1_controller.w1.get_current_pose(
                    "left_arm", is_local=self.w1_controller.is_local
                )[0]
                arm_r_pose = ast.literal_eval(self.text_field.text())
                move_point = {"left_arm": [arm_l_pose], "right_arm": [arm_r_pose]}

                if self.w1_controller.log_msg["move_dict"][1] == 1:
                    logging.info(
                        self.w1_controller.log_msg["move_dict"][0].format(
                            "N",
                            self.w1_controller.format_json_compact_inner_lists(
                                move_point
                            ),
                        )
                    )

                success = self.w1_controller.w1.w1_manager.move_linear_full_body(
                    waypoints_dict=move_point,
                    speed_ratio=self.w1_controller.device_cfg["ui_param"][
                        "movel_speed"
                    ],
                    is_local=self.w1_controller.is_local,
                )
                message = "右臂运动完成" if success else "右臂运动失败"
                self.finished_signal.emit(success, message)

            elif self.move_type == "linear_left":
                # 左臂线性运动
                arm_r_pose = self.w1_controller.w1.get_current_pose(
                    "right_arm", is_local=self.w1_controller.is_local
                )[0]
                arm_l_pose = ast.literal_eval(self.text_field.text())
                move_point = {"left_arm": [arm_l_pose], "right_arm": [arm_r_pose]}

                success = self.w1_controller.w1.w1_manager.move_linear_full_body(
                    waypoints_dict=move_point,
                    speed_ratio=self.w1_controller.device_cfg["ui_param"][
                        "movel_speed"
                    ],
                    is_local=self.w1_controller.is_local,
                )
                message = "左臂运动完成" if success else "左臂运动失败"
                self.finished_signal.emit(success, message)

            else:
                self.finished_signal.emit(False, f"未知的运动类型: {self.move_type}")

        except Exception as e:
            error_msg = f"{self.move_type.capitalize()}运动出错: {str(e)}"
            self.finished_signal.emit(False, error_msg)

class ControllerSignals(QObject):
    trigger_face_track_sig = Signal()

class W1Controller(W1Displayui):  # 主流程

    def __init__(self, main_window=None):
        super().__init__()
        self.last_checked_battery = -1 
        self.is_force_working = False 
        self.is_charging_cycle = False   # 初始化为 False，代表开机时没在充电周期里
        self.pending_auto_charging = False
        # 电量回调、任务线程可能并发修改回充状态，统一由此锁保护。
        self._chassis_state_lock = threading.RLock()
        # 新任务在真正创建线程前可能需要等待诊断和执行退桩；使用请求序号让
        # 旧任务的 Policy 恢复等待立即失效，避免退桩期间错误恢复 Policy。
        self._task_request_serial = 0
        self._charging_state_condition = threading.Condition(
            self._chassis_state_lock)
        self._auto_charging_requesting = False
        self._auto_charging_worker = None
        self._last_auto_charging_attempt_time = 0.0
        self._full_charge_undocking = False
        self._last_full_charge_undock_attempt_time = 0.0
        self._auto_charging_retry_interval = 30.0
        self._full_charge_undock_retry_interval = 30.0
        self.is_battery_charging = False
        self.is_charger_connected = False
        self.is_charging_current_detected = False
        self._has_charging_state_observation = False
        self._last_charging_state_observed_time = None
        self._charging_state_freshness_timeout = 5.0
        self._charger_disconnected_stable_count = 0
        self._charger_disconnected_stable_threshold = 5
        self._charging_cycle_confirmed_time = None
        self.signals = ControllerSignals()
        # 初始化所有配置变量
        self.config = None
        self.log_msg = None
        self.device_cfg = None
        self.calibration_cfg = None
        self.parameter_cfg = None
        self.parameter_path = None
        self.task_cfg = None
        self.vision_app = None
        self.w1 = None
        self.function = None
        self._editor_ref = None
        self.status_thread = None
        self.image_analyzer_window = None
        self.vision_detector_window = None
        self.ttt = 0
        self.tcp_monitoring_enabled = True
        self.http_server_thread = None
        self.auto_config_monitor = None
        self.detect_ros_process = None
        self.simbyviser_process = None
        self.recorder_process = None  # 录制  dexe_recorder 守护进程
        self.http_server_thread = None  # HTTP语音服务线程
        self.http_server_running = False
        self.payment_manager = None
        self.elec_voice = True
        self.robot_app = None
        # self.handler = orderHandler()
        self.main_window = main_window
        self.global_state = (
            "idle"  # 状态: idle(待机), running(运行中), stopping(用户任务终止,gohome)
        )
        self.client_socket = None
        self.connection_result = None  # OP&elec
        self.vision_param = None
        self.vision_logger = None
        self.voice_task_running = None # 判断任务状态
        self.picklight_vision = None
        self.is_local = None
        self.is_sim = False
        self.move_thread = None  # 当前运动线程
        self.active_move_threads = []  # 活动运动线程列表
        self.auto_voice = None
        self.auto_voice_new = None
        self.is_payment = False
        self.judgeCapsule = True
        self.judgeCup = True
        self.thread = None
        # # 新增：初始化支付功能状态
        # self.payment_enabled = False
        #self.update_payment_button_style(True)  # 直接调用UI方法设置样式
        self.voice_service_new_window = None
        self.model_manager = ModelInferenceManager(max_workers=1)  
        # 1112修改 胶囊计数
        self.capsule_timer = None
        if main_window is not None:
            self.setup_ui(main_window)
            self._connect_signals()
            # 1112修改 胶囊计数 添加胶囊计数器初始化
            self.setup_capsule_counter()
        # self.mainwindow = MainWindow()
        # self.voice_monitor = VoiceServiceMonitor()
        self.voice_monitor = None

        # self.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "w1_ui", "true")
        self.setup_ui_update_timer()

        # 录制 状态灯刷新（每2秒读 auto_config.recorder_status）
        self.recorder_refresh_timer = QTimer()
        self.recorder_refresh_timer.timeout.connect(self._refresh_recorder_light)
        self.recorder_refresh_timer.start(1500)
        #---------

        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self._write_heartbeat)
        self.heartbeat_timer.start(5000)

        # 回充失败后的重试不能依赖电量百分比再次变化。
        self.auto_charging_retry_timer = QTimer()
        self.auto_charging_retry_timer.timeout.connect(
            self._retry_pending_auto_charging)
        self.auto_charging_retry_timer.start(1000)

        self.policy_control_thread = None # 初始化 Policy 线程变量
        self.origin_cfg = None
       
    #录制
    def _refresh_recorder_light(self):
        """根据 auto_config 中的 recorder_status 更新录制指示灯"""
        try:
            if not isinstance(getattr(self, "device_cfg", None), dict):
                return
            status_data = self._read_status_file()
            recording = status_data.get("recorder_status", False)
            if hasattr(self, "L_recorder_status"):
                if recording:
                    self.update_light(self.L_recorder_status, True)
                    self.L_recorder_status.setText("录制: ON")
                else:
                    self.update_light(self.L_recorder_status, False)
                    self.L_recorder_status.setText("录制: OFF")
        except Exception:
            pass
    # Policy相关封装方法
    def _get_policy_bridge(self):
        """方案C: 若 basic_function 未接线 policy_bridge，则懒实例化后返回"""
        func = getattr(self, 'function', None)
        if func is None:
            return None
        if getattr(func, 'policy_bridge', None) is None:
            try:
                from application.policy_function import PolicyBridge
                func.policy_bridge = PolicyBridge(func)
            except Exception as e:
                logging.error(f"PolicyBridge 懒实例化失败: {e}")
                return None
        return func.policy_bridge

    def _on_policy_control_finished(self, success, message):
        """Policy 启动回调"""
        logging.info(message)
        if success:
            if hasattr(self, "L_policy_status"):
                self.update_light(self.L_policy_status, True)
                self.L_policy_status.setText("Policy: ON")
        else:
            if hasattr(self, "L_policy_status"):
                self.update_light(self.L_policy_status, False)
                self.L_policy_status.setText("Policy: OFF")
            # 启动失败：重新启用按钮，避免防呆禁用后卡死
            if hasattr(self, "B_resume_policy"):
                self.B_resume_policy.setEnabled(True)

    def on_asset_download(self):
        """处理资产下载同步，输出直接打印到终端"""
        config_path = self.T_asset_config.text().strip()
        # ... (路径检查逻辑)
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sync_script = os.path.join(project_root, "tools","asset_manager" ,"sync_assets.py")
        def run_sync():
            try:
                # 使用 -u 确保 python 实时无缓冲输出到终端
                process = subprocess.Popen(
                    [sys.executable, "-u", sync_script, "-c", config_path],
                    cwd=project_root
                )
                process.wait()
                if process.returncode == 0:
                    print("✅ 资产同步完成")
                else:
                    print(f"❌ 资产同步失败，退出码: {process.returncode}")
            except Exception as e:
                print(f"❌ 执行同步脚本出错: {str(e)}")

        threading.Thread(target=run_sync, daemon=True).start()
    # ------------------------------------------------------------ 多配置 -----------------------------------------------------------
    def get_isolated_task_config(self, module_name):
        """
        智能路由合成：Base通用配置 + 自动搜索的专属Workspace点位
        """
        import copy
        import glob
        # 1. 深度拷贝基础配置，确保内存隔离
        task_cfg = copy.deepcopy(self.parameter_cfg)
        
        # 2. 定位 workspaces 文件夹 (基于 parameter.json 所在目录)
        params_dir = os.path.dirname(self.parameter_path)
        workspaces_dir = os.path.join(params_dir, "workspaces")

        found_special = False
        # 3. 遍历所有以 workspace_ 开头的配置文件
        search_pattern = os.path.join(workspaces_dir, "workspace_*.json")
        
        for ws_file in glob.glob(search_pattern):
            try:
                with open(ws_file, 'r', encoding='utf-8') as f:
                    ws_content = json.load(f)
                    # 检查该文件是否包含当前任务/子任务的键值
                    if "workspace" in ws_content and module_name in ws_content["workspace"]:
                        # 将专项点位合并到基础配置副本中
                        task_cfg["workspace"].update(ws_content["workspace"])
                        # logging.info(f"✅ 路由成功：从 [{os.path.basename(ws_file)}] 提取点位")
                        found_special = True
                        break 
            except Exception as e:
                continue

        if not found_special:
            logging.info(f"ℹ️ 任务 [{module_name}] 未匹配专项配置，仅使用基础点位执行")
            task_cfg = self.origin_cfg
            
        return task_cfg

    # ------------------------------------------------------------ 支付模块 ------------------------------------------------------------
    def start_payment_service(self):
        """启动支付通知监听服务"""
        try:
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            if project_root not in sys.path:
                sys.path.insert(0, project_root)
            from payment.payment_manager import PaymentManager



            if self.payment_manager is not None:
                logging.info("支付服务已在运行")
                return

            # 从配置读取
            payment_config = self.device_cfg["PAYMENT_SERVER"]
            self.payment_manager = PaymentManager(payment_config)
            self.payment_manager.order_received.connect(lambda msg,order:self.Placeorder_pay(order))

            self.last_order_count = 0
            self.order_change_count = 0
            # 启动监听（在新线程中运行异步方法）
            import threading
            import asyncio
            def run_payment():
                """在新线程中运行异步函数"""
                try:
                    # 创建新的事件循环
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    loop.run_until_complete(
                        self.payment_manager.receive_payment_notifications()
                    )
                except Exception as e:
                    logging.error(f"支付监听错误: {str(e)}")
                finally:
                    if loop and not loop.is_closed():
                        loop.close()

            self.payment_thread = threading.Thread(target=run_payment, daemon=True)
            self.payment_thread.start()

            logging.info(
                f"支付服务已启动 - 门店: {payment_config['STORE_SN']}"
            )
            logging.info(
                f"待处理订单: {self.payment_manager.order_cache.__len__()}"
            )
            # 启动订单缓存长度监控线程：当长度增加时调用回调方法
            try:
                # 初始化监控标记与初始长度
                self._order_monitor_stop_event = threading.Event()
                try:
                    self._last_order_cache_len = (
                        self.payment_manager.order_cache.__len__()
                        if self.payment_manager and hasattr(self.payment_manager, "order_cache")
                        else 0
                    )
                except Exception:
                    self._last_order_cache_len = 0

                def _order_monitor():
                    # 从配置读取监控间隔（秒），默认为 1 秒                 
                    interval = 1

                    while not getattr(self, "_order_monitor_stop_event", threading.Event()).is_set():
                        try:
                            if self.payment_manager and hasattr(self.payment_manager, "order_cache"):
                                try:
                                    curr_len = self.payment_manager.order_cache.__len__()
                                except Exception:
                                    curr_len = 0

                                prev_len = getattr(self, "_last_order_cache_len", 0)
                                if curr_len > prev_len:
                                    # 更新记录长度并调用回调（用户实现）
                                    self._last_order_cache_len = curr_len
                                    try:
                                        self.execute_voice_command(self.parameter_cfg.get("payment_new_order_callback", ""))
                                    except Exception as e:
                                        logging.error(
                                            f"订单增加回调错误: {e}"
                                        )
                                else:
                                    # 同步最新长度
                                    self._last_order_cache_len = curr_len
                        except Exception as e:
                            logging.error(f"订单监控异常: {e}")
                        time.sleep(interval)

                self._order_monitor_thread = threading.Thread(target=_order_monitor, daemon=True)
                self._order_monitor_thread.start()
            except Exception as e:
                logging.error(f"启动订单监控失败: {e}")


            # # 启动一个服务用来检测机器人空闲状态和语音服务空闲状态，并且把活跃消息发送给语音服务
            # def idle_heartbeat():
            #     """每 5 秒检测：有订单且机器人和语言服务都空闲 → 发送活跃包"""
            #     HEART_BEAT_INTERVAL = 5
            #     while True:
            #         try:
            #             speech_busy = True # 默认在忙
            #             # ---- 1. 订单是否全空 ----
            #             no_order = (
            #                 self.payment_manager.order_cache.is_empty() and
            #                 self.payment_manager.order_cache.current_order is None
            #             )

            #             # ---- 2. 机器人和语音状态是否全部为可demo状态 ---- 
            #             if self._read_status_file().get("status", "running") != "running":
            #                 speech_busy = False


            #             # ---- 3. 发消息给语言服务处理数据 ----
            #             if not no_order and not speech_busy:
            #                 if hasattr(self.async_thread, 'send_msg'):
            #                     current_order = self.payment_manager.order_cache.current_order
            #                     msg = "PAYMENT_INFO: " + json.dumps(current_order.to_dict())

            #                     self.async_thread.send_msg(msg)
            #                     # 发送消息后删除当前活跃订单并且把下一个订单设为活跃
            #                     self.payment_manager.order_cache.remove_order(current_order.order_sn)
            #                     self.payment_manager.order_cache.set_next_as_current()
            #         except Exception as e:
            #             logging.error(f"空闲心跳异常: {e}")
            #         time.sleep(HEART_BEAT_INTERVAL)

            # self.idle_heartbeat_thread = threading.Thread(target=idle_heartbeat, daemon=True)
            # self.idle_heartbeat_thread.start()


        except Exception as e:
            logging.info(f"支付服务启动失败: {str(e)}")

    def Placeorder_pay(self,msg):

        try:       
            obtainPaymen=order_detail.obtainPaymen()
            data=obtainPaymen.obtain_order(msg)
            data = json.loads(data)
            self.task_queue(data)       
            self.on_single_task()   


        except Exception as e:
            print(f"运行工程失败: {e}")
            


    def task_queue(self,data):
        try:
            # 读取队列
            with open("/home/dexforce/workspace/dexe_application/application/parameters/Payment_task_queue.json", "r", encoding="utf-8") as f:
                task_queue = json.load(f)
            Total_Amount=0
        
            for i in data["goods"]:
               print(i["name"]) 
               order_name=i["name"]
               # 订单总价格
               Total_Amount=Total_Amount+i["totalAmount"]
               logging.info(Total_Amount)
               # 读取支付订单，将其添加进队列 
               logging.info(int(i["quantity"]))
               for j in range(int(i["quantity"])):
                    if order_name in task_queue["mapping"]:
                        product=task_queue["mapping"][f"{order_name}"]
                        task_queue["task_queue"].append(product)
                    else:
                        logging.info(f"{order_name}商品未映射，请联系工作人员！")

                

        #判断是否抽奖
            if(Total_Amount>=task_queue["lottery"]["amount"]):
                task=task_queue["lottery"]["task"]
                for k in task:
                    task_queue["task_queue"].append(k)
      
            logging.info(task_queue)  

            with open("/home/dexforce/workspace/dexe_application/application/parameters/Payment_task_queue.json", 'w', encoding='utf-8') as f:
                 json.dump(task_queue, f, ensure_ascii=False, indent=2)
                 logging.info("写入成功")     
                          
        except Exception as e:
            logging.error(f"运行工程失败: {e}")
            
        

    def on_single_task(self):
        try:
            time.sleep(0.3)
            with open("/home/dexforce/workspace/dexe_application/application/parameters/Payment_task_queue.json", "r", encoding="utf-8") as f:
                task_queue = json.load(f)
            logging.info(f'{task_queue["task_queue"]}  长度{len(task_queue["task_queue"])}')
            # if len(task_queue["task_queue"])>0:
            #     task_queue["task_queue"].append("招手")
            self.config['parameter_cfg']["workspace"]["执行订单任务"]["continuous"]=task_queue["task_queue"]
            self.function.set_config(self.config)
            # logging.info(f'开始执行订单任务.........................{task_queue["task_queue"]}...')
            while (len(task_queue["task_queue"])!=0):
                current_status = self._read_status_file()
                # logging.info(f'执行订单任务，查询状态: {current_status}')
                if(current_status["status"] == "idle"):
                    # logging.info(f'开始执行订单任务，查询状态: {current_status}')
                    self.on_execute(module_name="执行订单任务",task_list=task_queue["task_queue"]) 
                    # logging.info(f"已下发订单任务")
                    task_queue["task_queue"]=[]
                    with open("/home/dexforce/workspace/dexe_application/application/parameters/Payment_task_queue.json", 'w', encoding='utf-8') as f:
                        json.dump(task_queue, f, ensure_ascii=False, indent=2)
                        # logging.info("更新写入成功") 
                time.sleep(0.2)  
            logging.info(f"退出订单任务线程")     


        except Exception as e:
            logging.info(f"运行工程失败: {e}")

    def stop_payment_service(self):
        """停止支付服务"""
        try:
            # 停止订单监控线程（如果存在）
            try:
                if hasattr(self, '_order_monitor_stop_event') and self._order_monitor_stop_event:
                    self._order_monitor_stop_event.set()
            except Exception:
                pass
            try:
                if hasattr(self, '_order_monitor_thread') and self._order_monitor_thread:
                    try:
                        self._order_monitor_thread.join(timeout=1)
                    except Exception:
                        pass
            except Exception:
                pass

            # 清理 payment_manager 引用
            self.payment_manager = None
            # logging.info("支付服务已停止")
            logging.info(f"支付服务已停止")
        except Exception as e:
            logging.info(f"支付服务停止失败: {str(e)}")
            # logging.info(f"支付服务停止失败: {str(e)}")


    def add_virtual_order(self, flavor = None):
        """添加一个虚拟订单"""
        from payment.payment_manager import OrderInfo
        dummy = OrderInfo(
            msg_id="dummy_msg",
            store_sn="VIRTUAL_STORE",
            order_sn = str(int(time.time() * 1_000_000)),
            order_status=True,
            timestamp=int(time.time()),
            business_data={"item": "virtual-coffee", "price": 0.01},
            flavor = flavor
        )
        controller.payment_manager.order_cache.add_order(dummy)
        if controller.payment_manager.order_cache.current_order is None:
            controller.payment_manager.order_cache.current_order = dummy

    def toggle_flask_server(self):
        if self.flask_qt_thread and self.flask_qt_thread.isRunning():
            self.ip_timer.stop()
            self.flask_qt_thread.stop_server()
            self.flask_qt_thread.quit()
            self.flask_qt_thread.wait()
            self.flask_qt_thread = None
            self.B_http.setText("网页服务已关闭")
            self.B_http.setStyleSheet(self._get_button_style("gray"))

        else:
            self.setup_flask_server()
            self.B_http.setText("网页服务已打开")
            self.B_http.setStyleSheet(self._get_button_style("green"))
            

    def setup_flask_server(self):
        self.flask_qt_thread = FlaskServerQtThread()
        self.flask_qt_thread.trigger_task_signal.connect(self.trigger_task_slot, Qt.ConnectionType.DirectConnection)
        self.flask_qt_thread.pay3_signal.connect(self.pay3_slot, Qt.ConnectionType.DirectConnection)
        self.flask_qt_thread.ddone_signal.connect(self.ddone_slot, Qt.ConnectionType.DirectConnection)
        self.flask_qt_thread.coffee_signal.connect(self.handle_coffee_slot, Qt.ConnectionType.QueuedConnection)
        self.flask_qt_thread.money_signal.connect(self.handle_money_slot, Qt.ConnectionType.QueuedConnection)
        self.flask_qt_thread.result1_signal.connect(self.result1_slot, Qt.ConnectionType.QueuedConnection)
        self.flask_qt_thread.result2_signal.connect(self.result2_slot, Qt.ConnectionType.QueuedConnection)
        self.flask_qt_thread.result3_signal.connect(self.result3_slot, Qt.ConnectionType.QueuedConnection)
        self.flask_qt_thread.checkCapsule_signal.connect(self.checkCapsule_slot, Qt.ConnectionType.QueuedConnection)
        self.flask_qt_thread.start()
        
        self.current_ip = None
        if not hasattr(self, "ip_timer"):
            self.ip_timer = QTimer()
            self.ip_timer.timeout.connect(self.check_ip_change)
        self.ip_timer.start(1000) 

    def check_ip_change(self):
        new_ip = self.return_ip()
        if new_ip != self.current_ip:
            self.current_ip = new_ip
            ipa = f"网页服务IP：{new_ip}:8012"
            self.K4.clear()
            self.K4.append(ipa)
            print(f"🔄 IP更新为: {new_ip}")

    def return_ip(self):
        try:
            iface = "LAN1" 
            addrs = netifaces.ifaddresses(iface)
            ip_info = addrs.get(netifaces.AF_INET)
            if ip_info:
                return ip_info[0]['addr']
            return "未连接WiFi"
        except Exception as e:
            return f"获取WiFi IP失败: {e}"


    def handle_money_slot(self, data):
        logging.info(f"收到收钱吧切换信号")
        self.toggle_payment_function()
        switch_state = data.get("switchON")
        logging.info(f"收到收钱吧切换信号: {switch_state}")
        self.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "getMoney", switch_state)

    def read_yml_config(self, file_path, config_keys):
        """
        读取YAML配置文件中的特定配置项
        
        参数:
            file_path: YAML文件路径
            config_keys: 配置键列表，如["PAYMENT_SERVER", "ENABLE"]
        
        返回:
            配置项的值，如果不存在则返回None
        """
        try:
            # 检查文件是否存在
            if not os.path.isfile(file_path):
                raise FileNotFoundError(f"配置文件不存在: {file_path}")
            
            # 读取YAML文件
            with open(file_path, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file)
            
            # 逐级获取配置值
            current_level = config_data
            for key in config_keys:
                if isinstance(current_level, dict) and key in current_level:
                    current_level = current_level[key]
                else:
                    logging.info(f"警告: 配置项 '{key}' 不存在")
                    return None
            
            return current_level
            
        except FileNotFoundError as e:
            logging.error(f"文件错误: {e}")
            return None
        except yaml.YAMLError as e:
            logging.error(f"YAML解析错误: {e}")
            return None
        except Exception as e:
            logging.error(f"未知错误: {e}")
            return None
   
    
    def trigger_task_slot(self, data): 
        task = data.get("task") 
        card = data.get("card") 
        temperature = data.get("temperature") 
        is_auto = data.get("is_auto", False)
#####################重复订单号不执行
        # order_sn = data.get("orderSn")

        # if order_sn:
        #     now = time.time()
        #     last_order_sn = getattr(self, "_last_trigger_order_sn", None)
        #     last_trigger_time = getattr(self, "_last_trigger_order_time", 0)

        #     if last_order_sn == order_sn and now - last_trigger_time < 300:
        #         logging.warning(f"重复订单触发已忽略: orderSn={order_sn}, task={task}")
        #         return

        #     self._last_trigger_order_sn = order_sn
        #     self._last_trigger_order_time = now
#####################重复订单号不执行

        if str(card) == "1": 
            task = temperature + task 

        if is_auto:
            t1=time.time()
            while True: 
                tasks1 = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", ["tasks"]) 
                t2=time.time()
                t3=t2-t1
                if t3>10:
                    logging.error("机器人running状态超时")
                    self._write_status_file("status", "idle")
                    time.sleep(0.2)
                    break
                valuepp = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", ["status"]) 
                if valuepp == "running" and not tasks1["task_index"]==0: 
                    time.sleep(1) 
                    logging.warning("订单系统触发：机器人仍在运动，后台静默等待中...") 
                else: 
                    time.sleep(0.5) 
                    # logging.warning("机器人status异常：置status为idle") 
                    break 
        else:
            valuepp = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", ["status"]) 
            tasks = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", ["tasks"]) 
            if valuepp == "running" and not tasks["task_index"]==0: 
                logging.warning(f"网页点单已触发：机器人正忙，订单 [{task}] 已入队，接口直接返回。") 
                return  
            else:logging.warning("机器人status异常：置status为idle") 
        self._write_status_file("status", "idle")
        time.sleep(0.2)
        self.on_execute(voice_msg=f"{task}", module_name=f"{task}") 
        logging.info(f"✅✅✅已触发机器制作:{task}")
    def clear_order_json(self,file_path):
        try:
            # 以写入模式 ('w') 打开文件，这会自动覆盖原内容
            with open(file_path, 'w', encoding='utf-8') as f:
                # 写入一个空列表，这样你的 check_order_type 函数会返回 "文件数据为空"
                json.dump([], f, ensure_ascii=False, indent=4)
            print(f"成功清空文件: {file_path}")
        except Exception as e:
            print(f"清空文件时发生错误: {e}")
    def ddone_slot(self, data):
            logging.info(f"###data:{data}")
            name=data["name"]
            if isinstance(data, dict) and 'sequence' in data:
                sequence_val = data['sequence']
                formatted_sequence = f"{int(sequence_val):04d}" 
                self.voice_params({"order":formatted_sequence})
                self.execute_voice_command({"mode": "auto", "text": "订单号为：{order}的商品已经准备好了，请及时取走", "stop": False})
                logging.info(f"语音已触发， sequence 值为: {formatted_sequence}")
            logging.info(f"✅✅✅订单【{name}】流程完成")
            # current_order_path = '/home/dexforce/workspace/dexe_application/qt/ordermanagement/currentOrder.json'
            # self.clear_order_json(current_order_path)
       
    def pay3_slot(self, data):
            logging.info("订单已结束")


    def handle_coffee_slot(self, data):
        payment = False
        a = self.read_yml_config("/home/dexforce/workspace/dexe_application/qt/dexe_agent_config.yml",["PAYMENT_SERVER","ENABLE"])
        if a:
            if payment == True:
                
                self.on_execute(voice_msg="已付款")
            else:
                self.on_execute(voice_msg="route_qr")
                logging.info("支付功能已启用，请先完成支付")
        else:
            self.on_execute(voice_msg=data.get("name"), module_name = "泡咖啡")
            logging.info("支付功能已禁用，直接执行任务")

    def result1_slot(self, data):
        self.judgeCup = True

    def result2_slot(self, data):
        self.judgeCapsule = True

    def result3_slot(self, data):
        pass
   
    def checkCapsule_slot(self, data):
        status = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json",["status"])
        numCapsule = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json",["capsule_coffeemachine_num"])
        judgeCup = self.read_json_value("/home/dexforce/workspace/dexe_application/qt/roi_area.json",["fixed_roi","cup_area","classes","cup"])
        if ((numCapsule >20) and (status == "idle")and(self.judgeCapsule)):
            self.judgeCapsule = False
            
        elif ((judgeCup == False) and (status == "idle")and(self.judgeCup)):
            self.judgeCup = False

    def _write_heartbeat(self):
        try:
            with open("/tmp/w1_controller_heartbeat", "w") as f:
                f.write(str(int(time.time())))
        except Exception:
            pass 

    def setup_ui_update_timer(self):
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui_from_shared_vars)
        self.ui_timer.start(1000)  
    
    def update_ui_from_shared_vars(self):
        if self.voice_monitor and self.voice_monitor.is_monitoring:
            status = self.voice_monitor.shared_status
            w1ui.update_light(self.voice1, status['health'])
            w1ui.update_light(self.voice2, status['network'])
            w1ui.update_light(self.voice3, status['microphone'])
            w1ui.update_light(self.voice4, status['audio'])

    def start_voice_monitoring(self):
        if self.voice_monitor is None:
            self.voice_monitor = VoiceServiceMonitor()
            self.voice_monitor.start_monitoring()
            self.restartVoice()
            print("语音监控已启动")

    def restartVoice(self):
        if self.voice_monitor.is_service_alive():
            print("语音通信正常")
        else:
            print("语音通信异常")
        
    def voice_params(self,  params: dict):
        try:
            from contextlib import nullcontext
            ctrl = self.voice_service_new_window.voice_controller
            if not (ctrl and ctrl.audio_processor and ctrl.audio_processor.initialized):
                return {}

            return ctrl.execute_voice_params(params)
        except Exception as e:
            return {}

    def execute_voice_command(self, param:dict) -> bool:
        """执行语音命令"""
        try:
            if  (self.voice_service_new_window.voice_controller and 
                self.voice_service_new_window.voice_controller.audio_processor and 
                self.voice_service_new_window.voice_controller.audio_processor.initialized):

                return self.voice_service_new_window.voice_controller.execute_voice_command(param)
            else:
                return False
        except Exception as e:
            return False

    def voice_enable_set(self, param: dict):
        """执行语音命令，修改并返回使能状态"""
        try:
            if (self.voice_service_new_window.voice_controller and 
                self.voice_service_new_window.voice_controller.audio_processor and 
                self.voice_service_new_window.voice_controller.audio_processor.initialized):
                self.voice_service_new_window.voice_controller.enable = param.get("enable", True)

                logging.info(f"语音服务使能状态已设置为: {self.voice_service_new_window.voice_controller.enable}")

        except Exception as e:
            logging.error(f"设置语音使能失败: {e}")

    def wake_up_state(self, state:bool):
        """更该wake的状态---True为唤醒---False为"""
        try:
            if  (self.voice_service_new_window.voice_controller and 
                self.voice_service_new_window.voice_controller.audio_processor and 
                self.voice_service_new_window.voice_controller.audio_processor.initialized):

                self.voice_service_new_window.voice_controller.audio_processor._is_wake_up = state
        except Exception as e:
            logging.error(f"唤醒值转换失败: {str(e)}")
            return False

    def try_suspend_policy_service(self):
        """挂起 Policy 服务 → 委托 PolicyBridge"""
        try:
            bridge = self._get_policy_bridge()
            if bridge and bridge.is_active():
                logging.info("检测到Policy后台运行，自动挂起...")
                self.policy_suspended_by_task = True
                bridge.suspend()
                if hasattr(self, "L_policy_status"):
                    self.update_light(self.L_policy_status, False)
                    self.L_policy_status.setText("Policy: Suspend")
                # 事件驱动解锁：Policy 已挂起(不活跃)，重新启用恢复Policy按钮
                if hasattr(self, 'B_resume_policy'):
                    self.B_resume_policy.setEnabled(True)
                return True
            elif getattr(self, 'policy_suspended_by_task', False):
                # 如果 Policy 已经被上一个被打断的任务挂起了，当前任务也需要返回 True，在结束时负责恢复它
                return True
        except Exception as e:
            logging.warning(f"挂起Policy时出现竞态(可忽略): {e}")
        return False

    def try_resume_policy_service(self, force=False):
        """恢复 Policy 服务 → 委托 PolicyBridge"""
        bridge = self._get_policy_bridge()
        if bridge is not None:
            can_resume = force or not getattr(self.function, "stop_squence", False)
            if not can_resume:
                return
            logging.info("【系统】插队任务结束，恢复Policy运行...")
            self.policy_suspended_by_task = False
            try:
                bridge.resume_and_run_motion(force=force)
                if hasattr(self, "L_policy_status"):
                    self.update_light(self.L_policy_status, True)
                    self.L_policy_status.setText("Policy: ON")
                # 事件驱动：Policy 已活跃，禁用恢复Policy按钮防止重复恢复
                if hasattr(self, 'B_resume_policy'):
                    self.B_resume_policy.setEnabled(False)
            except Exception as e:
                logging.info(f"Policy恢复失败: {str(e)}")

    def try_resume_policy_for_task(self, task_request_serial: int) -> bool:
        """仅允许当前最新任务恢复 Policy，避免与新任务退桩发生竞态。"""
        with self._chassis_state_lock:
            if self._task_request_serial != task_request_serial:
                logging.info(
                    "[Policy恢复] 已有更新任务请求，忽略旧任务恢复："
                    f"旧序号={task_request_serial}，"
                    f"当前序号={self._task_request_serial}。")
                return False
            self.try_resume_policy_service(force=True)
            return True

    def on_manual_resume_policy(self):
        """手动点击恢复Policy按钮"""
        # 防呆机制：点击后立刻禁用
        if hasattr(self, 'B_resume_policy'):
            self.B_resume_policy.setEnabled(False)

        # 1. 模式检查拦截 只有当用户明确在下拉框选择了非“自动模式”时（currentIndex > 0），才进行拦截。
        if self.on_mode_changed.currentIndex() > 0:
            current_mode = self.on_mode_changed.currentText()
            if current_mode != "自动模式":
                logging.warning(f"⚠️ 拦截恢复请求：明确处于【{current_mode}】，不允许恢复 Policy！请先切换回自动模式")
                if hasattr(self, 'B_resume_policy'): self.B_resume_policy.setEnabled(True)
                return
            
        # 2. 核心组件拦截
        if not hasattr(self, "function") or self.function is None:
            logging.error("❌ 核心组件未初始化！系统此前在非自动模式下启动，底层功能被跳过加载。请在切换自动模式后重启软件")
            if hasattr(self, 'B_resume_policy'): self.B_resume_policy.setEnabled(True)
            return
        
        # 3. 安全检查: 必须是空闲状态
        status = self._read_status_file()
        if status.get("status") == "running":
            logging.warning("⚠️ 机器人正在执行任务，请先停止后再恢复Policy")
            if hasattr(self, 'B_resume_policy'): self.B_resume_policy.setEnabled(True)
            return

        logging.info(">>> [1/3] 安全检查通过 开始恢复Policy运动")
        self.policy_suspended_by_task = False

        if hasattr(self, "w1") and self.w1 and hasattr(self.w1, "w1_manager"):
            if getattr(self.w1.w1_manager.robot_client.status, 'value', 0) == 3:  # 3 = Running
                logging.info(">>> 探测到遗留的底层活跃通道，强制 stop_move 释放，防止启动阻塞...")
                self.w1.w1_manager.stop_move()
                for _ in range(10):
                    if getattr(self.w1.w1_manager.robot_client.status, 'value', 0) != 3:
                        break
                    time.sleep(0.05)

        # 4. 强制清理旧实例与状态标志 → 委托 PolicyBridge
        if hasattr(self, "function") and self.function:
            bridge = self._get_policy_bridge()
            if bridge is not None:
                bridge.destroy()
            self.function.stop_squence = False
            logging.info(">>> [2/3] 旧实例与状态标志已完全清理")

        if hasattr(self, "w1") and self.w1:
            self.w1.stop_squence = False

        # 5. 启动新线程
        if hasattr(self, 'policy_control_thread') and self.policy_control_thread and self.policy_control_thread.isRunning():
            self.policy_control_thread.quit()
            self.policy_control_thread.wait()

        self.policy_control_thread = PolicyControlThread(self.function, work_name="Manual_Policy_Start")
        self.policy_control_thread.finished_signal.connect(self._on_policy_control_finished)
        self.policy_control_thread.start()
        logging.info(">>> [3/3] 正在重新加载模型并回原点，请稍候...")

    def _destroy_policy_instance(self):
        """彻底销毁Policy实例和线程，防止模式切换时发生幽灵恢复 → 委托 PolicyBridge"""
        logging.info("正在清理 Policy 后台实例，防止意外恢复...")
        
        # 1. 停止并退出当前的 Policy 控制线程
        if hasattr(self, 'policy_control_thread') and self.policy_control_thread:
            if self.policy_control_thread.isRunning():
                self.policy_control_thread.quit()
                self.policy_control_thread.wait(1000)
            self.policy_control_thread = None

        # 2. 清理 Function 内部的 Policy 节点和管理器状态
        if hasattr(self, "function") and self.function:
            bridge = self._get_policy_bridge()
            if bridge is not None:
                try:
                    bridge.destroy()
                except:
                    pass
            self.function.stop_squence = False

        # 3. 更新 UI 面板的指示灯状态
        if hasattr(self, "L_policy_status"):
            self.update_light(self.L_policy_status, False)
            self.L_policy_status.setText("Policy: OFF")
        # 事件驱动解锁：Policy 已销毁(不活跃)，重新启用恢复Policy按钮
        if hasattr(self, 'B_resume_policy'):
            self.B_resume_policy.setEnabled(True)
            
        logging.info("Policy 实例已彻底销毁，后续需手动触发恢复。")

    @property
    def global_state(self):
        return self._global_state

    @global_state.setter
    def global_state(self, value):
        self._global_state = value
        if hasattr(self, "function") and self.function is not None:
            self.function.status = value

    def _connect_signals(self):  # 连接所有信号和槽
        self.signals.trigger_face_track_sig.connect(self.face_track)
        # 配置读取模块
        self.cfg_read.clicked.connect(self.on_config_read)
        # 外设通讯模块
        self.B_pickwiz.clicked.connect(lambda: self.on_tool_button_action("pickwiz_start"))
        self.B_voice_service_combo.currentIndexChanged.connect( self.on_voice_service_selection)
        self.B_http.clicked.connect(lambda: self.on_tool_button_action("http"))
        self.B_vision.clicked.connect(lambda: self.on_tool_button_action("pickwiz_connection"))
        self.B_voice_service.clicked.connect(lambda: self.on_tool_button_action("voice_connection"))
        self.B_picklight_vision.clicked.connect(self.run_picklight_vision)
        self.get_kfc_param.clicked.connect(self.Get_kfc_param)   
        self.B_read_picklight_vision.clicked.connect(self.read_picklight_vision)
        self.B_asset_download.clicked.connect(self.on_asset_download)
        # self.B_stop_gohome.clicked.connect(lambda: self.on_stop_execution(stop_gohome = True))
        # self.B_stop_gohome1.clicked.connect(lambda: self.on_stop_execution(stop_gohome = False))
        # W1信息获取模块
        self.get_left_arm.clicked.connect(lambda: self.get_status("left_arm", self.T3, self.get_left_arm))
        self.get_right_arm.clicked.connect(lambda: self.get_status("right_arm", self.T40, self.get_right_arm))
        self.get_left_hand.clicked.connect(lambda: self.get_status("left_arm_ee", self.T4, self.get_left_hand))
        self.get_right_hand.clicked.connect(lambda: self.get_status("right_arm_ee", self.T4, self.get_right_hand))
        self.get_head.clicked.connect(lambda: self.get_status("head", self.T5, self.get_head))
        self.get_torso.clicked.connect(lambda: self.get_status("torso", self.T28, self.get_torso))
        # 运动控制按钮
        self.move_head.clicked.connect(lambda: self.move_part(move_type="rotate",part="head",text_field=self.T5,speed_ratio_key="head_speed",))
        self.move_torso.clicked.connect(lambda: self.move_part(move_type="rotate",part="torso",text_field=self.T28,speed_ratio_key="torso_speed",))
        self.move_right_hand.clicked.connect(lambda: self.move_part(move_type="ee", part="right_arm_ee", text_field=self.T4))
        self.move_left_hand.clicked.connect(lambda: self.move_part(move_type="ee", part="left_arm_ee", text_field=self.T4))
        self.move_right_arm.clicked.connect(lambda: self.move_part(move_type="linear_right", text_field=self.T40))
        self.move_left_arm.clicked.connect(lambda: self.move_part(move_type="linear_left", text_field=self.T3))
        # PC1通信事件
        self.set_init.clicked.connect(lambda: self.on_control_event(self.set_init))
        self.drag_open.clicked.connect(lambda: self.on_control_event(self.drag_open))
        self.drag_close.clicked.connect(lambda: self.on_control_event(self.drag_close))
        self.clear_op.clicked.connect(lambda: self.on_control_event(self.clear_op))
        self.w1_power_off.clicked.connect(lambda: self.on_control_event(self.w1_power_off))
        self.on_mode_changed.currentIndexChanged.connect(lambda index: self.on_control_event(self.on_mode_changed))
        self.B21.clicked.connect(self.on_chassis_get)
        self.B22.clicked.connect(self.on_chassis_get)
        self.B23.clicked.connect(self.on_move_chassis_nav2)
        self.B24.clicked.connect(self.on_move_chassis_ctrl)
        self.B38.clicked.connect(self.open_image_analyzer)
        self.B5.clicked.connect(self.open_json_editor)
        self.update_config.clicked.connect(self.update_json)
        self.save_pc1_log.clicked.connect(self.save_pc1)      

        self.B6.clicked.connect(lambda: self.on_execute(interface_trigger=True))
        self.B9.clicked.connect(lambda: self.on_stop_execution(stop_gohome='1')) 
        self.ppp1.clicked.connect(lambda: self.set_recharge_point()) 
        # self.B9.clicked.connect(self.on_stop_execution)
        # self.trigger_face_track_sig.connect(self.face_track)
        self.clear_log_btn.clicked.connect(self.on_clear_log)
        self.save_log_btn.clicked.connect(self.on_save_log)
        self.show_timestamp_cb.stateChanged.connect(self.on_timestamp_display_changed)
        self.log_level_combo.currentIndexChanged.connect(self.on_log_level_changed)
        self.B_detect_ros.clicked.connect(self.launch_detect_ros)
        self.B_simbyviser.clicked.connect(lambda: self.on_tool_button_action("simbyviser"))
        self.B_payment_status.clicked.connect(lambda: (
        setattr(self, 'is_payment', not self.is_payment),
        self.B_payment_status.setText('关闭支付判断' if self.is_payment else '开启支付判断')
        ))
        # # 新增支付功能按钮连接
        # self.B_payment.clicked.connect(self.toggle_payment_function)
        # 连接HTTP服务器的信号
        if hasattr(self, "http_server_thread") and self.http_server_thread:
            try:
                self.http_server_thread.log_signal.disconnect()
                self.http_server_thread.coffee_signal.disconnect()
                self.http_server_thread.consult_signal.disconnect()
                self.http_server_thread.stop_signal.disconnect()
            except:
                pass
            self.http_server_thread.log_signal.connect(self.handle_http_server_log)
            self.http_server_thread.coffee_signal.connect(self.handle_coffee_order)
            self.http_server_thread.consult_signal.connect(self.handle_consult_message)
            self.http_server_thread.stop_signal.connect(self.handle_stop_motion)
        # 恢复Policy按钮连接
        if hasattr(self, 'B_resume_policy'):
            self.B_resume_policy.clicked.connect(self.on_manual_resume_policy)


    def save_pc1(self):
        """保存PC1日志：后台执行 pc1_log_save.py"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(current_dir, "pc1_log_save.py")

        if not os.path.exists(script_path):
            logging.info(f"❌ 找不到传输脚本: {script_path}")
            return

        self.pc1_log_thread = PC1LogSaveSimpleThread(script_path)
        self.pc1_log_thread.finished_signal.connect(self._pc1_log_finished)

        if hasattr(self, "active_threads"):
            self.active_threads.append(self.pc1_log_thread)

        logging.info("正在后台拉取 PC1 日志...")
        self.pc1_log_thread.start()

    def _pc1_log_finished(self, success, msg):
        """传输脚本执行结束后的回调"""
        logging.info(msg)

    #支付功能状态的切换和发送
    def toggle_payment_function(self):
        """切换支付功能状态"""
        try:
            self.payment_enabled = not self.payment_enabled
            
            # 调用UI层的方法更新按钮样式
            self.update_payment_button_style(self.payment_enabled)
            
            status = "启用" if self.payment_enabled else "禁用"
            logging.info(f"支付功能已{status}")

            # 确保这行代码（更新auto_config.json）
            self._write_status_file("payment_enabled", self.payment_enabled)
            
            # 发送支付状态到语音服务
            self.send_payment_status_to_voice()
                
        except Exception as e:
            logging.info(f"切换支付功能失败: {str(e)}")

    def send_payment_status_to_voice(self):
        """发送支付功能状态到语音服务"""
        try:
            if hasattr(self, 'async_thread') and self.async_thread and self.async_thread.is_alive():
                status = "enabled" if self.payment_enabled else "disabled"
                message = f"PAYMENT_FEATURE_STATUS:{status}"
                
                if hasattr(self.async_thread, 'send_msg'):
                    self.async_thread.send_msg(message)
                    logging.info(f"已发送支付状态到语音服务: {status}")
                else:
                    logging.info("语音服务线程没有send_msg方法")
            else:
                logging.info("语音服务未运行，无法发送支付状态")
        except Exception as e:
            logging.info(f"发送支付状态到语音服务失败: {str(e)}")

    def is_payment_enabled(self):
        """检查支付功能是否启用"""
        return self.payment_enabled

    ## ————————————配置初始化,更新,日志相关函数——————————————
    def update_json(self):  # 更新配置 TODO:功能需完善,调用方式
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_rel_path = self.T15.text().strip()
        if not config_rel_path:
            QMessageBox.warning(self.main_window, "警告", "请先读取配置文件")
            return
        config_abs_path = os.path.abspath(os.path.join(current_dir, config_rel_path))
        config_dir = os.path.dirname(config_abs_path)
        param_path = os.path.join(config_dir, "parameter.json")
        task_path = os.path.join(config_dir, "task.json")
        calibration_path = os.path.join(config_dir, "calibration.json")
        device_path = os.path.join(config_dir, "device.json")
        self.parameter_cfg = Initialize.load_json_config(param_path, "参数配置文件")
        self.origin_cfg = copy.deepcopy(self.parameter_cfg)
        self.task_cfg = Initialize.load_json_config(task_path, "任务配置文件")[
            "work_sequence"
        ]
        self.calibration_cfg = Initialize.load_json_config(
            calibration_path, "标定配置文件"
        )
        self.device_cfg = Initialize.load_json_config(device_path, "主配置文件")
        self.config = Initialize.load_config(config_abs_path)
        self.function.init = self.config
        target_task, task_id = self._find_first_active_task(self.task_cfg)
        self.is_local = self.device_cfg["w1_tcp_config"]["is_local"]
        self.is_sim = self.device_cfg.get("w1_tcp_config", {}).get("is_sim", False)
        self._populate_task_dropdowns(self.task_cfg, task_id)  # 填充到下拉框
        self.function.config = self.config
        self.function.vision_app = self.vision_app
        self.w1.log_msg = self.config["log_msg"]
        self.w1.parameter_cfg =  self.parameter_cfg
        self.parameter_path = param_path 
        
        if hasattr(self, 'function') and self.function:
            self.function.parameter = self.parameter_cfg.get("workspace", {})
            
            self.function.parameter_cfg = self.parameter_cfg
            
            if hasattr(self.function, 'init') and isinstance(self.function.init, dict):
                self.function.init['parameter_cfg'] = self.parameter_cfg
                
        # 3. 重置专项任务缓存，确保刷新后是从基础配置开始，而不是带着之前的专项配置
        self.task_config = None
        if self.parameter_cfg["use_picklight_vision"]["use"]:
            config_path = self.parameter_cfg["use_picklight_vision"]["config_path"]
            with open(config_path, "r", encoding="utf-8") as f:
                self.vision_param = json.load(f)
            log_dir = self.vision_param["logging_path"]
            self.picklight_vision.re_register(self.vision_logger, self.vision_param)
            logging.info("视觉配置重新注册成功")
        self.syslog.append("配置文件更新成功")

    def _find_first_active_task(self, task_cfg):  # 查找第一个激活的任务
        for task_id, task_info in task_cfg.items():
            if (
                isinstance(task_info, list)
                and len(task_info) > 1
                and task_info[-1] == 1
            ):
                return task_info, task_id
        return None, None

    def _populate_task_dropdowns(
        self, task_cfg, default_task_id=None
    ):  # 填充下拉框并切换到指定任务
        # 保存当前选中的ID（从文本中提取）
        current_text = self.select_module.currentText()
        current_task_id = None
        
        # 从当前文本中提取task_id
        if current_text and ": " in current_text:
            try:
                current_task_id = current_text.split(": ")[0]
            except:
                pass
        
        self.select_module.clear()
        self.select_module.blockSignals(True)
        try:
            sorted_tasks = sorted(task_cfg.items(), key=lambda x: int(x[0]))
            found_default = False
            
            # 先添加所有项目并记录需要更新的项
            update_item = None
            for task_id, task_info in sorted_tasks:
                if task_info[0]:
                    display_text = f"{task_id}: {task_info[0]}"
                    self.select_module.addItem(display_text)
                    
                    # 检查这个task_id是否是当前选中的项（内容可能已更改）
                    if str(task_id) == str(current_task_id):
                        update_item = display_text  # 记录更新后的文本
                    
                    if str(task_id) == str(default_task_id):
                        found_default = True
            
            # 优先级逻辑：
            # 1. 如果当前选中的项内容有变化，切换到更新后的新内容
            if update_item:
                self.select_module.setCurrentText(update_item)
            # 2. 如果没有要更新的项，但存在默认项，使用默认项
            elif found_default:
                # 找到默认项对应的显示文本
                for i in range(self.select_module.count()):
                    if f"{default_task_id}:" in self.select_module.itemText(i):
                        self.select_module.setCurrentIndex(i)
                        break
            # 3. 如果以上都不满足，且下拉框不为空，保持为空或设为第一项
            elif self.select_module.count() > 0:
                # 这里可以根据需求决定：是保持为空还是设为第一项
                # 如果要保持为空，就不设置；如果要设为第一项：
                # self.select_module.setCurrentIndex(0)
                pass  # 保持当前状态（为空）
            
        finally:
            self.select_module.blockSignals(False)

    def append_log_with_timestamp(self, message):  # 日志框日志
        print(message, flush=True)
        if self.show_timestamp_cb.isChecked():
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S - ")
            self.syslog.append(timestamp + message)
        else:
            self.syslog.append(message)

    def _safe_append_log(self, message):  # 安全添加日志，防止控件已被销毁的情况
        try:
            if hasattr(self, "syslog") and self.syslog is not None:
                if (
                    hasattr(self, "show_timestamp_cb")
                    and self.show_timestamp_cb is not None
                ):
                    if self.show_timestamp_cb.isChecked():
                        timestamp = time.strftime("%Y-%m-%d %H:%M:%S - ")
                        self.syslog.append(timestamp + message)
                    else:
                        self.syslog.append(message)
                else:
                    self.syslog.append(message)
        except RuntimeError:
            pass

    async def _load_config_file_async(
        self, file_path, is_first_load=False, auto_pickwiz=False
    ):
        """异步并行加载配置文件"""
        try:
            if self.auto_voice == True:
                self.B_voice_service_combo.setCurrentText("启用语音服务")
                self.B_voice_service_combo.currentIndexChanged.emit(self.B_voice_service_combo.currentIndex())
            target_task, task_id = self._find_first_active_task(self.task_cfg)
            self._populate_task_dropdowns(self.task_cfg, 0)

            current_dir = os.path.dirname(os.path.abspath(__file__))
            rel_path = os.path.relpath(file_path, current_dir)
            self.T15.setText(rel_path)
            logging.info(f"配置文件读取成功: {rel_path}")
            self.update_light(self.G3, True)

            self.vision_app = VisionApp(self.config)
            status_msg = self._read_status_file()
            # 创建加载对话框
            loading_dialog = LoadingDialog(self.main_window)
            loading_dialog.show()
            try:
                if self.auto_voice_new:
                    self._init_new_voice_ui(True)
                # 并行执行两个耗时任务
                loading_dialog.update_status("开始并行加载机器人应用和视觉序列...", 10)

                # 使用线程池执行阻塞操作
                with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                    loop = asyncio.get_event_loop()

                    # 提交任务到线程池
                    vision_task = loop.run_in_executor(executor, self._load_vision_sequence)
                    robot_task = loop.run_in_executor(executor, self._load_robot_app)
                    

                    loading_dialog.update_status("机器人应用加载中...", 30)

                    # 等待任务完成，设置超时
                    done, pending = await asyncio.wait(
                        [vision_task,robot_task],
                        timeout=1000.0,
                        return_when=asyncio.ALL_COMPLETED,
                    )

                    loading_dialog.update_status("处理加载结果...", 80)

                    # 处理完成的任务
                    robot_result = None
                    vision_result = None
                    errors = []

                    for task in done:
                        try:
                            result = task.result()
                            if task == vision_task:
                                vision_result = result
                                loading_dialog.update_status("视觉序列注册完成", 50)
                            elif task == robot_task:
                                robot_result = result
                                loading_dialog.update_status("机器人应用加载完成", 75)
                            
                             
                        except Exception as e:
                            errors.append(str(e))

                    # 取消未完成的任务
                    for task in pending:
                        task.cancel()

                    # 处理错误
                    if errors:
                        error_msg = "；".join(errors)
                        self._safe_append_log(f"部分组件加载失败: {error_msg}")
                    # 设置结果
                    self.w1 = robot_result
                    loading_dialog.update_status("初始化功能模块...", 90)
                    if self.auto_payment:
                        self.start_payment_service()
                        
    
            except asyncio.TimeoutError:
                loading_dialog.update_status("加载超时，使用降级方案", 100)
                self._safe_append_log("配置加载超时，使用默认配置")
                self.w1 = None
            except Exception as e:
                loading_dialog.update_status(f"加载错误: {str(e)}", 100)
                raise
            finally:
                loading_dialog.close()

            # 初始化功能模块
            if self.w1 is not None:
                # ----------------------
                try:
                    from modules.chassis.chassis_controll import ChassisControll
                    self.chassis_controll = ChassisControll(self.w1)
                    logging.warning("################底盘导入成功")
                except Exception as e:
                    logging.error(f"底盘控制初始化失败: {str(e)}")
                    self.chassis_controll = None
                # -------------------------------
                try:
                    
                    self.function = Function(
                        self.config,
                        self.vision_app,
                        robot_app=self.w1,
                        chassis_controll=self.chassis_controll,
                        picklight_vision=self.picklight_vision,
                        controller=self
                    )
                    if self.parameter_cfg["use_robot"]:
                        self.w1.w1_manager.set_safe_level(
                            float(self.device_cfg["w1_tcp_config"]["set_safe_level"])
                        )
                        logging.info(
                            "机械臂安全等级初始化为:"
                            + str(self.device_cfg["w1_tcp_config"]["set_safe_level"])
                        )
                        self.update_light(self.G2, True)
                        logging.info("机器人功能模块初始化成功")
                except Exception as e:
                    logging.info(f"功能模块初始化失败: {e}")
                    self.parameter_cfg["use_robot"] = False
            else:
                self.syslog.append("机器人未启动")
                self.function = Function(self.config, self.vision_app)
                if self.log_msg["no_robot_warehouse"][1] == 1:
                    logging.error(
                        self.log_msg["no_robot_warehouse"][0].format("初始化失败")
                    )
                self.parameter_cfg["use_robot"] = False

            # 其他初始化逻辑
            if self.device_cfg["ui_param"]["auto_pickwiz"] == True:
                self.on_tool_button_action("pickwiz_connection")

            # if self.parameter_cfg["use_robot"] and not self.is_sim:
                # while status_msg.get("op_status") != True:
                #     time.sleep(3)
                #     status_msg = self._read_status_file()
                #     if self.log_msg["pc1_op_error"][1] == 1:
                #         logging.info(self.log_msg["pc1_op_error"][0].format())

            voice_status = False
            running = True
            if self.auto_voice == True:
                while running:
                    config_file = (
                        "/home/dexforce/workspace/dexe_application/qt/auto_config.json"
                    )
                    if os.path.exists(config_file):
                        with open(config_file, "r", encoding="utf-8") as f:
                            config_data = json.load(f)
                    if (
                        config_data["voice_service_status"] == True
                        and voice_status == False
                    ):
                        self.on_tool_button_action("voice_connection")
                        time.sleep(2)
                        voice_status = True
                        running = False
                    else:
                        if self.log_msg["voice_auto_connect"][1] == 1:
                            logging.info(self.log_msg["voice_auto_connect"][0].format())
                        time.sleep(3)

            self.execute_voice_command(self.parameter_cfg["start_voice"])

            self.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "w1_ui", "true")
            logging.info("配置文件加载即将完成")
            loading_dialog.update_status("加载即将完成", 80)
            logging.info("配置文件加载完成")
            loading_dialog.update_status("加载完成", 100)

        except Exception as e:
            logging.info(f"配置文件读取失败: {str(e)}")
            self.update_light(self.G3, False)
            if "loading_dialog" in locals():
                loading_dialog.close()

    def _load_config_file(self, file_path, is_first_load=False, auto_pickwiz=False):
        """同步包装方法，用于保持现有接口兼容性"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            task = self._load_config_file_async(file_path, is_first_load, auto_pickwiz)
            result = loop.run_until_complete(task)
            return result
        except Exception as e:
            logging.info(f"异步加载失败: {str(e)}")
            return self._load_config_file_fallback(
                file_path, is_first_load, auto_pickwiz
            )  # 降级到同步加载
        finally:
            loop.close()

    def _load_config_file_fallback(
        self, file_path, is_first_load=False, auto_pickwiz=False
    ):
        """降级同步加载方法"""
        logging.info("使用同步降级方案加载配置...")

        try:
            # 原有的同步加载逻辑
            # self.w1manage_status_monitor("stop")
            # self.w1manage_status_monitor("start")
            if hasattr(self, 'status_thread') and self.status_thread:
                self.status_thread.stop_monitoring()
            if hasattr(self, 'ros_battery_thread') and self.ros_battery_thread:
                self.ros_battery_thread.stop_monitoring()

            self.status_thread = StatusMonitorThread(self, self.device_cfg, is_sim=self.is_sim)
            self.status_thread.start()
            
            self.ros_battery_thread = RosBatteryMonitorThread(self)
            self.ros_battery_thread.update_signal.connect(self.op_clec_update)
            self.ros_battery_thread.update_signal.connect(self.low_battery_go_home)
            self.ros_battery_thread.refresh_signal.connect(self.op_clec_update)
            self.ros_battery_thread.charging_state_signal.connect(
                self.update_charging_state,
                Qt.ConnectionType.DirectConnection)
            self.ros_battery_thread.start()

            target_task, task_id = self._find_first_active_task(self.task_cfg)
            self._populate_task_dropdowns(self.task_cfg, 0)

            current_dir = os.path.dirname(os.path.abspath(__file__))
            rel_path = os.path.relpath(file_path, current_dir)
            self.T15.setText(rel_path)
            logging.info(f"配置文件读取成功: {rel_path}")
            self.update_light(self.G3, True)

            self.vision_app = VisionApp(self.config)
            status_msg = self._read_status_file()

            # while status_msg.get("op_status") != True:
            #     time.sleep(3)
            #     status_msg = self._read_status_file()
            #     if self.log_msg["pc1_op_error"][1] == 1:
            #         logging.info(self.log_msg["pc1_op_error"][0].format())

            # 顺序加载（原有的串行逻辑）
            loading_dialog = LoadingDialog(self.main_window)
            loading_dialog.show()

            try:
                loading_dialog.update_status("加载机器人应用...", 30)
                self.w1 = self._load_robot_app()
                loading_dialog.update_status("机器人应用加载完成", 60)

                loading_dialog.update_status("注册视觉序列...", 80)
                if self.parameter_cfg["use_picklight_vision"]["use"]:
                    self._load_vision_sequence()
                loading_dialog.update_status("视觉序列注册完成", 90)

            finally:
                loading_dialog.close()

            self.function = Function(self.config, self.vision_app, self.w1)
            self.w1.w1_manager.set_safe_level(
                float(self.device_cfg["w1_tcp_config"]["set_safe_level"])
            )
            logging.info(
                "机械臂安全等级初始化为:"
                + str(self.device_cfg["w1_tcp_config"]["set_safe_level"])
            )
            self.update_light(self.G2, True)

            if self.device_cfg["ui_param"]["auto_pickwiz"] == True:
                self.on_tool_button_action("pickwiz_connection")

        except Exception as e:
            logging.info(f"降级加载失败: {str(e)}")
            self.update_light(self.G3, False)

    def _load_robot_app(self):  # 初始化时,线程加载w1_manager
        """加载机器人应用"""
        try:
            if not self.parameter_cfg["use_robot"]:
                return None
            logging.info("开始加载机器人应用...")
            robot_app = RobotApp(self.config["log_msg"], self.parameter_cfg,self.device_cfg, is_sim=self.is_sim)
            self.robot_app = robot_app
            self._write_status_file("op_status", True)
            self._safe_append_log("机器人应用加载成功")
            return robot_app
        except Exception as e:
            logging.info(f"机器人应用加载失败: {e}")
            raise
 
    def _load_vision_sequence(self):  # 初始化时,线程加载yolo,双目模型
        """加载视觉序列"""
        try:
            if not self.parameter_cfg["use_picklight_vision"]["use"]:
                logging.info("picklight视觉未启动")
                return None

            logging.info("开始注册视觉序列...")

            vision_dir = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), "vision"
            )
            sys.path.insert(0, vision_dir)

            from vision.vision_squence import picklight_vision
            from vision.RotatingDailyLogger import RotatingDailyLogger

            config_path = self.parameter_cfg["use_picklight_vision"]["config_path"]
            with open(config_path, "r", encoding="utf-8") as f:
                self.vision_param = json.load(f)

            log_dir = self.vision_param["logging_path"]
            self.vision_logger = RotatingDailyLogger(log_dir=log_dir, max_lines=100000)
            self.picklight_vision = picklight_vision()
            status = self.picklight_vision.register(self.vision_logger, self.vision_param)
            if status == 'camera_fail':
                logging.info("❌ 头部相机取图失败,请检查相机状态")   
            else:
                logging.info("视觉序列注册成功")
            return True
        except Exception as e:
            logging.info(f"视觉序列注册失败: {e}")  
            raise

    def read_json_value(self, file_path, key_chain):
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
            current_level = data
            for key in key_chain:
                if isinstance(current_level, dict) and key in current_level:
                    current_level = current_level[key]
                else:
                    print(f"错误：在路径 {' -> '.join(key_chain)} 中找不到键 '{key}'")
                    return None
            return current_level
        except Exception as e:
            print(f"读取JSON时发生错误：{e}")
            return None

    def update_json_value(self, filename, key, new_value):
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if key in data:
                if isinstance(new_value, str):
                    if new_value.lower() == "true":
                        new_value = True
                    elif new_value.lower() == "false":
                        new_value = False
                elif isinstance(data[key], bool) and isinstance(new_value, str):
                    if new_value.lower() in ["true", "false"]:
                        new_value = new_value.lower() == "true"
                data[key] = new_value
                print(f"{key} = {new_value}")
            else:
                print(f"键 '{key}' 不存在于文件中")
                return False
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=4)
            return True
        except Exception as e:
            print(f"修改失败: {e}")
            return False

    def on_config_read_origin(self):  # 读取配置文件初始化参数
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            target_dir = os.path.normpath(
                os.path.join(current_dir, "../application/parameters")
            )
            if not os.path.exists(target_dir):
                os.makedirs(target_dir)
            file_path = target_dir + "/device.json"
            if not file_path:
                logging.info("未选择配置文件")
                self.update_light(self.G3, False)
                return
            # 尝试初始化日志系统
            try:
                ColorLogger.init()
            except ValueError as e:
                import sys, io

                sys.stdout = io.TextIOWrapper(
                    open(sys.stdout.fileno(), "w", encoding="utf-8")
                )
                ColorLogger.init()
            self.config = Initialize.load_config(file_path)
            self.log_msg = self.config["log_msg"]



            file_path1 = "/home/dexforce/workspace/dexe_application/vision/config/vision_config.json"
            with open(file_path1, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            if data['stereo_param']["use_model"]=='trt':
                logging.info("✅ 使用trt模型,启动视觉服务")
                # try:
                #     script_path = "/home/dexforce/workspace/dexe_application/qt/start_vision_services.sh"
                    
                #     # 使用 gnome-terminal 打开新窗口执行脚本
                #     # -- bash -c "..." 是为了在脚本执行完后不立即关闭窗口（可选）
                #     self.service_bash_process = subprocess.Popen([
                #         "gnome-terminal", 
                #         "--", 
                #         "bash", "-c", f"bash {script_path}; exec bash" 
                #     ])
                    
                #     logging.info("✅ FVision 终端窗口已弹出并启动服务")
                # except Exception as e:
                #     logging.error(f"无法打开终端窗口: {e}")
                try:
                    script_path = "/home/dexforce/workspace/dexe_application/qt/start_vision_services.sh"
                    self.service_bash_process = subprocess.Popen(["bash", script_path])
                    logging.info("✅ FVision 容器服务已在后台启动")
                except Exception as e:
                    logging.error(f"后台服务脚本启动失败: {e}")
            else:
                logging.info("✅ 使用pth模型,不启动视觉服务")

            # 拉起 dexe_recorder 守护进程（常驻后台，通过 recorderctl.sh 控制录制启停）
            try:
                recorder_script = "/home/dexforce/workspace/dexe_recorder/run_recorder.sh"
                if os.path.exists(recorder_script):
                    self.recorder_process = subprocess.Popen(["bash", recorder_script])
                    logging.info("✅ dexe_recorder 录制守护进程已在后台启动")
                else:
                    logging.warning(f"⚠️ 录制启动脚本不存在: {recorder_script}")
            except Exception as e:
                logging.error(f"录制守护进程启动失败: {e}")

            file_path1 = "/home/dexforce/workspace/dexe_application/vision/config/trt_config.json"
            search_str = "home/dexforce/workspace"  # 要查找的字符串
            # embed()
            with open(file_path1, 'r', encoding='utf-8') as f:
                content = f.read()
            if search_str in content:
                logging.error(f"trt_config中不应该存在home/dexforce/workspace这个路径，请修改后重新打开软件")

            self.device_cfg = self.config["device_cfg"]
            self.calibration_cfg = self.config["calibration_cfg"]
            self.parameter_cfg = self.config["parameter_cfg"]
            self.origin_cfg = copy.deepcopy(self.parameter_cfg)
            self.parameter_path = os.path.join(target_dir, "parameter.json")
            self.task_cfg = self.config["task_cfg"]["work_sequence"]
            auto_http_serve = self.device_cfg["ui_param"]["auto_http_serve"]
            auto_flaskServer = self.device_cfg['ui_param']['auto_flaskServer']
            auto_voiceStatus = self.device_cfg['ui_param']['auto_voiceStatus']
            self.auto_voice = self.device_cfg["ui_param"]["auto_voice"]
            self.auto_voice_new = self.device_cfg["ui_param"]["auto_voice_new"]
            self.auto_payment = self.device_cfg["ui_param"].get("auto_payment", False)
            auto_pickwiz = self.device_cfg["ui_param"]["auto_pickwiz"]
            self.is_local = self.device_cfg["w1_tcp_config"]["is_local"]
            self.is_sim = self.device_cfg.get("w1_tcp_config", {}).get("is_sim", False)
            self._write_status_file("voice_service_status", False)
            self._write_status_file("pickwiz_status", False)
            self._write_status_file("status", "idle")
            self._write_status_file("finish", True)
            self._write_status_file("op_status", False)
            self._write_status_file("goinghome", False)
            self._write_status_file("face_tracking", False)  
            self._write_status_file("capsule_coffeemachine_num", 0)
            self._write_status_file("coffeemachine_threshold", False)
            self._write_status_file("capsule_threshold_value", 15)
            self._write_status_file("agent_msg", "")
            self._write_status_file("voice_service_new_status", False)
            self._write_status_file("w1_ui", False)
            self._write_status_file("health_endpoint", False)
            self._write_status_file("network_endpoint", False)
            self._write_status_file("microphone_endpoint", False)
            self._write_status_file("audio_endpoint", False)
            self._write_status_file("getMoney", False)
            self._write_status_file("payment_enabled", False)
            self._write_status_file("statusCoffee", "origin")
            self._write_status_file("face_tracking", False)

             # 新增：支付功能初始化
            payment_enabled = self.device_cfg["ui_param"].get("payment_enabled", False)
            self.payment_enabled = payment_enabled
            self._write_status_file("payment_enabled", payment_enabled)
            time.sleep(0.5)
            if auto_http_serve == True:
                self.on_tool_button_action("http")
            if auto_pickwiz == True:
                self.on_tool_button_action("pickwiz_start")
            if auto_flaskServer == True:
                self.setup_flask_server()
            if auto_voiceStatus == True:
                self.start_voice_monitoring()   
            if auto_pickwiz == True:
                while running:
                    config_file = (
                        "/home/dexforce/workspace/dexe_application/qt/auto_config.json"
                    )
                    if os.path.exists(config_file):
                        with open(config_file, "r", encoding="utf-8") as f:
                            config_data = json.load(f)
                    if (
                        config_data["pickwiz_status"] == True
                        and pickwiz_status == False
                    ):
                        pickwiz_status = True
                        running = False
                    else:
                        if self.log_msg["pickwiz_auto_connect"][1] == 1:
                            logging.info(
                                self.log_msg["pickwiz_auto_connect"][0].format()
                            )
                        time.sleep(3)
            # self.w1manage_status_monitor("stop")
            # self.w1manage_status_monitor("start")
            if hasattr(self, 'status_thread') and self.status_thread and self.status_thread.isRunning():
                self.status_thread.stop_monitoring()
                self.status_thread.quit()
                self.status_thread.wait(500)
            self.status_thread = StatusMonitorThread(self, self.device_cfg, is_sim=self.is_sim)
            if not self.is_sim:
                self.status_thread._start_client()
            self.status_thread.start()

            # 启动基于 ROS 的电量监控
            if hasattr(self, 'ros_battery_thread') and self.ros_battery_thread and self.ros_battery_thread.isRunning():
                self.ros_battery_thread.stop_monitoring()
                self.ros_battery_thread.quit()
                self.ros_battery_thread.wait(500)
            self.ros_battery_thread = RosBatteryMonitorThread(self)
            self.ros_battery_thread.update_signal.connect(self.op_clec_update)
            self.ros_battery_thread.update_signal.connect(self.low_battery_go_home)
            self.ros_battery_thread.refresh_signal.connect(self.op_clec_update)
            self.ros_battery_thread.charging_state_signal.connect(
                self.update_charging_state,
                Qt.ConnectionType.DirectConnection)
            self.ros_battery_thread.start()


            data = ''
            message='get_mode'
            self.client_socket = self.status_thread.client_socket
            self.status_thread.tcp_monitoring_enabled = False
            logging.info(f"获取当前模式中...")  
            data='setting'
            while data=='setting' and  self.status_thread.client_connected == True:
                self.client_socket.sendall(message.encode("utf-8"))
                data = self.client_socket.recv(
                    self.device_cfg["w1_tcp_config"]["buffer_size"]
                ).decode("utf-8")
                logging.info(f"当前模式为:{data}")


                    
            self.client_socket.sendall(message.encode("utf-8"))
            data = self.client_socket.recv(
                self.device_cfg["w1_tcp_config"]["buffer_size"]
            ).decode("utf-8")
            logging.info(f"当前模式为:{data}")                    
            self.status_thread.tcp_monitoring_enabled = True

            if data == 'auto_mode':
                self._load_config_file(file_path, is_first_load=True, auto_pickwiz=True)
            else:
                logging.info("当前为遥操模式,仅启动Tcp通讯,UI功能仅模式切换生效")
            
            # self.face_track()

            if self.device_cfg.get("ui_param", {}).get("act_control", False) == True:
                if hasattr(self, 'function') and self.function:
                    # 清理可能存在的旧线程
                    if hasattr(self, 'policy_control_thread') and self.policy_control_thread and self.policy_control_thread.isRunning():
                        self.policy_control_thread.quit()
                        self.policy_control_thread.wait()
                    # 启动 Policy 线程
                    self.policy_control_thread = PolicyControlThread(self.function)
                    self.policy_control_thread.finished_signal.connect(self._on_policy_control_finished)
                    self.policy_control_thread.start()
                    logging.info("检测到 ui_param.act_control=True，正在后台启动 Policy 服务...")

        except Exception as e:
            logging.info(f"配置文件读取错误: {str(e)}")
            self.update_light(self.G3, False)

    def _request_auto_charging(self, trigger: str) -> bool:
        """在非任务状态下启动一次后台回充，避免阻塞界面与重试定时器。"""
        now = time.monotonic()

        charging_status = self._get_auto_charging_runtime_status()
        if charging_status in {
                "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING",
                "CANCELING"}:
            with self._chassis_state_lock:
                # 保留待回充标志：若现有回充失败，定时器还能继续接手；一旦
                # 诊断确认已经充电，update_charging_state() 会清除此标志。
                self.pending_auto_charging = True
            logging.info(
                "[低电量回充] 跳过重复请求：已有回充 Action 正在执行，"
                f"状态={charging_status}（{trigger}）。")
            return False

        with self._chassis_state_lock:
            if self.is_force_working:
                self.pending_auto_charging = True
                logging.info(
                    f"[低电量回充] 暂不请求：任务正在占用底盘，"
                    f"已标记待回充（{trigger}）。")
                return False
            if self.is_charging_cycle:
                self.pending_auto_charging = True
                logging.info(
                    f"[低电量回充] 跳过请求：当前仍处于充电周期"
                    f"，已保留待回充状态（{trigger}）。")
                return False
            if self._auto_charging_requesting:
                logging.info(
                    f"[低电量回充] 跳过请求：已有回充请求正在处理"
                    f"（{trigger}）。")
                return False
            elapsed = now - self._last_auto_charging_attempt_time
            if elapsed < self._auto_charging_retry_interval:
                remaining = self._auto_charging_retry_interval - elapsed
                logging.info(
                    f"[低电量回充] 暂不重试：处于请求冷却期，"
                    f"剩余 {remaining:.1f} 秒（{trigger}）。")
                return False
            self._auto_charging_requesting = True

        logging.warning(f"[低电量回充] 电量触发，开始请求回充（{trigger}）。")
        worker = threading.Thread(
            target=self._run_auto_charging_request,
            args=(trigger,),
            name="auto_charging_worker",
            daemon=True)
        self._auto_charging_worker = worker
        try:
            worker.start()
        except Exception as e:
            with self._chassis_state_lock:
                self._auto_charging_requesting = False
                self._last_auto_charging_attempt_time = time.monotonic()
                self.pending_auto_charging = True
            self._auto_charging_worker = None
            logging.error(f"[低电量回充] 后台线程启动失败: {str(e)}")
            return False
        return True

    def _get_auto_charging_runtime_status(self) -> str:
        """读取真实回充 Action 状态；不可用时返回 UNKNOWN。"""
        manager = (getattr(self.w1, "w1_manager", None)
                   if self.w1 is not None else None)
        if manager is None:
            return "UNKNOWN"
        try:
            _, status = manager.get_auto_charging_status()
            return status
        except Exception as e:
            logging.warning(f"[低电量回充] 查询回充 Action 状态失败: {str(e)}")
            return "UNKNOWN"

    def _run_auto_charging_request(self, trigger: str) -> None:
        """后台发送回充目标并等待结果，失败后保留待回充状态。"""
        try:
            try:
                result = self.w1.w1_manager.set_auto_charging()
                request_success = getattr(result, "name", None) == "SUCCESS"
            except Exception as e:
                logging.error(f"[低电量回充] 回充请求异常: {str(e)}")
                request_success = False

            if not request_success:
                charging_status = self._get_auto_charging_runtime_status()
                with self._chassis_state_lock:
                    self._auto_charging_requesting = False
                    self.pending_auto_charging = True
                    if charging_status not in {
                            "WAITING_SERVER", "SENDING", "ACCEPTED",
                            "EXECUTING", "CANCELING"}:
                        self._last_auto_charging_attempt_time = time.monotonic()
                if charging_status in {
                        "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING",
                        "CANCELING"}:
                    logging.info(
                        "[低电量回充] 发送时发现已有回充 Action，"
                        f"状态={charging_status}；不按失败处理。")
                    return
                logging.error(
                    f"[低电量回充] 回充目标发送失败，"
                    f"{self._auto_charging_retry_interval:.0f} 秒后自动重试。")
                return

            wait_success = False
            wait_message = ""
            try:
                _, wait_success, wait_message = (
                    self.w1.w1_manager.wait_for_auto_charging_done(
                        timeout=300.0))
            except Exception as e:
                logging.error(f"[低电量回充] 等待回充结果异常: {str(e)}")
                wait_message = str(e)

            with self._chassis_state_lock:
                self._auto_charging_requesting = False
                self._last_auto_charging_attempt_time = time.monotonic()
                if wait_success:
                    self.is_charging_cycle = True
                    self._charging_cycle_confirmed_time = time.monotonic()
                    self._charger_disconnected_stable_count = 0
                    self.pending_auto_charging = False
                else:
                    self.is_charging_cycle = False
                    self.pending_auto_charging = True

            if wait_success:
                logging.info(
                    f"[低电量回充] 回充成功，message={wait_message}。")
                self._refresh_battery_snapshot("回充成功后")
            else:
                logging.error(
                    f"[低电量回充] 回充失败，message={wait_message}，"
                    f"{self._auto_charging_retry_interval:.0f} 秒后自动重试。")
        except Exception as e:
            with self._chassis_state_lock:
                self._auto_charging_requesting = False
                self._last_auto_charging_attempt_time = time.monotonic()
                self.pending_auto_charging = True
            logging.exception(f"[低电量回充] 后台回充线程异常: {str(e)}")
        finally:
            self._auto_charging_worker = None

    def _retry_pending_auto_charging(self) -> None:
        """低电量回充失败后，脱离电量变化事件按冷却时间自动重试。"""
        now = time.monotonic()
        charging_status = self._get_auto_charging_runtime_status()
        charging_action_active = charging_status in {
            "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING", "CANCELING"
        }
        with self._chassis_state_lock:
            is_startup_pending = self._last_auto_charging_attempt_time <= 0.0
            should_retry = (
                self.pending_auto_charging and
                0 < self.last_checked_battery <= 30 and
                self.w1 is not None and
                getattr(self.w1, "w1_manager", None) is not None and
                not self.is_force_working and
                not self.is_charging_cycle and
                not self.is_battery_charging and
                not self.is_charger_connected and
                not self.is_charging_current_detected and
                not self._auto_charging_requesting and
                not charging_action_active and
                now - self._last_auto_charging_attempt_time >=
                self._auto_charging_retry_interval)

        if not should_retry:
            return

        if is_startup_pending:
            logging.warning(
                f"[低电量回充] 机器人初始化完成，当前电量 "
                f"{self.last_checked_battery}%，开始执行待回充请求。")
            trigger = "启动后补发低电量回充"
        else:
            logging.warning(
                f"[低电量回充] 冷却结束，当前电量 "
                f"{self.last_checked_battery}%，自动重试回充。")
            trigger = "失败后定时重试"
        self._request_auto_charging(trigger)

    def _refresh_battery_snapshot(self, reason: str) -> None:
        """在关键流程边界，用最新诊断刷新电量、电压和电流显示。"""
        battery_thread = getattr(self, "ros_battery_thread", None)
        if battery_thread is None or not battery_thread.isRunning():
            logging.warning(f"[电量刷新][{reason}] 电量监控线程未运行。")
            return
        battery_thread.refresh_latest_battery(reason, timeout=3.0)

    def _on_execute_task_finished(self, task_thread) -> bool:
        """等待任务触发的回充终态，再释放底盘占用并恢复后续流程。"""
        auto_charging_was_active = False
        auto_charging_success = True

        should_request_auto_charging = False
        with self._chassis_state_lock:
            # 旧任务结束时不能清除新任务的占用状态。
            if getattr(self, "thread", None) is not task_thread:
                return True

        manager = (getattr(self.w1, "w1_manager", None)
                   if self.w1 is not None else None)
        if manager is not None:
            try:
                _, charging_status = manager.get_auto_charging_status()
                auto_charging_was_active = charging_status in {
                    "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING",
                    "CANCELING"
                }
                if auto_charging_was_active:
                    logging.info(
                        "[回充任务收尾] 检测到回充 Action 仍在执行，"
                        f"当前状态={charging_status}；等待最终结果后再结束任务。")
                    _, auto_charging_success, charging_message = (
                        manager.wait_for_auto_charging_done(timeout=300.0))
                    with self._chassis_state_lock:
                        if auto_charging_success:
                            self.is_charging_cycle = True
                            self._charging_cycle_confirmed_time = (
                                time.monotonic())
                            self._charger_disconnected_stable_count = 0
                            self.pending_auto_charging = False
                        else:
                            self.is_charging_cycle = False

                    if auto_charging_success:
                        logging.info(
                            "[回充任务收尾] 回充成功，"
                            f"message={charging_message}。")
                        self._refresh_battery_snapshot("回充成功后")
                    else:
                        logging.error(
                            "[回充任务收尾] 回充失败，"
                            f"message={charging_message}。")
            except Exception as e:
                auto_charging_was_active = True
                auto_charging_success = False
                logging.exception(f"[回充任务收尾] 等待回充结果异常: {str(e)}")

        with self._chassis_state_lock:
            # 等待期间若新任务已经替换旧任务，不能释放新任务的占用状态。
            if getattr(self, "thread", None) is not task_thread:
                return auto_charging_success
            self.is_force_working = False
            should_request_auto_charging = (
                self.pending_auto_charging and not self.is_charging_cycle)

        if not auto_charging_was_active:
            self._refresh_battery_snapshot("任务结束后")

        if should_request_auto_charging:
            logging.info("[低电量回充] 当前任务结束，继续执行待回充请求。")
            self._request_auto_charging("任务结束")
        return auto_charging_success

    def _undock_after_full_charge(self, current_battery: int) -> None:
        """满电后先确认离桩成功，再允许导航到接待处。"""
        now = time.monotonic()
        with self._chassis_state_lock:
            if self.is_force_working:
                logging.info("[满电离桩] 任务正在使用底盘，跳过自动回接待处。")
                return
            # is_charging_cycle 只是回充流程标志，回充被外部中断或手动离桩后
            # 可能短时间残留，不能单独作为发送退桩运动的依据。
            charging_state_fresh = (
                self._last_charging_state_observed_time is not None and
                now - self._last_charging_state_observed_time <=
                self._charging_state_freshness_timeout)
            physically_docked = (
                charging_state_fresh and
                (self.is_battery_charging or self.is_charger_connected or
                 self.is_charging_current_detected))
            if (not physically_docked or self._full_charge_undocking):
                if (self.is_charging_cycle and not charging_state_fresh):
                    logging.warning(
                        "[满电离桩] 充电物理状态已超过 5 秒未更新；为避免使用"
                        "过期状态误退桩，本次不发送底盘运动。")
                elif self.is_charging_cycle and not physically_docked:
                    logging.warning(
                        "[满电离桩] 仅检测到充电周期标志，未检测到正在充电或"
                        "充电器连接；为避免误退桩，本次不发送底盘运动。")
                return
            if now - self._last_full_charge_undock_attempt_time < \
                    self._full_charge_undock_retry_interval:
                return
            self._full_charge_undocking = True
            self._last_full_charge_undock_attempt_time = now

        logging.info(f"[满电离桩] 当前电量 {current_battery}%，发送离桩直控目标。")
        result = self.w1.w1_manager.move_chassis_ctrl([0.3, 0, 0])
        if getattr(result, "name", None) != "SUCCESS":
            with self._chassis_state_lock:
                self._full_charge_undocking = False
            logging.error("[满电离桩] 离桩直控目标发送失败，不导航到接待处。")
            return

        undock_success = self.wait_for_navigation_arrival(
            "ctrl", timeout=15.0, cancel_on_timeout=True)
        with self._chassis_state_lock:
            self._full_charge_undocking = False
            if not undock_success:
                logging.error("[满电离桩] 离桩未成功，不导航到接待处。")
                return
            self.is_charging_cycle = False
            self.is_battery_charging = False
            self.is_charger_connected = False
            self.is_charging_current_detected = False
            self._charging_cycle_confirmed_time = None
            self._charger_disconnected_stable_count = 0

        nav_result = self.w1.w1_manager.move_chassis_nav2([0.8, 0.41, 0])
        if getattr(nav_result, "name", None) == "SUCCESS":
            logging.info("[满电离桩] 离桩成功，已发送接待处导航目标。")
        else:
            logging.error("[满电离桩] 离桩成功，但接待处导航目标发送失败。")

    def low_battery_go_home(self, battery_info: str):
        try:
            current_battery = int(
                float(battery_info.split("电量：", 1)[1].split("\n", 1)[0].strip()))
        except (IndexError, ValueError):
            logging.error("电量数据解析异常")
            return

        # 已知底盘会偶发上报 0%；该值不参与回充/离桩决策。
        if current_battery <= 0:
            logging.warning(f"[电量异常] 忽略 {current_battery}% 电量上报。")
            return

        if current_battery != self.last_checked_battery:
            self.last_checked_battery = current_battery
            logging.info(f"[当前电量] {current_battery}%")

        # 决策不能只依赖“电量变化”：例如一次回充请求发送失败后，电量仍为
        # 30% 时也必须允许后续定时器在冷却结束后重试。
        if current_battery <= 30:
            with self._chassis_state_lock:
                self.pending_auto_charging = True
                if self.is_force_working:
                    logging.warning(
                        f"[低电量回充] 当前电量 {current_battery}%，"
                        "任务结束后自动回充。")
                    return

            if (not hasattr(self, "w1") or self.w1 is None or
                    getattr(self.w1, "w1_manager", None) is None):
                logging.info(
                    f"[低电量回充] 当前电量 {current_battery}%，"
                    "机器人尚未初始化，已记录待回充状态。")
                return

            self._request_auto_charging(f"当前电量 {current_battery}%")
        elif current_battery >= 90:
            if (not hasattr(self, "w1") or self.w1 is None or
                    getattr(self.w1, "w1_manager", None) is None):
                logging.info(
                    f"[满电离桩] 当前电量 {current_battery}%，"
                    "机器人尚未初始化，暂不执行离桩。")
                return
            self._undock_after_full_charge(current_battery)

    def update_charging_state(self, battery_charging: bool,
                              charger_connected: bool,
                              charging_current_detected: bool,
                              observed_time: float):
        """保存充电状态，连续确认离桩后清除过期的充电周期标记。"""
        with self._charging_state_condition:
            # 回充等待会阻塞界面线程，期间的诊断信号可能在事件队列中积压。
            # 旧样本不能用于判断刚成功的回充周期是否已经离桩。
            if (self._charging_cycle_confirmed_time is not None and
                    observed_time < self._charging_cycle_confirmed_time):
                return

            state_changed = (
                not self._has_charging_state_observation or
                self.is_battery_charging != battery_charging or
                self.is_charger_connected != charger_connected or
                self.is_charging_current_detected != charging_current_detected)
            self.is_battery_charging = battery_charging
            self.is_charger_connected = charger_connected
            self.is_charging_current_detected = charging_current_detected
            self._has_charging_state_observation = True
            self._last_charging_state_observed_time = observed_time
            # 任务入口可能正在等待首帧/新一帧电源板诊断。
            self._charging_state_condition.notify_all()

            if state_changed:
                logging.info(
                    "[充电物理状态] "
                    f"正在充电={battery_charging}，"
                    f"充电器连接={charger_connected}，"
                    f"检测到正向充电电流={charging_current_detected}。")

            if (battery_charging or charger_connected or
                    charging_current_detected or
                    self._auto_charging_requesting):
                self._charger_disconnected_stable_count = 0
                if (battery_charging or charger_connected or
                        charging_current_detected):
                    self.pending_auto_charging = False
                return

            if not self.is_charging_cycle:
                self._charger_disconnected_stable_count = 0
                return

            self._charger_disconnected_stable_count += 1
            stable_count = self._charger_disconnected_stable_count
            stable_threshold = self._charger_disconnected_stable_threshold
            logging.info(
                "[充电周期状态校验] 连续检测到未充电且未连接充电器："
                f"{stable_count}/{stable_threshold}。")

            if stable_count < stable_threshold:
                return

            self.is_charging_cycle = False
            self._charging_cycle_confirmed_time = None
            self._charger_disconnected_stable_count = 0
            logging.warning(
                "[充电周期状态校验] 已连续确认机器人未充电且未连接充电器，"
                "清除过期的 is_charging_cycle 状态。")

    # -------------------------------------------------- 路点导航 --------------------------------------------------
    def start_undocking_emit(self, x, y, w):
        """
        统一发射离桩导航信号
        """
        try:
            with _global_lock:
                result = self.w1.w1_manager.move_chassis_ctrl(
                    [float(x), float(y), float(w)])
            if getattr(result, "name", None) != "SUCCESS":
                logging.error("离桩直控目标发送失败")
                return False
            return self.wait_for_navigation_arrival("ctrl")

        except Exception as e:
            logging.error(f"离桩信号转换或发送异常: {str(e)}")
            return False

    def get_current_point(self):
        try:
            res, chassis_pose = self.w1.w1_manager.get_current_chassis_pose()
            return np.round(chassis_pose, 2).tolist()
            
        except Exception as e: 
            logging.error(f"当前点位获取异常: {str(e)}")
            return [0.0,0.0,0.0]

    def wait_for_navigation_arrival(self, status, timeout=600.0,
                                    cancel_on_timeout=True):
        """
        等待导航或底盘直控 Action 进入最终状态。

        W1Manager 状态查询接口返回 ``(StatusCode, Action状态)``；这里只判断
        第二项。仅 SUCCEEDED 返回 True，失败、取消、接口不可用或超时返回 False。
        """
        start_time = time.monotonic()
        action_name = "导航" if status == "nav" else "底盘直控"

        while True:
            # 1. 获取当前控制状态
            if status == "nav":
                with _global_lock:
                    current_status = self.w1.w1_manager.get_nav_status()
            elif status == "ctrl":
                with _global_lock:
                    current_status = self.w1.w1_manager.get_control_status()
            else:
                logging.error(f"未知的底盘等待类型: {status}")
                return False

            # 状态查询失败时，第二项为 DISABLED 或 NOT_INITIALIZED。
            action_status = current_status[1]
            # 2. 检查是否成功到达
            if action_status == "SUCCEEDED":
                logging.info(f"[{action_name}] 已成功完成。")
                return True
                
            # 3. 检查是否进入了失败或取消的状态
            if action_status in {
                    "FAILED", "CANCELLED", "ABORTED", "DISABLED",
                    "NOT_INITIALIZED"
            }:
                logging.error(f"[{action_name}] 未完成，状态={action_status}。")
                return False

            # 4. 超时检查
            if time.monotonic() - start_time > timeout:
                logging.error(f"[{action_name}] 超时（{timeout:.1f}s）。")
                if cancel_on_timeout:
                    if status == "ctrl":
                        self.w1.w1_manager.cancel_chassis_control()
                    else:
                        self.w1.w1_manager.cancel_chassis_motion()
                return False

            # 5. 频率控制
            time.sleep(0.1)

        return False

    def smart_chess(self, x, y, w, wait=True):
        try:
            with _global_lock:
                result = self.w1.w1_manager.move_chassis_nav2([x, y, w])
            if getattr(result, "name", None) != "SUCCESS":
                logging.error("导航目标发送失败")
                return False
            return self.wait_for_navigation_arrival("nav") if wait else True
        except Exception as e:
            logging.error(f"底盘运动出错: {str(e)}")
            return False
    # -------------------------------------------------- 路网导航 --------------------------------------------------


    def face_track(self):
        if "face_tracking" in self.device_cfg['ui_param'] and self.device_cfg['ui_param']['face_tracking']==True:
            if (hasattr(self, 'picklight_vision') and self.picklight_vision and self.robot_app and
                hasattr(self, 'vision_logger') and self.vision_logger):
                self.face_tracking_thread = FaceTrackingThread(
                    self.picklight_vision,
                    self.robot_app.w1_manager,
                    self.vision_logger,
                    self
                )
                self.face_tracking_thread.finished_signal.connect(self._on_face_tracking_finished)
                self.face_tracking_thread.start()
                # logging.info("正在后台启动人脸跟踪...")

    def _on_face_tracking_finished(self, success, message):
        """人脸跟踪线程完成回调"""
        logging.info(message)
        self._write_status_file('face_tracking',False)
        if not success:
            # 可以选择重试或记录错误
            logging.info("人脸跟踪将在稍后重试")

    def on_config_read(self):  # 界面读取配置"
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            target_dir = os.path.normpath(
                os.path.join(current_dir, "../application/parameters")
            )
            if not os.path.exists(target_dir):
                os.makedirs(target_dir)
            file_path, _ = QFileDialog.getOpenFileName(
                None, "选择配置文件", target_dir, "JSON Files (device.json)"
            )
            if not file_path:
                logging.info("未选择配置文件")
                self.update_light(self.G3, False)
                return

            self._load_config_file(file_path, is_first_load=True)
        except Exception as e:
            logging.info(f"配置文件读取错误: {str(e)}")
            self.update_light(self.G3, False)

    def on_clear_log(self):  # 清除日志内容
        self.syslog.clear()

    def on_save_log(self):  # 保存日志
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            file_path, _ = QFileDialog.getSaveFileName(
                self.main_window,
                "保存日志文件",
                os.path.join(current_dir, "w1_log.txt"),
                "Text Files (*.txt)",
            )

            if file_path:
                with open(file_path, "w", encoding="utf-8") as f:
                    f.write(self.syslog.toPlainText())
                logging.info(f"日志已保存到: {file_path}")
        except Exception as e:
            logging.info(f"保存日志失败: {str(e)}")

    def on_log_level_changed(self, index):  # 日志级别过滤变更事件
        level = self.log_level_combo.currentText()
        # 这里需要实现实际的日志过滤逻辑
        logging.info(f"日志级别设置为: {level}")

        # 示例过滤实现 (需要根据实际日志系统调整)
        if level == "DEBUG":
            self.logger.setLevel(logging.DEBUG)
        elif level == "INFO":
            self.logger.setLevel(logging.INFO)
        elif level == "WARNING":
            self.logger.setLevel(logging.WARNING)
        elif level == "ERROR":
            self.logger.setLevel(logging.ERROR)
        else:  # CRITICAL
            self.logger.setLevel(logging.CRITICAL)

    def _write_status_file(self, name, status):  # 写入状态到JSON文件
        try:
            status_file = self.device_cfg["config_path"]["auto_config_path"]
            os.makedirs(os.path.dirname(status_file), exist_ok=True)
            status_data = {}
            if os.path.exists(status_file):
                try:
                    with open(status_file, "r", encoding="utf-8") as f:
                        status_data = json.load(f)
                    status_data[name] = status
                    with open(status_file, "w", encoding="utf-8") as f:
                        json.dump(status_data, f, indent=2, ensure_ascii=False)
                except:
                    time.sleep(0.5)
                    with open(status_file, "r", encoding="utf-8") as f:
                        status_data = json.load(f)
                    status_data[name] = status
                    with open(status_file, "w", encoding="utf-8") as f:
                        json.dump(status_data, f, indent=2, ensure_ascii=False)
            time.sleep(0.1)
        # except Exception as e:
        #     print(f"写入状态文件失败: {e}")
        #测试
        except Exception as e:
            import traceback
            print(f"[W1Controller._write_status_file] 写入失败: name={name}, status={status}, error={e}")
            traceback.print_exc()
        #测试

    def _read_status_file(self):  # 读取auto_config.json信息
        """读取auto_config.json信息"""
        try:
            status_file = self.device_cfg["config_path"]["auto_config_path"]
            os.makedirs(os.path.dirname(status_file), exist_ok=True)
            status_data = {}
            if os.path.exists(status_file):
                try:
                    with open(status_file, "r", encoding="utf-8") as f:
                        status_data = json.load(f)
                except:
                    status_data = {}
            time.sleep(0.1)
            return status_data
        # except Exception as e:
        #     print(f"写入状态文件失败: {e}")
        #测试
        except Exception as e:
            import traceback
            print(f"[W1Controller._read_status_file] 读取失败: error={e}")
            traceback.print_exc()
        #测试

    def on_timestamp_display_changed(self, state):  # 日志时间戳显示设置改变
        """时间戳显示设置改变"""
        enabled = state == Qt.CheckState.Checked.value
        # 添加一条日志来显示状态变化, 这条日志本身也会根据当前设置决定是否带时间戳
        logging.info(f"时间戳显示 {'启用' if enabled else '禁用'}")

        # 如果需要立即刷新现有日志的时间戳显示, 可以添加以下代码
        current_log = self.syslog.toPlainText()
        self.syslog.clear()
        if enabled:
            # 这里简单实现, 实际可能需要更复杂的解析来保持原有格式
            for line in current_log.split("\n"):
                if line:  # 非空行
                    logging.info(line)
        else:
            # 移除时间戳 (假设时间戳格式为 YYYY-MM-DD HH:MM:SS - )
            for line in current_log.split("\n"):
                if line:  # 非空行
                    # 尝试移除时间戳部分
                    if re.match(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2} - ", line):
                        self.syslog.append(line[20:])  # 移除前20个字符(时间戳部分)
                    else:
                        self.syslog.append(line)

    def show_password_dialog(self):  # 显示密码对话框
        """显示密码对话框"""
        dialog, password_input, submit_btn = self.create_password_dialog(
            self.main_window
        )

        def verify_password():
            """验证密码"""
            if password_input.text() == "123456":  # 示例密码
                dialog.accept()
            else:
                styles = self.get_password_dialog_styles()
                password_input.setStyleSheet(styles["input_error_style"])
                QMessageBox.warning(dialog, "错误", "密码不正确！")
                password_input.clear()
                password_input.setStyleSheet(styles["input_style"])
                password_input.setFocus()

        password_input.returnPressed.connect(verify_password)
        submit_btn.clicked.connect(verify_password)
        return dialog.exec() == QDialog.Accepted

    def kill_voiceServer(self):
        if self.voice_process and self.voice_process.poll() is None:
            logging.info("正在终止语音服务...")
            self.voice_process.terminate() 
            try:
                self.voice_process.wait(timeout=5) 
            except subprocess.TimeoutExpired:
                logging.info("语音服务未响应，强制终止...")
                self.voice_process.kill()
            self.voice_process = None
            
        self.kill_w1_voice_server_processes()

    def kill_w1_voice_server_processes(self):
        import psutil
        killed_pids = []
        for proc in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmdline = proc.info['cmdline']
                if not cmdline:
                    continue
                cmd_str = ' '.join(cmdline)
                if 'w1_voice_server.py' in cmd_str:
                    if any('python' in part.lower() for part in cmdline[:2]):
                        print(f"发现残留进程 PID={proc.pid}, cmdline: {cmdline}")
                        proc.terminate() 
                        killed_pids.append(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        gone, alive = psutil.wait_procs(
            [psutil.Process(pid) for pid in killed_pids if psutil.pid_exists(pid)],
            timeout=5
        )
        for p in alive:
            try:
                print(f"强制杀死未响应进程 PID={p.pid}")
                p.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
    
    def kill_python_processes(self, process_name):
        import psutil
        killed_pids = []
        for proc in psutil.process_iter(['pid', 'cmdline']):
            try:
                cmdline = proc.info['cmdline']
                if not cmdline:
                    continue
                cmd_str = ' '.join(cmdline)
                if process_name in cmd_str:
                    if any('python' in part.lower() for part in cmdline[:2]):
                        print(f"发现残留进程222 PID={proc.pid}, cmdline: {cmdline}")
                        proc.terminate() 
                        killed_pids.append(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

        if killed_pids:
            gone, alive = psutil.wait_procs(
                [psutil.Process(pid) for pid in killed_pids if psutil.pid_exists(pid)],
                timeout=5
            )
            for p in alive:
                try:
                    print(f"强制杀死未响应进程 PID={p.pid}")
                    p.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        return killed_pids

    def _cleanup_yolo_models(self):
        """清理YOLO模型占用的显存"""
        try:
            print("开始清理YOLO模型显存...")
            
            # 1. 清理Function类中的YOLO模型
            if hasattr(self, 'function') and self.function:
                if hasattr(self.function, 'yolo_models') and self.function.yolo_models:
                    print(f"清理Function中的YOLO模型，数量: {len(self.function.yolo_models)}")
                    
                    # 遍历所有模型并释放
                    for model_path, model in list(self.function.yolo_models.items()):
                        try:
                            # 从字典中移除引用
                            del self.function.yolo_models[model_path]
                            print(f"已移除YOLO模型: {model_path}")
                        except Exception as e:
                            print(f"移除模型 {model_path} 失败: {e}")
                    
                    # 清空字典
                    self.function.yolo_models.clear()
            
            # 2. 强制清理PyTorch显存
            try:
                import torch
                if torch.cuda.is_available():
                    print("清理PyTorch GPU显存...")
                    torch.cuda.empty_cache()
                    # 清理所有CUDA缓存
                    for i in range(torch.cuda.device_count()):
                        torch.cuda.set_device(i)
                        torch.cuda.empty_cache()
                        torch.cuda.ipc_collect()
                    print("PyTorch GPU显存清理完成")
            except ImportError:
                print("PyTorch未安装，跳过GPU显存清理")
            except Exception as e:
                print(f"清理PyTorch显存失败: {e}")
            
            # 3. 清理picklight_vision中的模型
            if hasattr(self, 'picklight_vision') and self.picklight_vision:
                try:
                    # 如果有释放模型的方法，调用它
                    if hasattr(self.picklight_vision, 'cleanup_models'):
                        self.picklight_vision.cleanup_models()
                        print("picklight_vision模型清理完成")
                    elif hasattr(self.picklight_vision, 'release_models'):
                        self.picklight_vision.release_models()
                        print("picklight_vision模型释放完成")
                except Exception as e:
                    print(f"清理picklight_vision模型失败: {e}")
            
            # 4. 清理vision_app中的模型
            if hasattr(self, 'vision_app') and self.vision_app:
                try:
                    # 如果有释放模型的方法，调用它
                    if hasattr(self.vision_app, 'cleanup_models'):
                        self.vision_app.cleanup_models()
                        print("vision_app模型清理完成")
                    elif hasattr(self.vision_app, 'release_models'):
                        self.vision_app.release_models()
                        print("vision_app模型释放完成")
                except Exception as e:
                    print(f"清理vision_app模型失败: {e}")
            
            # 5. 强制垃圾回收
            import gc
            gc.collect()
            print("垃圾回收完成")
            
            # 6. 清理其他可能的YOLO模型实例
            try:
                # 清理ultralytics相关的模型缓存
                import sys
                yolo_modules = [name for name in sys.modules if 'ultralytics' in name.lower()]
                for module_name in yolo_modules:
                    try:
                        del sys.modules[module_name]
                    except:
                        pass
                print(f"清理了 {len(yolo_modules)} 个ultralytics相关模块")
            except Exception as e:
                print(f"清理ultralytics模块失败: {e}")
            
            print("YOLO模型显存清理完成")
            
        except Exception as e:
            print(f"清理YOLO模型失败: {e}")


    def closeEvent(self, event):
        """增强的关闭事件处理"""
        try:
            print("开始执行完整清理流程...")
            if hasattr(self, 'service_bash_process') and self.service_bash_process:
                try:
                    print("正在向后台服务脚本发送停止信号...")
                    # 发送 SIGTERM，这会触发 Bash 脚本中的 cleanup trap
                    self.service_bash_process.terminate() 
                    # 给予3秒钟时间让其优雅关闭里面的 Docker 进程
                    self.service_bash_process.wait(timeout=3)
                except Exception as e:
                    # 如果超时未响应，则强制杀掉
                    print(f"安全关闭服务脚本超时或异常，强制终止: {e}")
                    self.service_bash_process.kill()

            # 录制   停止 dexe_recorder 守护进程
            if hasattr(self, 'recorder_process') and self.recorder_process:
                try:
                    print("正在停止 dexe_recorder 录制守护进程...")
                    self.recorder_process.terminate()
                    self.recorder_process.wait(timeout=3)
                except Exception as e:
                    print(f"停止录制守护进程超时，强制终止: {e}")
                    self.recorder_process.kill()
                self.recorder_process = None

            # 彻底杀死 ACT 相关进程和释放资源
            if hasattr(self, 'function') and self.function:
                # 调用强制停止方法 (Hard Stop)
                self.function._force_stop_act_control("程序关闭")
                self.function.cleanup()
            
            # 清理 Policy 启动线程
            if hasattr(self, 'policy_control_thread') and self.policy_control_thread:
                if self.policy_control_thread.isRunning():
                    self.policy_control_thread.quit()
                    self.policy_control_thread.wait()
                self.policy_control_thread = None

            self.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "w1_ui", "false")
            if hasattr(self, 'function'):
                self.function.cleanup()
                
            # 1. 停止人脸跟踪线程
            if hasattr(self, 'face_tracking_thread') and self.face_tracking_thread:
                try:
                    # 停止人脸检测
                    if hasattr(self, 'picklight_vision') and self.picklight_vision:
                        self.picklight_vision.face_tracking_status = False
                    
                    # 停止线程
                    if self.face_tracking_thread.isRunning():
                        self.face_tracking_thread.quit()
                        if not self.face_tracking_thread.wait(1000):  # 等待1秒
                            self.face_tracking_thread.terminate()
                            self.face_tracking_thread.wait()
                    
                    self.face_tracking_thread = None
                    print("人脸跟踪线程已停止")
                    
                    # 更新状态文件
                    self._write_status_file('face_tracking', False)
                    
                except Exception as e:
                    print(f"停止人脸跟踪线程失败: {e}")
            
            # 2. 停止所有定时器
            if hasattr(self, 'ui_timer') and self.ui_timer:
                self.ui_timer.stop()
            if hasattr(self, 'heartbeat_timer') and self.heartbeat_timer:
                self.heartbeat_timer.stop()
            if (hasattr(self, 'auto_charging_retry_timer') and
                    self.auto_charging_retry_timer):
                self.auto_charging_retry_timer.stop()
            if hasattr(self, 'capsule_timer') and self.capsule_timer:
                self.capsule_timer.stop()

            #录制
            if hasattr(self, 'recorder_refresh_timer') and self.recorder_refresh_timer:
                self.recorder_refresh_timer.stop()

            # 停止支付服务
            self.stop_payment_service()
            
            # 3. 停止新语音服务
            if self.voice_service_new_window:
                try:
                    self.voice_service_new_window.closeEvent()
                    self.voice_service_new_window = None
                except Exception as e:
                    print(f"关闭语音服务窗口失败: {e}")
            
            # 4. 强制终止所有Python进程
            self.kill_all_python_processes()
            
            # 5. 停止状态监控线程
            # self.w1manage_status_monitor("stop")
            if hasattr(self, 'status_thread') and self.status_thread:
                self.status_thread.stop_monitoring()
                self.status_thread.quit()
            if hasattr(self, 'ros_battery_thread') and self.ros_battery_thread:
                self.ros_battery_thread.stop_monitoring()
                self.ros_battery_thread.quit()
            # 6. 停止HTTP服务器
            self._stop_http_server()
            
            # 7. 停止Flask服务器
            if hasattr(self, 'flask_qt_thread'):
                try:
                    self.flask_qt_thread.stop_server()
                except Exception as e:
                    print(f"停止Flask服务器失败: {e}")
            
            # 8. 停止语音监控
            if self.voice_monitor:
                try:
                    self.voice_monitor.stop_continuous_check()
                    self.voice_monitor.stop_monitoring()
                except Exception as e:
                    print(f"停止语音监控失败: {e}")
            
            # 9. 释放视觉资源
            self._release_vision_resources()
            
            # 10. 清理活动线程
            self._cleanup_all_threads()
            
            # 11. 更新配置文件状态
            self.update_json_value("/home/dexforce/workspace/dexe_application/qt/auto_config.json", "w1_ui", "false")
            
            print("清理完成，准备退出...")
            
        except Exception as e:
            print(f"关闭过程中发生错误: {e}")
        finally:
            # 确保进程被杀死
            self.force_kill_related_processes()
            super().closeEvent(event) if event else None

    def _release_vision_resources(self):
        """释放视觉资源"""
        try:
            if hasattr(self, 'vision_app') and self.vision_app:
                # 断开视觉连接
                self.vision_app.dis_connection()
                
            if hasattr(self, 'picklight_vision') and self.picklight_vision:
                # 如果有释放资源的方法，调用它
                if hasattr(self.picklight_vision, 'release'):
                    self.picklight_vision.release()
            
            # 强制清理GPU显存
            self.force_clear_gpu_memory()
            
        except Exception as e:
            print(f"释放视觉资源失败: {e}")

    def force_clear_gpu_memory(self):
        """强制清理GPU显存"""
        try:
            import subprocess
            # 清理PyTorch显存（如果使用）
            subprocess.run(['python3', '-c', 
                        'import torch; torch.cuda.empty_cache() if torch.cuda.is_available() else None'],
                        timeout=5)
        except:
            pass

    def kill_all_python_processes(self):
        """杀死所有相关的Python进程"""
        processes_to_kill = [
            "w1_controller.py",
            "w1_voice_server.py", 
            "dexe_agent_start.py",
            "pickwiz_start.py",
            "detect_roi_topic.py",
            "pointcloud_browser.py",
            #录制
            "dexe_recorder_node"
        ]
        
        for process_name in processes_to_kill:
            self.kill_python_processes(process_name)

    def _cleanup_all_threads(self):
        """清理所有线程"""
        # 清理运动线程
        if hasattr(self, "active_move_threads"):
            for thread in self.active_move_threads:
                if thread.isRunning():
                    thread.terminate()
                    thread.wait(500)
            self.active_move_threads.clear()
        
        # 清理其他活动线程
        if hasattr(self, "active_threads"):
            for thread in self.active_threads:
                if thread.isRunning():
                    thread.quit()
                    thread.wait(1000)
        
        # 清理异步线程
        if hasattr(self, "async_thread") and self.async_thread:
            self.async_thread.stop()

    def force_kill_related_processes(self):
        """强制杀死相关进程"""
        try:
            import psutil
            import signal
            
            current_pid = os.getpid()
            current_process = psutil.Process(current_pid)
            
            # 获取所有子进程
            children = current_process.children(recursive=True)
            for child in children:
                try:
                    child.send_signal(signal.SIGTERM)
                    child.wait(timeout=3)
                except (psutil.NoSuchProcess, psutil.TimeoutExpired):
                    try:
                        child.kill()
                    except:
                        pass
            
            # 额外杀死可能残留的进程
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if cmdline and any('python' in str(part).lower() for part in cmdline):
                        # 检查是否与我们的应用相关
                        if any(keyword in str(cmdline) for keyword in 
                            ['w1_controller', 'dexe_agent', 'pickwiz', 'vision']):
                            if proc.info['pid'] != current_pid:
                                try:
                                    proc.terminate()
                                    proc.wait(timeout=2)
                                except:
                                    proc.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                    
        except Exception as e:
            print(f"强制杀死进程失败: {e}")


    ## ———————————W1相关功能————————————————
    def safe_shutdown(self):  # 下电
        import platform

        system = platform.system()
        try:
            if system == "Windows":
                subprocess.run(["shutdown", "/s", "/t", "0"], check=True)

            elif system == "Linux":
                if os.geteuid() == 0:
                    os.system("poweroff")
                else:
                    os.system("echo '123456' | sudo -S poweroff")

            elif system == "Darwin":
                if os.geteuid() == 0:
                    os.system("shutdown -h now")
                else:
                    subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
            else:
                print(f"不支持的操作系统: {system}")
        except subprocess.CalledProcessError as e:
            print(f"关机命令执行失败: {e}")
        except Exception as e:
            print(f"发生错误: {e}")

    def set_recharge_point(self): 
        """设置回充点"""
        try: 
            command = ["ros2", "service", "call", "/save_charging_pose", "std_srvs/srv/Trigger", "{}"]
            result = subprocess.run(command, capture_output=True, text=True)
            if result.returncode == 0:
                output_msg = result.stdout.strip()
                if "success=True" in output_msg:
                    try:
                        coords = output_msg.split("message='")[1].split("')")[0]
                        logging.info(f"回充点保存成功！当前坐标: {coords}")
                    except IndexError:
                        logging.info("回充点保存成功！")
                else:
                    logging.info(f"回充点保存失败，服务端拒绝: {output_msg}")
            else:
                error_msg = result.stderr.strip()
                logging.info(f"指令执行失败! 错误信息: {error_msg}")
        except Exception as e: 
            logging.info(f"设置回充点执行异常: {str(e)}")

    def on_stop_execution(self,stop_gohome = None):  # 停止机器人运动和语音
        try:
            logging.info("正在执行中断操作...")
            self.w1.w1_manager.cancel_chassis_motion()
            self.picklight_vision.face_tracking_status = False
            if hasattr(self, "function") and self.function:
                self.function.stop_squence = True # 这会触发 basic_function 中的 while 循环退出
                self.function.vision_finish=False
            # 2. Policy 停止逻辑
            self.try_suspend_policy_service()
            self.policy_suspended_by_task = False
            # 事件驱动解锁：中断后 Policy 不再活跃，确保恢复Policy按钮可再次点击
            if hasattr(self, 'B_resume_policy'):
                self.B_resume_policy.setEnabled(True)

            if stop_gohome == True:
                logging.info("开始执行任务停止")
                self.function.stop_squence = True
                self.function.vision_finish=False
                self.w1.stop_squence = True
                self.w1.w1_manager.stop_move()
                return

            elif hasattr(self, "w1") and self.w1:
                if  self.picklight_vision:
                    self.picklight_vision.face_tracking_status = False
                self.function.stop_squence = True
                self.w1.stop_squence = True
                self.function.vision_finish=False
                self.w1.w1_manager.stop_move()
                if hasattr(self, "function") and self.function:
                    self.execute_voice_command(self.parameter_cfg["stop_btn"])

                

            self._write_status_file("status", "idle")
            self.update_light(self.G4, False)
            logging.info("已停止所有机器人运动和任务执行")


        except Exception as e:
            logging.info(f"停止执行时出错: {str(e)}")
            self.update_light(self.G4, False)

    def _abort_task_preparation(self, task_request_serial: int,
                                policy_need_resume: bool,
                                reason: str) -> None:
        """任务线程创建前失败时，释放占用并恢复本次请求挂起的 Policy。"""
        with self._chassis_state_lock:
            if self._task_request_serial != task_request_serial:
                return
            self.is_force_working = False

        logging.warning(
            f"[任务准备] 请求序号={task_request_serial} 已取消，原因={reason}。")
        if policy_need_resume:
            logging.info("[任务准备] 本次任务未启动，恢复此前挂起的 Policy。")
            self.try_resume_policy_for_task(task_request_serial)
            
    def on_execute(
        # self, voice_msg="", module_name="", interface_trigger=False ,payment_trigger=False  # 新增参数，标识是否由支付流程触
        # ):  # 任务执行函数
        self, voice_msg="", module_name="", interface_trigger=False , task_list=[], is_new_ui=False): 
        """voice_msg: '语音识别文本', module_name: "执行模块名",若不填默认"""
        self.function.vision_finish=False
        self.picklight_vision.face_tracking_status = False

        # 语音任务必须先确认可接收，不能在最终判定“忙碌”之前就执行退桩。
        current_status = self._read_status_file()
        if not interface_trigger and current_status["status"] != "idle":
            logging.warning(f"已在任务状态:{module_name}!禁止触发")
            return

        with self._chassis_state_lock:
            self._task_request_serial += 1
            task_request_serial = self._task_request_serial

        # 精准退桩本身已经属于新任务的底盘运动，必须在它之前挂起 Policy。
        # 返回值表示新任务结束后是否需要负责恢复 Policy。
        policy_need_resume = self.try_suspend_policy_service()
        logging.info(
            f"[任务准备] 请求序号={task_request_serial}，"
            f"Policy恢复责任={policy_need_resume}。")

        # if self.is_charging_cycle:
        #     self.w1.w1_manager.move_chassis_ctrl([0.3,0,0])
        #     self.is_charging_cycle = False
        #     self.is_force_working = True

        # 退桩会直接驱动底盘，只允许实时物理状态触发。
        # is_charging_cycle 只是“曾进入回充流程”的软件标志；回充被中断、
        # 外部手动离桩后它可能短时间残留，绝不能单独授权退桩运动。
        with self._charging_state_condition:
            charging_cycle = self.is_charging_cycle
            battery_charging = self.is_battery_charging
            charger_connected = self.is_charger_connected
            charging_current_detected = self.is_charging_current_detected
            last_observed_time = self._last_charging_state_observed_time
            charging_state_fresh = (
                last_observed_time is not None and
                time.monotonic() - last_observed_time <=
                self._charging_state_freshness_timeout)
            physical_dock_state = (
                battery_charging or charger_connected or
                charging_current_detected)

            # 刚启动时 PowerBoard 首帧诊断可能晚于自动回充结果。此时等待新状态，
            # 既不能直接取消正常任务，也不能凭流程标志盲目退桩。
            if not charging_state_fresh:
                wait_timeout = 20.0
                wait_deadline = time.monotonic() + wait_timeout
                logging.warning(
                    "[任务前离桩] 回充状态可能有效，但电源板物理状态尚未更新，"
                    f"最多等待 {wait_timeout:.0f} 秒确认。")
                while not charging_state_fresh:
                    remaining = wait_deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._charging_state_condition.wait(timeout=remaining)
                    charging_cycle = self.is_charging_cycle
                    battery_charging = self.is_battery_charging
                    charger_connected = self.is_charger_connected
                    charging_current_detected = (
                        self.is_charging_current_detected)
                    last_observed_time = self._last_charging_state_observed_time
                    charging_state_fresh = (
                        last_observed_time is not None and
                        time.monotonic() - last_observed_time <=
                        self._charging_state_freshness_timeout)
                    physical_dock_state = (
                        battery_charging or charger_connected or
                        charging_current_detected)

            need_undock = charging_state_fresh and physical_dock_state
            if need_undock:
                self.is_force_working = True

        if not charging_state_fresh:
            logging.error(
                "[任务前离桩] 等待 20 秒仍未收到新的充电物理状态；为避免"
                "盲目退桩，取消本次任务。请检查 /diagnostics 电源板状态。")
            self._abort_task_preparation(
                task_request_serial, policy_need_resume, "充电物理状态超时")
            return

        if need_undock:
            logging.info(
                "[任务前离桩] 检测到机器人可能仍在充电桩上："
                f"充电周期={charging_cycle}，正在充电={battery_charging}，"
                f"充电器连接={charger_connected}，"
                f"正向充电电流={charging_current_detected}；先执行精准退桩。")
            control_result = self.w1.w1_manager.move_chassis_ctrl([0.3, 0, 0])
            if getattr(control_result, "name", None) != "SUCCESS":
                with self._chassis_state_lock:
                    self.is_force_working = False
                logging.warning("离开充电桩运动控制目标发送失败，取消本次任务执行")
                self._abort_task_preparation(
                    task_request_serial, policy_need_resume, "退桩目标发送失败")
                return

            start_time = time.time()
            timeout = 15

            while time.time() - start_time < timeout:
                time.sleep(0.1)
                _, control_status = self.w1.w1_manager.get_control_status()
                if control_status == "SUCCEEDED":
                    with self._chassis_state_lock:
                        self.is_charging_cycle = False
                        self.is_battery_charging = False
                        self.is_charger_connected = False
                        self.is_charging_current_detected = False
                        self._charging_cycle_confirmed_time = None
                        self._charger_disconnected_stable_count = 0
                        self.is_force_working = True
                    logging.info("机器人已离开充电桩，开始执行任务")
                    break
                if control_status in {"FAILED", "CANCELLED", "ABORTED"}:
                    logging.error(f"离开充电桩失败，状态={control_status}，取消本次任务执行")
                    with self._chassis_state_lock:
                        self.is_force_working = False
                    self._abort_task_preparation(
                        task_request_serial, policy_need_resume,
                        f"退桩执行失败：{control_status}")
                    return
            else:
                logging.warning(f"离开充电桩超时，超过 {timeout}s 未完成，取消本次任务执行")
                with self._chassis_state_lock:
                    self.is_force_working = False
                self._abort_task_preparation(
                    task_request_serial, policy_need_resume, "退桩执行超时")
                return
        elif charging_cycle:
            logging.warning(
                "[任务前离桩] 仅检测到过期的充电周期标志，物理状态为未充电且"
                "未连接充电器；为避免误退桩，直接执行本次任务。")

        self._refresh_battery_snapshot("任务开始前")

        if self.payment_manager != None and self.is_payment and module_name=="泡咖啡":
                if self.payment_manager.order_cache.__len__() == 0:
                    self.execute_voice_command(self.parameter_cfg["no_pay"])
                    self._abort_task_preparation(
                        task_request_serial, policy_need_resume, "没有可执行订单")
                    return
                current_order = self.payment_manager.order_cache.current_order
                self.payment_manager.order_cache.remove_order(current_order.order_sn)
                self.payment_manager.order_cache.set_next_as_current()

        # self.function.stop_squence = False
        self._write_status_file("status", "running")
        
       
        if interface_trigger == True:  # 界面触发
            if  hasattr(self, 'picklight_vision') and self.picklight_vision!=None and self.picklight_vision.face_tracking_status == True:
                self.picklight_vision.face_tracking_status = False
            # self._write_status_file("status", "running")
            if not is_new_ui:
                task_msg = self.select_module.currentText()
                module_name = task_msg.split(": ")[1] if ": " in task_msg else task_msg
            get_config_by_name = module_name
            if task_list!=[]:
                get_config_by_name = task_list[0]
            exclusive_params = self.get_isolated_task_config(get_config_by_name)
            self.thread = ExecuteTaskThread(
                w1_controller=self, voice_msg=voice_msg,
                module_name=module_name, task_config=exclusive_params,
                policy_need_resume=policy_need_resume,
                task_request_serial=task_request_serial)
            self.active_threads.append(self.thread)
            #为了避免执行任务时，回去充电
            self.is_force_working = True
            self.thread.start()


        elif current_status["status"] == "idle" : # 语音触发
            if hasattr(self, 'picklight_vision') and self.picklight_vision!=None and self.picklight_vision.face_tracking_status == True:
                self.picklight_vision.face_tracking_status = False
                while current_status['face_tracking']==True:
                    current_status = self._read_status_file()
                    logging.info(f"人脸跟踪状态为：{current_status['face_tracking']},关闭中...")
                    time.sleep(0.2)
            # if module_name=="说话手势":
            #     self._write_status_file("status", "talking")
            # else:
            #     self._write_status_file("status", "running")
            self._write_status_file("statusCoffee","origin")
            get_config_by_name = module_name
            if task_list!=[]:
                get_config_by_name = task_list[0]
            exclusive_params = self.get_isolated_task_config(get_config_by_name)
            self.thread = ExecuteTaskThread(
                w1_controller=self, voice_msg=voice_msg,
                module_name=module_name, task_config=exclusive_params,
                policy_need_resume=policy_need_resume,
                task_request_serial=task_request_serial)
            self.active_threads.append(self.thread)
            #为了避免执行任务时，回去充电
            self.is_force_working = True
            self.thread.start()
            

        else:
            logging.warning(f"已在任务状态:{module_name}!禁止触发")
            self._abort_task_preparation(
                task_request_serial, policy_need_resume, "任务状态已变化")


    def w1manage_status_monitor(self, action):  # OP及电量监控
        """
        W1_manage统一管理状态监控功能(OP监控及电量监控)
        action: 'start' - 启动监控, 'stop' - 停止监控, 'update' - 更新显示
        """
        if action == "start":
            if self.status_thread and self.status_thread.isRunning():
                self.status_thread.stop_monitoring()
                self.status_thread.quit()
                if not self.status_thread.wait(500):
                    self.status_thread.terminate()
                self.status_thread = None
            self.status_thread = StatusMonitorThread(self, self.device_cfg, is_sim=self.is_sim)
            self.status_thread.update_signal.connect(self.op_clec_update)

            if self.is_sim:
                self.status_thread.start()
                logging.info("仿真模式已启动，TCP连接监控已禁用。")
                return
                
            max_retries = 100  # 最大重试次数
            retry_interval = 1.5  # 重试间隔(秒)
            for attempt in range(max_retries):
                try:
                    self.connection_result = self.status_thread._start_client()
                    logging.info(f"连接尝试 {attempt + 1}/{max_retries}: {self.connection_result}")
                    if self.status_thread.client_connected:
                        self.status_thread.start()
                        logging.info("W1状态监控已启动")
                        break
                    else:
                        if attempt < max_retries - 1:  # 不是最后一次尝试
                            logging.info(f"连接失败，{retry_interval}秒后重试...") 
                            time.sleep(retry_interval)
                        else:
                            logging.info("无法连接到W1状态服务器，监控功能将不可用") 

                except Exception as e:
                    logging.info(f"连接尝试 {attempt + 1} 发生错误: {str(e)}")
                    if attempt < max_retries - 1:
                        time.sleep(retry_interval)

        elif action == "stop":
            if self.status_thread:
                self.status_thread.stop_monitoring()
                self.status_thread.quit()
                if not self.status_thread.wait(500):
                    self.status_thread.terminate()
                self.status_thread = None
                logging.info("W1状态监控已停止")

    def op_clec_update(self, status_text):  # 更新w1manage状态显示到UI中,带满电量语音返回
        try:
            from PySide6.QtCore import QTimer

            def safe_update():
                try:
                    if hasattr(self, "K3") and self.K3 is not None:
                        # 🚀 关键修改：直接用传入的 status_text 覆盖整个文本框的内容，不再使用 append
                        self.K3.setPlainText(status_text)
                        
                    # 下方的电量数值提取和语音播报逻辑保持不变
                    if "电量：" in status_text:
                        try:
                            # 提取电量数值
                            battery_level = int(status_text.split("：")[1].split("\n")[0])
                            
                            # 初始化计数器
                            if not hasattr(self, 'elec_voice_count'):
                                self.elec_voice_count = 0
                           
                            if self.elec_voice == True:
                                if battery_level >= self.device_cfg["w1_tcp_config"]["elec_overpressure"][0]:
                                    if hasattr(self, "function") and self.function:
                                        self.execute_voice_command(self.parameter_cfg["full_charge"])
                                        self.elec_voice_count += 1
                                        if self.elec_voice_count >= 3:
                                            self.elec_voice = False
                                            self.elec_voice_count = 0 
                                elif battery_level <= self.device_cfg["w1_tcp_config"]["elec_lowpressure"][0]:
                                    if hasattr(self, "function") and self.function:
                                        self.execute_voice_command(self.parameter_cfg["low_battery"])
                                        self.elec_voice_count += 1
                                        if self.elec_voice_count >= 3:
                                            self.elec_voice = False
                                            self.elec_voice_count = 0 
                        except Exception as e:
                            print(f"提取电量值错误: {e}")
                except Exception as e:
                    print(f"UI更新错误: {e}")

            # 确保 UI 更新在主线程安全执行
            QTimer.singleShot(0, safe_update)
            
        except Exception as e:
            print(f"更新状态显示失败: {e}")

    def on_control_event(self, control, *args):  # 拖拽,清除OP,下电,模式切换功能
        try:
            if control == self.set_init:
                # 设置初始位置的逻辑

                if self.show_password_dialog():
                    self.w1.w1_manager.set_home_qpos(self.w1.w1_manager.get_current_qpos()[1], True)
                    logging.info("初始位置设置成功")
            elif control == self.drag_open:
                # 开启拖拽模式的逻辑
                if not self.w1:
                    QMessageBox.warning(self.main_window, "警告", "请先读取配置文件")
                    return

                self.w1.w1_manager.set_drag_mode(True)
                self.update_light(self.G10, True)
                self.syslog.append("拖拽模式已开启")
            elif control == self.drag_close:
                # 关闭拖拽模式的逻辑
                if not self.w1:
                    QMessageBox.warning(self.main_window, "警告", "请先读取配置文件")
                    return

                self.w1.w1_manager.set_drag_mode(False)
                self.update_light(self.G10, False)
                self.syslog.append("拖拽模式已关闭")
            elif control == self.clear_op:
                # 清除操作错误的逻辑

                self.w1.w1_manager.clear_robot_error()
                self.syslog.append("清除OP报错完成")
            elif control == self.w1_power_off:
                # 关闭W1电源的逻辑
                self.tcp_monitoring_enabled = False
                logging.info("TCP状态监控已暂停")

                try:
                    # 弹出确认对话框询问是否关闭本机
                    dialog = self.create_tech_message_box(
                        "确认关机", "是否要求PC1关机？", self.main_window
                    )
                    result = dialog.exec()

                    if result == QDialog.Accepted:
                        message = "poweroff"
                        if hasattr(self, "client_socket") and self.client_socket:
                            self.client_socket.sendall(message.encode("utf-8"))
                            self.syslog.append(f"W1下电指令已发送")
                        else:
                            self.syslog.append("未连接到TCP服务器，无法发送消息")

                        dialog = self.create_tech_message_box(
                            "确认关机",
                            "W1下电指令已发送\n是否也要关闭本机电脑？",
                            self.main_window,
                        )
                        result = dialog.exec()

                        if result == QDialog.Accepted:
                            self.safe_shutdown()
                    return "下电指令已发送"
                except Exception as e:
                    # self.client_connected = False
                    return f"发送下电指令失败: {e}"
                finally:
                    time.sleep(0.1)
                    self.tcp_monitoring_enabled = True
                    logging.info("TCP状态监控已启用")

            elif control == self.on_mode_changed:
                # 模式切换的逻辑（确认弹窗已移至新UI前端，此处直接执行）
                if self.on_mode_changed.currentIndex() == 0:
                    return
                mode_text = self.on_mode_changed.currentText()
                try:
                    if mode_text == "自动模式":
                        message = "auto_mode"
                        log_text = f"切换到自动模式: "
                        if hasattr(self, "w1") and self.w1 and hasattr(self.w1, "w1_manager"):
                            ret = self.w1.w1_manager.set_sim_mode(False)
                            if ret == 0 or str(ret) == "StatusCode.SUCCESS":
                                self.is_sim = False
                                logging.info("本地W1Manager已切换至真实模式")
                            else:
                                logging.info(f"本地切换真实模式失败: {ret}")
                    elif mode_text == "仿真模式":
                        message = "sim_mode"
                        log_text = f"切换到仿真模式: "
                        if hasattr(self, "w1") and self.w1 and hasattr(self.w1, "w1_manager"):
                            ret = self.w1.w1_manager.set_sim_mode(True)
                            if ret == 0 or str(ret) == "StatusCode.SUCCESS":
                                self.is_sim = True
                                logging.info("本地W1Manager已切换至仿真模式")
                            else:
                                logging.info(f"本地切换仿真模式失败: {ret}")
                    elif mode_text == "遥操模式":
                        message = "tele_mode"
                        log_text = f"切换到遥操模式: "
                    elif mode_text == "当前模式":
                        message = "get_mode"
                        log_text = f"当前模式: "
                    elif mode_text == "开箱站立":
                        self.w1_stand()
                        return
                    elif mode_text == "装箱蹲下":
                        self.w1_rest()
                        return
                    else:
                        logging.info(f"未知模式类型")
                        return
                    self.client_socket = self.status_thread.client_socket
                    self.status_thread.tcp_monitoring_enabled = False
                    time.sleep(2)
                    if hasattr(self, "client_socket") and self.client_socket:
                        self.client_socket.sendall(message.encode("utf-8"))
                    else:
                        logging.info(
                            f"未连接到TCP服务器，无法发送消息"
                        )
                    data = self.client_socket.recv(
                        self.device_cfg["w1_tcp_config"]["buffer_size"]
                    ).decode("utf-8")
                    logging.info(f"{log_text}{data}")
                except Exception as e:
                    error_msg = f"指令失败: {e}"
                    logging.info(f"{error_msg}")
                finally:
                    self.status_thread.tcp_monitoring_enabled = True
            else:
                logging.info(
                    f"未知的控件触发事件: {control.objectName()}"
                )
        except Exception as e:
            logging.info(f"事件处理失败: {str(e)}")

    def w1_stand(self):  # 开箱站立
        logging.error("1111")
        # script_path = "/home/dexforce/workspace/dexe_application/tools/slowly_move_to.py"
        # json_path = "/home/dexforce/workspace/dexe_application/tools/standby.json"
        # subprocess.Popen([
        #     sys.executable, 
        #     script_path, 
        #     json_path, 
        #     "1000"  # n_frames
        # ])


    def w1_rest(self):  # 打包姿态
        script_path = "/home/dexforce/workspace/dexe_application/tools/slowly_move_to.py"
        json_path = "/home/dexforce/workspace/dexe_application/tools/rest.json"
        subprocess.Popen([
            sys.executable, 
            script_path, 
            json_path, 
            "1000"  # n_frames
        ])


    def on_chassis_get(self):  # 获取底盘状态
        try:
            res, chassis_pose = self.w1.w1_manager.get_current_chassis_pose()
            text_chassis_pose = np.round(chassis_pose, 2).tolist()
            self.T29.setText(str(text_chassis_pose))
            self.T30.setText(str(text_chassis_pose))
            logging.info(
                f"底盘状态获取成功" if res else "底盘状态获取失败"
            )
        except Exception as e:
            logging.info(f"获取底盘状态出错: {str(e)}")

    def on_move_chassis_nav2(self):  # 控制底盘运动 导航
        try:
            base_position = np.round(ast.literal_eval(self.T29.text()), 4)
            self.w1.w1_manager.move_chassis_nav2(base_position)
            logging.info(f"底盘运动到位置: {base_position}")
        except Exception as e:
            logging.info(f"底盘运动出错: {str(e)}")

    def on_move_chassis_ctrl(self):  # 底盘运动
        try:
            base_position = np.round(ast.literal_eval(self.T30.text()), 4)
            self.w1.w1_manager.move_chassis_ctrl(base_position)


            start_time = time.time()
            timeout = 20

            # 循环条件直接包含时间判断：未超时且未成功则继续
            while time.time() - start_time < timeout:
                time.sleep(0.1)
                _, control_status = self.w1.w1_manager.get_control_status()
                if control_status == "SUCCEEDED":
                    break  # 成功则跳出循环，跳过 else 块
                if control_status in {"FAILED", "CANCELLED"}:
                    break
            else:
                # 只有当循环因为条件不满足（即超时）自然结束，且没有被 break 打断时，才会执行这里
                logging.info(f"底盘控制没有读取到成功状态，超时{timeout}s,退出")


            logging.info(f"底盘运动到位置: {base_position}")
        except Exception as e:
            logging.info(f"底盘运动出错: {str(e)}")

    def stop_execute(self):  # 任务过程中,接受客户语音停止
        try:
            if self.parameter_cfg["auto_gohome"]["use"] == True:
                current_status = self._read_status_file()
                if current_status["status"] == "running" and current_status[
                    "current_module"
                ] in self.parameter_cfg.get("dont_return_gohome", []):
                    time.sleep(0.5)
                    t = current_status.get("story_time")
                    if current_status.get("story_time") == "no":
                        pass
                    elif int(current_status.get("story_time", 0)) > 60:
                        pass

                    elif 60 > int(current_status.get("story_time", 0)) > 30:
                        pass
                        
                    elif 30 > int(current_status.get("story_time", 0)):
                        pass

                elif (
                    current_status["status"] == "running"
                    and current_status["goinghome"] != True
                    and current_status["current_module"]
                    not in self.parameter_cfg.get("dont_return_gohome", [])
                ):
                    self._write_status_file("status", "stopping")
                    time.sleep(0.2)
                    self._write_status_file("stop_num", 1)
                    # self.w1.w1_manager.stop_move()
                    self.w1.stop_squence = True
                    time.sleep(0.2)
                    self.on_execute(voice_msg="", module_name="回原点")
                elif current_status["status"] == "stopping":
                    pass
                elif current_status["status"] == "idle":
                    pass

        except Exception as e:
            logging.error(f"Stop execute error: {e}")

    def get_status(self, part, text_field, button, pose_type="cartesian"):  # 新增 pose_type 参数，默认为笛卡尔
        """通用函数，用于获取不同部位的状态
        参数:
            pose_type: "cartesian" (默认，获取6位笛卡尔坐标) 或 "joint" (获取7位关节坐标)
        """
        try:
            # 将左右臂合并处理，因为它们的逻辑完全一致
            if part in ["left_arm", "right_arm"]:
                # 同时获取笛卡尔和关节坐标
                cart_pose, joint_pose = self.w1.get_current_pose(part, is_local=self.is_local)
                
                # 根据传入的参数决定使用哪个坐标系
                if pose_type == "joint":
                    pose = joint_pose
                    log_msg = f"{part}关节坐标获取成功"
                else:  # 默认为 "cartesian" 或其他无效输入时，兜底使用笛卡尔
                    pose = cart_pose
                    log_msg = f"{part}笛卡尔坐标获取成功"
                    
                pose = np.round(pose, 2).tolist()
                text_field.setText(str(pose))
                logging.info(log_msg)

            elif part == "left_arm_ee":
                hand_status = self.w1.get_current_pose(part)[0]
                converted = [int(x) for x in hand_status]
                text_field.setText(str(converted))
                logging.info(f"{part}状态获取成功")
            elif part == "right_arm_ee":
                hand_status = self.w1.get_current_pose(part)[0]
                converted = [int(x) for x in hand_status]
                text_field.setText(str(converted))
                logging.info(f"{part}状态获取成功")
            elif part == "head":
                head_status = self.w1.get_current_pose(part)[0]
                rounded = [int(np.round(x)) for x in head_status]
                text_field.setText(str(rounded))
                logging.info(f"{part}状态获取成功")
            elif part == "torso":
                torso_status = self.w1.get_current_pose(part)[0]
                rounded = [int(np.round(x)) for x in torso_status]
                text_field.setText(str(rounded))
                logging.info(f"{part}状态获取成功")
            else:
                logging.info(f"未知部位: {part}")
                return

            self.button_color(button, "blue")
        except Exception as e:
            logging.info(f"获取{part}状态失败: {str(e)}")

    def move_part(
        self, move_type, part=None, text_field=None, speed_ratio_key=None
    ):  # 界面触发不同类型运动
        """
        在线程中执行不同类型的运动操作，不阻塞前端
        参数:
        - move_type (str): 操作类型，可以是 'rotate', 'ee', 'linear_right', 'linear_left'
        - part (str, optional): 部位名称，如 'head', 'torso', 'right_arm_ee', 'left_arm_ee'
        - text_field (QLineEdit, optional): 包含目标位置信息的文本框
        - speed_ratio_key (str, optional): 速度比例在 device_cfg 中的键
        """
        try:
            # 检查W1是否可用
            if not hasattr(self, "w1") or self.w1 is None:
                logging.info("错误: 机器人未初始化，请先读取配置文件")
                return

            # 检查是否有运动正在执行
            if self.move_thread and self.move_thread.isRunning():
                logging.info("警告: 已有运动正在执行，请等待完成")
                return

            # 更新UI状态（立即响应）
            self.update_light(self.G4, True)

            # 创建并启动运动线程
            self.move_thread = MovePartThread(
                self, move_type, part, text_field, speed_ratio_key
            )
            self.move_thread.finished_signal.connect(self._on_move_part_finished)

            # 添加到活动线程列表便于管理
            self.active_move_threads.append(self.move_thread)

            # 启动线程
            self.move_thread.start()

            # 立即返回，不阻塞前端
            part_name = part if part else move_type
            logging.info(f"开始执行{part_name}运动...")

        except Exception as e:
            error_msg = f"启动运动线程失败: {str(e)}"
            logging.info(error_msg)
            self.update_light(self.G4, False)

    def _on_move_part_finished(self, success, message):
        """运动完成后的回调函数"""
        try:
            # 更新UI状态
            self.update_light(self.G4, False)

            # 显示运动结果
            logging.info(message)

            # 清理完成的线程
            self._cleanup_finished_threads()

        except Exception as e:
            logging.info(f"运动完成回调错误: {str(e)}")
            self.update_light(self.G4, False)

    def _cleanup_finished_threads(self):
        """清理已完成的运动线程"""
        try:
            # 移除已完成的线程
            active_threads = []
            for thread in self.active_move_threads:
                if thread.isRunning():
                    active_threads.append(thread)
                else:
                    # 断开信号连接
                    try:
                        thread.finished_signal.disconnect()
                    except:
                        pass

            self.active_move_threads = active_threads

            # 重置当前线程引用
            if self.move_thread and not self.move_thread.isRunning():
                self.move_thread = None

        except Exception as e:
            print(f"清理运动线程错误: {e}")

    def format_json_compact_inner_lists(
        self, obj, indent=2, level=0
    ):  # 优化日志打印, 方便查问题
        sp = " " * (indent * level)
        if isinstance(obj, dict):
            priority_order = [
                "left_arm",
                "left_arm_speed_ratio",
                "right_arm",
                "right_arm_speed_ratio",
                "left_end_effector",
                "right_end_effector",
                "torso",
                "torso_speed_ratio",
                "head",
                "head_speed_ratio",
            ]
            prioritized_items = []
            other_items = []
            for k, v in obj.items():
                if k in priority_order:
                    prioritized_items.append((priority_order.index(k), k, v))
                else:
                    other_items.append((len(priority_order), k, v))  # Others come after

            sorted_items = sorted(
                prioritized_items + other_items, key=lambda x: (x[0], x[1])
            )
            items = []
            for _, k, v in sorted_items:
                # Special handling for speed ratio keys - always compact
                if k.endswith("_speed_ratio"):
                    items.append(
                        f'{sp}{" " * indent}"{k}": {self.format_speed_ratio(v)}'
                    )
                else:
                    items.append(
                        f'{sp}{" " * indent}"{k}": {self.format_json_compact_inner_lists(v, indent, level+1)}'
                    )
            return "{\n" + ",\n".join(items) + f"\n{sp}}}"
        elif isinstance(obj, list):
            if all(i is None for i in obj):
                return "[" + ", ".join(["null"] * len(obj)) + "]"
            lines = []
            nulls = []
            for item in obj:
                if item is None:
                    nulls.append("null")
                else:
                    if nulls:
                        lines.append(" " * (indent * (level + 1)) + ", ".join(nulls))
                        nulls = []
                    if isinstance(item, list) and all(
                        isinstance(x, (int, float, str, type(None), list)) for x in item
                    ):

                        def item_repr(i):
                            if isinstance(i, list):
                                return (
                                    "["
                                    + ", ".join(
                                        "null" if x is None else repr(x) for x in i
                                    )
                                    + "]"
                                )
                            return "null" if i is None else repr(i)

                        lines.append(
                            " " * (indent * (level + 1))
                            + "["
                            + ", ".join(item_repr(i) for i in item)
                            + "]"
                        )
                    else:
                        lines.append(
                            " " * (indent * (level + 1))
                            + self.format_json_compact_inner_lists(
                                item, indent, level + 1
                            )
                        )
            if nulls:
                lines.append(" " * (indent * (level + 1)) + ", ".join(nulls))
            return "[\n" + ",\n".join(lines) + f"\n{sp}]"
        elif obj is None:
            return "null"
        elif isinstance(obj, str):
            return '"' + obj + '"'
        else:
            return str(obj)

    ## ————————————外部UI函数————————————————
    def open_json_editor(self):  # 打开JSON编辑器
        """Args:modal: 是否为模态窗口，True-不能操作主界面，False-可以操作主界面"""
        modal = self.device_cfg["json_ui_config"]["modal"]
        try:
            config_rel_path = self.T15.text().strip()
            if not config_rel_path:
                QMessageBox.warning(self.main_window, "警告", "请先读取配置文件")
                return
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_abs_path = os.path.abspath(
                os.path.join(current_dir, config_rel_path)
            )
            config_dir = os.path.dirname(config_abs_path)
            param_path = os.path.join(config_dir, "parameter.json")
            if not os.path.exists(param_path):
                QMessageBox.warning(
                    self.main_window,
                    "文件不存在",
                    f"未找到parameter.json文件:\n{param_path}",
                )
                return
            editor = show_json_editor(
                file_path=param_path, parent=self.main_window, modal=modal
            )
            if editor:
                editor.setWindowTitle(f"当前json文件路径: {param_path}")
                self._editor_ref = editor
                editor.activateWindow()
                editor.raise_()
                editor.destroyed.connect(
                    lambda: self._reload_after_editor_close(param_path)
                )
            else:
                QMessageBox.warning(self.main_window, "错误", "无法创建JSON编辑器")
        except Exception as e:
            QMessageBox.critical(self.main_window, "错误", f"打开编辑器失败: {str(e)}")

    def _reload_after_editor_close(
        self, file_path
    ):  # JSON编辑器关闭后重新加载文件 TODO 检查重新加载是否更新配置
        try:
            if os.path.exists(file_path):
                self._load_config_file(file_path, is_first_load=False)
        except Exception as e:
            logging.info(f"重新加载配置文件失败: {str(e)}")

    def open_image_analyzer(self):  # 打开图像分析窗口
        from qt.image_analyzer import ImageAnalyzerWindow

        try:
            if self.image_analyzer_window is not None:
                try:
                    self.image_analyzer_window.close()
                except:
                    pass
                self.image_analyzer_window = None
            self.image_analyzer_window = ImageAnalyzerWindow(self.main_window)
            self.image_analyzer_window.setAttribute(Qt.WA_DeleteOnClose, True)
            self.image_analyzer_window.show()
            self.image_analyzer_window.raise_()
            self.image_analyzer_window.activateWindow()
            self.image_analyzer_window.destroyed.connect(
                lambda: self._on_image_analyzer_closed()
            )
        except Exception as e:
            logging.info(f"打开图像分析器失败: {str(e)}")

    def _on_image_analyzer_closed(self):  # 图像分析器关闭时的处理"""
        self.image_analyzer_window = None

    def launch_detect_ros(self):  # 打开ROS物体检测UI
        try:
            import subprocess
            import sys
            import os

            # 获取当前文件所在目录
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # 构建detect_roi_ros.py的完整路径
            detect_ros_path = os.path.join(current_dir, "detect_roi_topic.py")

            # 检查文件是否存在
            if not os.path.exists(detect_ros_path):
                logging.info(
                    f"错误: 未找到ROS物体检测文件 {detect_ros_path}"
                )
                return

            # 使用子进程启动ROS物体检测
            self.detect_ros_process = subprocess.Popen(
                [sys.executable, detect_ros_path]
            )
            logging.info("ROS物体检测已启动")

        except Exception as e:
            logging.info(f"启动ROS物体检测失败: {str(e)}")

    def on_voice_service_selection(self):
        """处理语音服务下拉框选择"""
        try:
            index = self.B_voice_service_combo.currentIndex()
            current_text = self.B_voice_service_combo.currentText()
            
            if index == 0: return # 提示项

            # 确保初始化
            if self.voice_service_new_window is None:
                self._init_new_voice_ui(auto_status=False)
            
            if not self.voice_service_new_window: return

            self.B_voice_service_combo.blockSignals(True)

            # --- 逻辑 A: 服务控制 ---
            if current_text == "启动语音服务":
                self.voice_service_new_window.start_voice_service()
                self.B_voice_service_combo.setItemText(index, "关闭语音服务")
            elif current_text == "关闭语音服务":
                self.voice_service_new_window.stop_voice_service()
                self.voice_service_new_window.hide()
                self.B_voice_service_combo.setItemText(index, "启动语音服务")
                # 联动修改“显隐”文本
                s_idx = self.B_voice_service_combo.findText("隐藏语音服务")
                if s_idx != -1: self.B_voice_service_combo.setItemText(s_idx, "展示语音服务")

            # --- 逻辑 B: 显隐控制 ---
            elif current_text == "展示语音服务":
                self.voice_service_new_window.show()
                self.voice_service_new_window.raise_()
                self.B_voice_service_combo.setItemText(index, "隐藏语音服务")
            elif current_text == "隐藏语音服务":
                self.voice_service_new_window.hide()
                self.B_voice_service_combo.setItemText(index, "展示语音服务")

            self.B_voice_service_combo.setCurrentIndex(0)

        except Exception as e:
            logging.info(f"操作失败: {e}")
        finally:
            self.B_voice_service_combo.blockSignals(False)
            
    def _init_new_voice_ui(self, auto_status = False):
        """启动新的语音服务（带新UI界面）"""
        try:
            if self.voice_service_new_window is not None:
                return
            from voice_service_new_ui import VoiceServiceNewUI
            # 实例化新的UI窗口
            self.voice_service_new_window = VoiceServiceNewUI(self.main_window)

            # 设置控制器引用
            self.voice_service_new_window.set_main_controller(self)

            # 设置窗口属性
            self.voice_service_new_window.setAttribute(Qt.WA_DeleteOnClose, False)

            # 【核心改进】连接信号：当子窗口隐藏时，自动同步主界面的下拉框文本
            self.voice_service_new_window.window_close_sig.connect(self._sync_combo_text_on_ui_hidden)

            # 显示新窗口
            # self.voice_service_new_window.show()
            # self.voice_service_new_window.raise_()
            # self.voice_service_new_window.activateWindow()

            if auto_status:
                self.voice_service_new_window.start_voice_service()

        except ImportError as e:
            logging.info(f"新语音服务模块包体导入错误: {str(e)}")
        except Exception as e:
            logging.info(f"新语音服务启动失败: {str(e)}")
            import traceback

            traceback.print_exc()

    def _sync_combo_text_on_ui_hidden(self):
        """当语音窗口被手动关闭(隐藏)时，同步下拉框文本"""
        if self.B_voice_service_combo:
            # 查找“隐藏语音服务”这一项，并把它改回“展示语音服务”
            idx = self.B_voice_service_combo.findText("隐藏语音服务")
            if idx != -1:
                self.B_voice_service_combo.blockSignals(True)
                self.B_voice_service_combo.setItemText(idx, "展示语音服务")
                self.B_voice_service_combo.blockSignals(False)

    def on_tool_button_action(
        self, action
    ):  # 统一处理pickwiz,语音,http服务的启动及链接
        try:
            if action == "pickwiz_start":
                # 启动Pickwiz
                try:
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    pickwiz_path = os.path.join(current_dir, "pickwiz_start.py")
                    if not os.path.exists(pickwiz_path):
                        logging.info(
                            f"错误: 未找到Pickwiz启动文件 {pickwiz_path}"
                        )
                        return
                    subprocess.Popen([sys.executable, pickwiz_path])
                    logging.info("Pickwiz启动命令已发送")
                except Exception as e:
                    logging.info(f"启动Pickwiz失败: {str(e)}")

            elif action == 'voice_start':
                try:
                    subprocess.run(["bash", "/home/dexforce/workspace/dexe_application/qt/sound.sh"], check=True)

                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    w1_start_path = os.path.join(current_dir, "dexe_agent_start.py")
                    if not os.path.exists(w1_start_path):
                        logging.info(f"错误: 未找到W1启动文件 {w1_start_path}")
                        return
                    self.voice_process = subprocess.Popen([sys.executable, w1_start_path, "--voice"])
                    logging.info("语音服务启动命令已发送")

                except Exception as e:
                    logging.info(f"启动语音服务失败: {str(e)}")

            elif action == "http":
                # 启动或停止HTTP服务
                try:
                    self.toggle_flask_server()
                except Exception as e:
                    error_msg = f"启动HTTP服务失败: {str(e)}"
                    print(error_msg)
                    logging.info(error_msg)
                    import traceback

                    print(traceback.format_exc())

            elif action == "simbyviser":
                # 启动或停止SimByViser仿真
                try:
                    # 优先根据按钮文字判断用户意图，防止进程崩溃后 UI 状态卡在“关闭仿真”
                    if self.B_simbyviser.text() == "启动仿真":
                        # 启动仿真
                        sim_script_path = "/home/dexforce/workspace/dexe_application/tools/simbyviser/src/main_ros.py"
                        if not os.path.exists(sim_script_path):
                            logging.info(f"错误: 未找到仿真脚本 {sim_script_path}")
                            return
                        
                        self.simbyviser_process = subprocess.Popen([sys.executable, sim_script_path])
                        self.B_simbyviser.setText("关闭仿真")
                        self.B_simbyviser.setStyleSheet(
                            """
                            QPushButton {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #cc0000, stop:1 #660000);
                                color: white;
                                border: 2px solid #ff4444;
                                border-radius: 5px;
                                padding: 5px;
                                font-weight: bold;
                            }
                            QPushButton:hover {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #ff0000, stop:1 #880000);
                            }
                            QPushButton:pressed {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #880000, stop:1 #440000);
                            }
                        """
                        )  # red style
                        logging.info("仿真服务启动成功")
                    else:
                        # 停止仿真/重置状态
                        if self.simbyviser_process is not None:
                            if self.simbyviser_process.poll() is None:
                                self.simbyviser_process.terminate()
                                logging.info("仿真服务已停止")
                            else:
                                logging.info("仿真服务已意外退出,正在重置UI状态")
                            self.simbyviser_process = None
                        
                        self.B_simbyviser.setText("启动仿真")
                        self.B_simbyviser.setStyleSheet(
                            """
                            QPushButton {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #00aa00, stop:1 #006600);
                                color: white;
                                border: 2px solid #00ff00;
                                border-radius: 5px;
                                padding: 5px;
                                font-weight: bold;
                            }
                            QPushButton:hover {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #00cc00, stop:1 #008800);
                            }
                            QPushButton:pressed {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #008800, stop:1 #004400);
                            }
                        """
                        )  # green style
                except Exception as e:
                    logging.info(f"操作仿真服务失败: {str(e)}")
                    # 发生任何错误时，强制将按钮重置回“启动仿真”状态，防止卡在“关闭仿真”
                    self.B_simbyviser.setText("启动仿真")
                    self.B_simbyviser.setStyleSheet(
                        """
                        QPushButton {
                            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                    stop:0 #00aa00, stop:1 #006600);
                            color: white;
                            border: 2px solid #00ff00;
                            border-radius: 5px;
                            padding: 5px;
                            font-weight: bold;
                        }
                        QPushButton:hover {
                            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                    stop:0 #00cc00, stop:1 #008800);
                        }
                        QPushButton:pressed {
                            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                    stop:0 #008800, stop:1 #004400);
                        }
                    """
                    )

            elif action == "pickwiz_connection":
                # 视觉通讯开关
                try:
                    if self.B_vision.text() == "视觉通讯":
                        # 启动视觉服务
                        if not hasattr(self, "vision_app") or self.vision_app is None:
                            logging.info("请先读取配置文件")
                            return

                        sign = self.vision_app.connection()
                        if sign:
                            self.B_vision.setText("停止视觉通讯")
                            self.B_vision.setStyleSheet(
                                """
                                QPushButton {
                                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                            stop:0 #00aa00, stop:1 #006600);
                                    color: white;
                                    border: 2px solid #00ff00;
                                    border-radius: 5px;
                                    padding: 5px;
                                    font-weight: bold;
                                }
                                QPushButton:hover {
                                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                            stop:0 #00cc00, stop:1 #008800);
                                }
                                QPushButton:pressed {
                                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                            stop:0 #008800, stop:1 #004400);
                                }
                            """
                            )  # green style
                            logging.info("视觉通讯启动成功")
                        else:
                            logging.info("视觉通讯启动失败")
                    else:
                        # 停止视觉服务
                        self.vision_app.dis_connection()
                        self.B_vision.setText("视觉通讯")
                        self.B_vision.setStyleSheet(
                            """
                            QPushButton {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #808080, stop:1 #606060);
                                color: white;
                                border: 2px solid #a0a0a0;
                                border-radius: 5px;
                                padding: 5px;
                                font-weight: bold;
                            }
                            QPushButton:hover {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #909090, stop:1 #707070);
                            }
                            QPushButton:pressed {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #707070, stop:1 #505050);
                            }
                        """
                        )  # gray style
                        logging.info("视觉通讯已停止")

                except Exception as e:
                    logging.info(f"视觉通讯操作失败: {str(e)}")

            elif action == "voice_connection":
                # 语音通讯开关
                try:
                    if self.B_voice_service.text() == "语音通讯":
                        # 启动语音服务
                        self.start_voice_communication()
                        self.B_voice_service.setText("语音通讯关闭")
                        self.B_voice_service.setStyleSheet(
                            """
                            QPushButton {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #00aa00, stop:1 #006600);
                                color: white;
                                border: 2px solid #00ff00;
                                border-radius: 5px;
                                padding: 5px;
                                font-weight: bold;
                            }
                            QPushButton:hover {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #00cc00, stop:1 #008800);
                            }
                            QPushButton:pressed {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #008800, stop:1 #004400);
                            }
                        """
                        )  # green style
                        logging.info("语音通讯启动成功")
                    else:
                        # 停止语音服务
                        self.stop_voice_communication()
                        self.B_voice_service.setText("语音通讯")
                        self.B_voice_service.setStyleSheet(
                            """
                            QPushButton {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #808080, stop:1 #606060);
                                color: white;
                                border: 2px solid #a0a0a0;
                                border-radius: 5px;
                                padding: 5px;
                                font-weight: bold;
                            }
                            QPushButton:hover {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #909090, stop:1 #707070);
                            }
                            QPushButton:pressed {
                                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #707070, stop:1 #505050);
                            }
                        """
                        )  # gray style
                        logging.info("语音通讯已停止")

                except Exception as e:
                    logging.info(f"语音通讯操作失败: {str(e)}")

            else:
                logging.info(f"未知操作: {action}")
        except Exception as e:
            logging.info(f"执行操作 '{action}' 时出错: {str(e)}")

    def on_sim_mode_changed(self):
        """处理仿真/数字孪生模式切换"""
        try:
            if not self.w1 or not hasattr(self.w1, "w1_manager"):
                logging.info("错误: 机器人未初始化,请先读取配置")
                return
            
            index = self.B_sim_mode_combo.currentIndex()
            enable_sim = (index == 1)
            self.is_sim = enable_sim
            mode_name = "纯仿真" if enable_sim else "数字孪生"
            
            logging.info(f"正在切换到 {mode_name} 模式...")
            
            # 调用 W1Manager 的 set_sim_mode 方法
            result = self.w1.w1_manager.set_sim_mode(enable_sim=enable_sim)
            
            # 检查结果 (StatusCode)
            if "SUCCESS" in str(result):
                logging.info(f"成功切换到 {mode_name} 模式")
                # 同步更新 RobotApp 的 is_sim 状态
                if hasattr(self.w1, "is_sim"):
                    self.w1.is_sim = enable_sim
            else:
                logging.info(f"切换到 {mode_name} 模式失败: {result}")
                
        except Exception as e:
            logging.info(f"模式切换异常: {str(e)}")

    def run_picklight_vision(self):  # 手动触发picklight视觉流程
        """启动实时取图脚本（关闭窗口自动结束）"""
        try:
            self._picklight_vision_process = subprocess.Popen(
                ["python3", "/home/dexforce/workspace/dexe_application/vision/imag_show2.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            logging.info("实时取图脚本已启动，关闭图像窗口即可退出")
        except Exception as e:
            logging.info(f"实时取图脚本启动失败: {str(e)}")

    def Get_kfc_param(self):  # 手动触发picklight视觉流程
        """获取相机内参"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(current_dir, "get_camera_param.py")

        if not os.path.exists(script_path):
            logging.info(f"❌ 找不到传输脚本: {script_path}")
            return

        self.get_kfc_param_thread = PC1LogSaveSimpleThread(script_path)
        self.get_kfc_param_thread.finished_signal.connect(self._get_kfc_param_finished)

        if hasattr(self, "active_threads"):
            self.active_threads.append(self.get_kfc_param_thread)

        logging.info("正在后台获取KFC相机内参...")
        self.get_kfc_param_thread.start()

    def _get_kfc_param_finished(self, success, msg):
        """传输脚本执行结束后的回调"""
        logging.info(msg)

    def read_picklight_vision(self):  # 打开点云浏览器UI
        """打开点云浏览器UI"""
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            pointcloud_browser_path = os.path.join(
                os.path.dirname(os.path.dirname(current_dir)),  # 回到workspace目录
                "dexe_application",
                "vision",
                "pointcloud_browser.py",
            )
            if not os.path.exists(pointcloud_browser_path):
                logging.info(
                    f"错误: 未找到点云浏览器文件 {pointcloud_browser_path}"
                )
                return
            self.pointcloud_browser_process = subprocess.Popen(
                [sys.executable, pointcloud_browser_path]
            )
            logging.info("点云浏览器已启动")
        except Exception as e:
            logging.info(f"启动点云浏览器失败: {str(e)}")
            import traceback

            logging.info(f"详细错误: {traceback.format_exc()}")

    def start_voice_communication(self):  # 启动语音通信
        """启动语音通信"""
        try:
            if (
                not hasattr(self, "async_thread")
                or not self.async_thread
                or not self.async_thread.is_alive()
            ):
                self.async_thread = DexAgentThread(task_handler=self)
                self.async_thread.start()
                logging.info("语音通信已启动")
                # # 新增支付功能，延迟发送支付状态，确保语音服务已准备好
                # QTimer.singleShot(2000, self.send_payment_status_to_voice)
                
            else:
                logging.info("语音通信已在运行中")
        except Exception as e:
            logging.info(f"启动语音通信失败: {str(e)}")

    def stop_voice_communication(self):  # 停止语音通信
        try:
            if hasattr(self, "async_thread") and self.async_thread:
                self.async_thread.stop()
                self.async_thread = None
                logging.info("语音通信已停止")
        except Exception as e:
            logging.info(f"停止语音通信失败: {str(e)}")
        try:
            if (
                not hasattr(self, "async_thread")
                or not self.async_thread
                or not self.async_thread.is_alive()
            ):
                self.async_thread = DexAgentThread(task_handler=self)
                self.async_thread.start()
                logging.info("语音通信已启动")
        except Exception as e:
            logging.info(f"启动语音通信失败: {str(e)}")

    ## ————————————HTTP服务相关函数————————————————
    def handle_http_operations(self, operation_type, *args):
        """
        统一的HTTP操作处理函数
        operation_type: 'start_server', 'coffee_order', 'stop_motion', 'consult_message'
        """
        try:
            if operation_type == "start_server":
                return self._start_http_server()
            elif operation_type == "coffee_order":
                coffee_name, client_ip = args
                logging.info(
                    f"收到咖啡订单: {coffee_name} (来自: {client_ip})"
                )
                self.on_execute(voice_msg=coffee_name)
            elif operation_type == "stop_motion":
                client_ip = args[0] if args else "未知IP"
                logging.info(f"收到停止运动指令 (来自: {client_ip})")
                self.on_stop_execution(stop_gohome=True)
            elif operation_type == "consult_message":
                message_content, client_ip = args
                logging.info(
                    f"收到咨询消息: {message_content} (来自: {client_ip})"
                )
                self.on_execute(voice_msg=message_content)
            else:
                logging.info(f"未知的HTTP操作类型: {operation_type}")

        except Exception as e:
            logging.info(f"HTTP操作失败 ({operation_type}): {str(e)}")

    def _update_http_log_ui(self, message):
        """在UI线程中更新日志显示"""
        try:
            if hasattr(self, "K4") and self.K4 is not None:
                # 添加时间戳
                # timestamp = datetime.now().strftime("%H:%M:%S")
                log_message = f"{message}"

                # 添加到K4日志框
                self.K4.append(log_message)

                # 自动滚动到底部
                scrollbar = self.K4.verticalScrollBar()
                if scrollbar:
                    scrollbar.setValue(scrollbar.maximum())

        except Exception as e:
            print(f"更新HTTP日志UI错误: {e}")

    def handle_http_server_log(self, message):
        """处理HTTP服务器日志 - 确保线程安全"""
        self._update_http_log_ui(message)

    def _start_http_server(self):
        """启动HTTP服务器的内部实现"""
        try:
            # 如果服务器已经在运行，先停止它
            if self.http_server_running:
                self._stop_http_server()
                return "HTTP服务已重启"
            self.http_server_thread = HTTPServerThread(self)
            self.http_server_thread.log_signal.connect(
                lambda msg: print(f"[HTTP] {msg}")
            )
            self.http_server_thread.coffee_signal.connect(
                lambda name, ip: self.handle_http_operations("coffee_order", name, ip)
            )
            self.http_server_thread.consult_signal.connect(
                lambda content, ip: self.handle_http_operations(
                    "consult_message", content, ip
                )
            )
            self.http_server_thread.stop_signal.connect(
                lambda ip: self.handle_http_operations("stop_motion", ip)
            )
            self.http_server_thread.start()
            self.http_server_running = True
            self.B_http.setText("停止HTTP服务")
            self.B_http.setStyleSheet(self._get_button_style("green"))
            return "HTTP服务启动成功"

        except Exception as e:
            error_msg = f"启动HTTP服务失败: {str(e)}"
            print(error_msg)
            logging.info(error_msg)
            import traceback

            print(traceback.format_exc())
            return error_msg

    def _stop_http_server(self):
        """停止HTTP服务器的内部实现"""
        try:
            if self.http_server_thread and self.http_server_thread.isRunning():
                self.http_server_thread.stop()
                if not self.http_server_thread.wait(1000):
                    self.http_server_thread.terminate()
                    self.http_server_thread.wait()

            self.http_server_thread = None
            self.http_server_running = False
            print("[HTTP] HTTP服务已停止")
            return "HTTP服务已停止"
        except Exception as e:
            logging.info(f"HTTP服务停止失败: {str(e)}")

    # 1112修改 胶囊计数
    def update_capsule_count(self):
        """更新胶囊数量显示 - 只读取特定字符"""
        try:
            # 使用相对路径获取 auto_config.json
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # 回到qt文件夹的上级目录（dexe_application），然后进入parameters文件夹
            app_root = os.path.dirname(current_dir)
            capsule_file = os.path.join(app_root, "qt", "auto_config.json")

            capsule_num = 0  # 默认值

            if os.path.exists(capsule_file):
                with open(capsule_file, "r", encoding="utf-8") as f:
                    content = f.read()

                # 使用正则表达式只匹配 "capsule_coffeemachine_num" 的赋值
                import re

                match = re.search(r'"capsule_coffeemachine_num"\s*:\s*(\d+)', content)
                if match:
                    capsule_num = int(match.group(1))
                else:
                    # 如果正则匹配失败，尝试其他可能的格式
                    match = re.search(
                        r"capsule_coffeemachine_num\s*:\s*(\d+)", content
                    )  # 不带引号的情况
                    if match:
                        capsule_num = int(match.group(1))
                    else:
                        capsule_num = 0  # 保持默认值
            else:
                capsule_num = 0

            # 更新UI显示和警示灯状态
            if hasattr(self, "capsule_label"):
                self.capsule_label.setText(f"胶囊数量: {capsule_num}")

                # 根据胶囊数量设置警示灯颜色
                # 读取阈值配置
                threshold = 15  # 默认阈值
                if os.path.exists(capsule_file):
                    with open(capsule_file, "r", encoding="utf-8") as f:
                        config_data = json.load(f)
                        # 使用新的键名，避免冲突
                        threshold = config_data.get("capsule_threshold_value", 15)

                if capsule_num >= threshold:
                    # 红色荧光填充
                    self.capsule_label.setStyleSheet(
                        """
                        QLabel {
                            color: #ffffff;
                            background-color: #ff0000;
                            border: 2px solid #ff4444;
                            border-radius: 10px;
                            text-align: center;
                            font-weight: bold;
                        }
                    """
                    )
                else:
                    # 正常状态（灰色）
                    self.capsule_label.setStyleSheet(
                        """
                        QLabel {
                            color: #ffffff;
                            background-color: #1a2a3a;
                            border: 1px solid #00b4ff;
                            border-radius: 10px;
                            text-align: center;
                        }
                    """
                    )

        except Exception as e:
            if hasattr(self, "capsule_label"):
                self.capsule_label.setText("胶囊数量: 读取错误")
                # 读取错误时显示黄色警示
                self.capsule_label.setStyleSheet(
                    """
                    QLabel {
                        color: #ffffff;
                        background-color: #ffaa00;
                        border: 1px solid #ffcc00;
                        border-radius: 10px;
                        text-align: center;
                    }
                """
                )

    def setup_capsule_counter(self):
        """设置胶囊计数器"""
        self.capsule_timer = QTimer()
        self.capsule_timer.timeout.connect(self.update_capsule_count)
        self.capsule_timer.start(10000)  # 10秒刷新一次

        # 确保按钮存在后再连接信号
        if hasattr(self, "capsule_reset_btn") and self.capsule_reset_btn:
            try:
                # 先断开可能存在的连接，避免重复连接
                self.capsule_reset_btn.clicked.disconnect()
            except:
                pass
            # 重新连接信号
            self.capsule_reset_btn.clicked.connect(self.on_capsule_reset)
            logging.info("胶囊重置按钮连接成功")

        # 立即更新一次
        self.update_capsule_count()

    def on_capsule_reset(self):
        """处理胶囊数量重置 - 简化版本"""
        try:
            logging.info("开始重置胶囊数量...")

            # 直接使用绝对路径，避免路径问题
            current_dir = os.path.dirname(os.path.abspath(__file__))
            capsule_file = os.path.join(current_dir, "auto_config.json")

            logging.info(f"目标文件: {capsule_file}")

            if not os.path.exists(capsule_file):
                logging.info(f"错误: 文件不存在 {capsule_file}")
                return

            # 直接读取和修改文件
            try:
                with open(capsule_file, "r", encoding="utf-8") as f:
                    data = json.load(f)

                # 更新胶囊数量
                data["capsule_coffeemachine_num"] = 0
                data["coffeemachine_threshold"] = False
                data["capsule_threshold_value"] = 15  # 实际的阈值数值

                # 写回文件
                with open(capsule_file, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)

                logging.info("胶囊数量已重置为 0")

                # 立即更新显示
                self.update_capsule_count()

            except Exception as e:
                logging.info(f"文件操作错误: {str(e)}")

        except Exception as e:
            error_msg = f"重置胶囊数量失败: {str(e)}"
            logging.info(error_msg)
            import traceback

            logging.info(f"详细错误: {traceback.format_exc()}")

class FakeTextField:
    def __init__(self, data_list):
        self.data_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def text(self):
        return str(self.data_list)

def check_single_instance():
    """检测单实例锁"""
    # 建议使用一个固定的锁文件，防止重复启动
    # 可以放在 /tmp 或项目根目录下
    lock_file_path = app_dir / ".w1_controller.lock"
    
    # 必须将 fp 定义为全局变量或返回它，否则函数结束 fp 被销毁，锁会失效
    global lock_file
    lock_file = open(lock_file_path, 'w')
    
    try:
        # fcntl.LOCK_EX: 排他锁
        # fcntl.LOCK_NB: 非阻塞，如果已被占用立即抛出异常
        fcntl.lockf(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except IOError:
        print("w1_controller.py 已经在运行中，自动退出。")
        sys.exit(0)

class orderHandler:
    def __init__(self):
        self.json_file = "/home/dexforce/workspace/dexe_application/qt/ordermanagement/currentOrder.json"
        self.config_file = "/home/dexforce/workspace/dexe_application/qt/auto_config.json"
        with open(self.config_file, "r+", encoding="utf-8") as f:
            data = json.load(f)
            data.setdefault("tasks", {})["done"] = True
            f.seek(0)
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.truncate()

    def start(self):
        threading.Thread(target=self._loop_check, daemon=True).start()

    def _loop_check(self):
        while True:
            self._check_and_execute()
            time.sleep(1) 

    def _check_and_execute(self):
        products = self._read_products()
        if not products:
            # logging.warning("✅ 所有商品处理完毕")
            return

        if self._read_done():
            logging.warning("✅ done=True，处理商品")
            self._process_one_product()
        else:
            pass
            # logging.warning("⏳ 等待 done=True...")

    def _read_products(self):
        try:
            with open(self.json_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except:
            return []

    def _read_done(self):
        try:
            with open(self.config_file, "r", encoding="utf-8") as f:
                data = json.load(f)
                return data.get("tasks", {}).get("done", False)
        except:
            return False

    def _process_one_product(self):
        # 处理一个商品
        products = self._read_products()
        if not products:
            return

        with open(self.config_file, "r+", encoding="utf-8") as f:
            data = json.load(f)
            data.setdefault("tasks", {})["done"] = False
            f.seek(0)
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.truncate()

        product = products[-1]
        task = product.get("task", "")
        card = product.get("card", "")
        temperature = product.get("temperature", "")
        if str(card) == "1":
            task=temperature+task
        logging.info(f"🚀 执行任务: {task}")
        controller.on_execute(voice_msg=f"{task}", module_name = f"{task}")
        

if __name__ == "__main__":
    try:
        check_single_instance()
    except ImportError:
        # 如果在 Windows 环境运行，防止报错（或者使用 portalocker 库）
        pass
    ui_app = QApplication([])
    window = QMainWindow()
    w1ui = W1Displayui()
    controller = W1Controller(window)

    
    window.destroyed.connect(lambda: controller.closeEvent(None))
    window.show()

    # from ui import MainWindow as NewUIWindow
    # new_window = NewUIWindow(w1_ctrl_instance=controller)
    # new_window.show()
    # new_window.destroyed.connect(lambda: controller.closeEvent(None))

    def initialize_and_monitor():
        controller.on_config_read_origin()

    QTimer.singleShot(0, initialize_and_monitor)

    result = ui_app.exec()
    controller.closeEvent(None)
    sys.exit(result)
