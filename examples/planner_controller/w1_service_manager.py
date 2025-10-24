#!/usr/bin/env python3
"""
W1 Controller Service Manager
A more advanced service manager for W1Controller with proper lifecycle management
"""

import numpy as np
from dexe_robot_client.w1.w1_controller import W1Controller
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
import time
import signal
import threading
import argparse
import sys
from contextlib import contextmanager


class W1ServiceManager:
    """W1 Controller Service Manager with proper lifecycle management"""
    
    def __init__(self, 
                 is_sim: bool = False,
                 arm_type: W1ArmType = W1ArmType.ANTHROPOMORPHIC,
                 w1_version: W1Version = W1Version.V2,
                 end_effector_type: EndEffectorType = None):
        
        self.is_sim = is_sim
        self.arm_type = arm_type
        self.w1_version = w1_version
        self.end_effector_type = end_effector_type
        
        self.w1_controller = None
        self.shutdown_event = threading.Event()
        self.is_running = False
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """处理退出信号"""
        signal_names = {signal.SIGINT: "SIGINT (Ctrl+C)", signal.SIGTERM: "SIGTERM"}
        signal_name = signal_names.get(signum, f"Signal {signum}")
        print(f"\n🛑 Received {signal_name}, shutting down gracefully...")
        self.shutdown()
    
    def start(self):
        """启动W1 Controller服务"""
        if self.is_running:
            print("⚠️  W1 Controller services are already running!")
            return False
        
        try:
            print("🚀 Starting W1 Controller services...")
            print(f"   - Simulation mode: {self.is_sim}")
            print(f"   - Arm type: {self.arm_type}")
            print(f"   - W1 version: {self.w1_version}")
            print(f"   - End effector: {self.end_effector_type}")
            
            self.w1_controller = W1Controller(
                is_sim=self.is_sim,
                arm_type=self.arm_type,
                w1_version=self.w1_version,
                end_effector_type=self.end_effector_type
            )
            
            self.is_running = True
            print("✅ W1 Controller services started successfully!")
            print("📡 ROS2 services are now available:")
            print("   - /connect")
            print("   - /is_connected") 
            print("   - /clear_robot_error")
            print("   - /set_drag_mode")
            print("   - /set_safe_level")
            print("   - /set_sim_mode")
            print("   - /set_ee_qpos")
            print("   - /go_home")
            print("   - /stop_move")
            print("   - /set_home_qpos")
            print("   - /execute_joint_traj")
            print("   - /start_moving")
            print("\n💡 Press Ctrl+C to shutdown gracefully")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to start W1 Controller services: {e}")
            self.is_running = False
            return False
    
    def shutdown(self):
        """关闭服务"""
        if not self.is_running:
            return
        
        print("🔄 Shutting down W1 Controller services...")
        self.shutdown_event.set()
        self.is_running = False
        
        # 这里可以添加更多清理逻辑
        if self.w1_controller:
            # 如果W1Controller有清理方法，在这里调用
            pass
        
        print("✅ W1 Controller services stopped gracefully.")
    
    def wait_for_shutdown(self):
        """等待关闭信号"""
        try:
            self.shutdown_event.wait()
        except KeyboardInterrupt:
            print("\n🔄 Keyboard interrupt received...")
            self.shutdown()
    
    def run(self):
        """运行服务管理器"""
        if self.start():
            self.wait_for_shutdown()
        else:
            sys.exit(1)
    
    @contextmanager
    def service_context(self):
        """上下文管理器，确保服务正确启动和关闭"""
        try:
            if self.start():
                yield self.w1_controller
            else:
                raise RuntimeError("Failed to start W1 Controller services")
        finally:
            self.shutdown()


def main():
    """主函数，支持命令行参数"""
    parser = argparse.ArgumentParser(description="W1 Controller Service Manager")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode")
    parser.add_argument("--real", action="store_true", help="Run in real robot mode")
    parser.add_argument("--arm-type", type=str, default="anthropomorphic", 
                       choices=["anthropomorphic"], help="Arm type")
    parser.add_argument("--version", type=str, default="v2", 
                       choices=["v2"], help="W1 version")
    parser.add_argument("--ee-type", type=str, default=None,
                       help="End effector type")
    
    args = parser.parse_args()
    
    # 处理模式选择
    if args.sim and args.real:
        print("❌ Cannot specify both --sim and --real")
        sys.exit(1)
    elif args.real:
        is_sim = False
    else:
        is_sim = True  # 默认仿真模式
    
    # 转换参数
    arm_type = W1ArmType.ANTHROPOMORPHIC  # 暂时只支持这一种
    w1_version = W1Version.V2
    end_effector_type = None  # 可以根据args.ee_type进一步处理
    
    # 创建和运行服务管理器
    manager = W1ServiceManager(
        is_sim=is_sim,
        arm_type=arm_type,
        w1_version=w1_version,
        end_effector_type=end_effector_type
    )
    
    manager.run()


def simple_example():
    """简单使用示例"""
    manager = W1ServiceManager(is_sim=True)
    manager.run()


def context_example():
    """使用上下文管理器的示例"""
    manager = W1ServiceManager(is_sim=True)
    
    # 使用上下文管理器确保正确清理
    with manager.service_context() as w1_controller:
        print("Services are running in context...")
        # 在这里可以执行其他操作
        input("Press Enter to shutdown...")
        # 上下文退出时会自动关闭服务


if __name__ == "__main__":
    main()
