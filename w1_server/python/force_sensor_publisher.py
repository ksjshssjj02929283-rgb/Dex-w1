import serial
import time
import struct
import sys
import matplotlib.pyplot as plt  # 新增
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import threading

class SensorReader(Node):
    def __init__(self, port, topic):
        # rclpy.init(args=None)  # 删除这一行
        super().__init__('wrench_sensor_' + port.replace('/', '_'))
        self.ser = None
        self.port = port
        self.baudrate = 460800
        self.data_format = '<ccffffffcc'
        self.zero_frame = None  
        self.zero_buffer = []   
        self.zero_time = 0.1   
        self.zero_set = False  
        self.filtered = None 
        self.alpha = self.calc_alpha(1000, 10)  # 1kHz采样，10Hz截止
        self.wrench = self.create_publisher(WrenchStamped, topic, 10)
        self.frame_count = 0

    def calc_alpha(self, fs, fc):
        """计算一阶低通滤波器系数"""
        dt = 1.0 / fs
        rc = 1.0 / (2 * np.pi * fc)
        alpha = dt / (rc + dt)
        return alpha

    def sleep_ms(self, milliseconds):
        """毫秒级延迟"""
        time.sleep(milliseconds / 1000.0)
        
    def configure_port(self):
        """配置串口参数"""
        try:
            # 打开串口
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 100ms超时
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            # 确保串口已打开
            if not self.ser.is_open:
                self.ser.open()
                
            # 清空缓冲区
            self.ser.flushInput()
            self.ser.flushOutput()
            
            return True
            
        except Exception as e:
            print(f"配置串口失败: {str(e)}")
            return False
            
    def send_command(self, cmd):
        """发送命令到传感器"""
        if not self.ser or not self.ser.is_open:
            print("串口未打开")
            return

        tx_buf = bytearray()
        tx_buf.append(cmd)
        tx_buf.append(0xAA)
        tx_buf.append(0x0D)
        tx_buf.append(0x0A)

        self.ser.write(tx_buf)
        
    def start_sensor(self):
        """开始转换数据"""
        self.send_command(0x48)
        
    def stop_sensor(self):
        """停止转换数据"""
        self.send_command(0x43)
        
    def get_version(self):
        """获取版本信息"""
        self.send_command(0xF0)
        
    def lowpass_filter(self, current, last):
        """一阶低通滤波器"""
        return self.alpha * current + (1 - self.alpha) * last

    def read_valid_frame(self, timeout=0.01):
        """读取一帧有效数据（cd和cr校验通过），超时返回None"""
        if not self.ser or not self.ser.is_open:
            assert False, "串口未打开"
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                return None
            first_byte = self.ser.read(1)
            if not first_byte or first_byte[0] not in [0x48, 0x49]:
                continue
            remaining_data = self.ser.read(27)
            if len(remaining_data) != 27:
                print(f"数据不完整，期望27字节，实际收到{len(remaining_data)}字节")
                continue
            data = first_byte + remaining_data

            unpacked = struct.unpack(self.data_format, data)
            cd = unpacked[8]
            cr = unpacked[9]
            if (cd != b'\x0D') or (cr != b'\x0A'):
                continue
            return data
    
    def init(self):
        self.start_sensor()
        """采集前500帧数据取平均值作为零位校准"""
        zero_buffer = []
        print(f"Port: {self.port} 正在采集零位校准数据，请保持传感器静止...")
        for _ in range(1000):
            data = self.read_valid_frame()
            if data is None:
                continue
            unpacked = struct.unpack(self.data_format, data)
            g = 9.80665
            Fx = unpacked[2] * g
            Fy = unpacked[3] * g
            Fz = unpacked[4] * g
            Mx = unpacked[5] * g
            My = unpacked[6] * g
            Mz = unpacked[7] * g
            zero_buffer.append([Fx, Fy, Fz, Mx, My, Mz])
            self.sleep_ms(1)
        arr = np.array(zero_buffer)
        self.zero_frame = tuple(np.mean(arr, axis=0))
        print("零位校准完成，零点值：", self.zero_frame)
        

    def pub_sensor_data(self):
        data = self.read_valid_frame()
        if data is None:
            return
        unpacked = struct.unpack(self.data_format, data)
        g = 9.80665
        Fx = unpacked[2] * g
        Fy = unpacked[3] * g
        Fz = unpacked[4] * g
        Mx = unpacked[5] * g
        My = unpacked[6] * g
        Mz = unpacked[7] * g

        # 零点校准
        Fx -= self.zero_frame[0]
        Fy -= self.zero_frame[1]
        Fz -= self.zero_frame[2]
        Mx -= self.zero_frame[3]
        My -= self.zero_frame[4]
        Mz -= self.zero_frame[5]

        # 低通滤波
        raw = np.array([Fx, Fy, Fz, Mx, My, Mz])
        if self.filtered is None:
            self.filtered = np.zeros(6)
        else:
            self.filtered = self.lowpass_filter(raw, self.filtered)
        Fx, Fy, Fz, Mx, My, Mz = self.filtered

        sensor_data = {
            'Fx': Fx,
            'Fy': Fy,
            'Fz': Fz,
            'Mx': Mx,
            'My': My,
            'Mz': Mz,
        }
        
        if not sensor_data:
            return
        
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.x = sensor_data['Fx']
        msg.wrench.force.y = sensor_data['Fy']
        msg.wrench.force.z = sensor_data['Fz']
        msg.wrench.torque.x = sensor_data['Mx']
        msg.wrench.torque.y = sensor_data['My']
        msg.wrench.torque.z = sensor_data['Mz']
        self.wrench.publish(msg)
        self.frame_count += 1
        if self.frame_count % 500 == 0:
            print(f"[{self.port}] 第{self.frame_count}帧: {sensor_data}")

            
    def run(self):
        """运行主循环"""
        # 配置串口
        if not self.configure_port():
            return
        
        self.init()

        try:
            while True:
                self.pub_sensor_data()
                self.sleep_ms(1)
        except KeyboardInterrupt:
            print(f"\n检测到Ctrl+C，正在安全退出 {self.port} ...")
            self.stop_sensor()
            if self.ser and self.ser.is_open:
                self.ser.close()
            print(f"{self.port} 已安全关闭。")
        except Exception as e:
            print(f"运行时异常: {e}")
            self.stop_sensor()
            if self.ser and self.ser.is_open:
                self.ser.close()

def run_sensor(port, topic):
    reader = SensorReader(port=port, topic=topic)
    reader.run()

if __name__ == "__main__":
    rclpy.init(args=None)
    try:
        left_thread = threading.Thread(target=run_sensor, args=('/dev/ttyUSB0', '/feedback/force_sensor/left'))
        left_thread.start()
        left_thread.join()

        right_thread = threading.Thread(target=run_sensor, args=('/dev/ttyUSB1', '/feedback/force_sensor/right'))
        right_thread.start()
        right_thread.join()
    except KeyboardInterrupt:
        print("\n安全退出传感器线程...")
