# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import serial

class BraincoHands:
    def __init__(self, tty_name):
        self.client = serial.Serial(
        port=tty_name,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1)
        self.left_id = 1
        self.right_id = 2
        try:
            if not self.client.is_open:
                 self.client.open()
                 print("串口已打开")
        except Exception as e:
             print(f"错误: {e}")

    def CloseSerial(self):
        if self.client.is_open:
            self.client.close()
            print("串口已关闭")

    def modbus_crc16(self,buffer):
        """计算Modbus RTU CRC16校验码"""
        crc = 0xFFFF
        for byte in buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return ((crc << 8) | (crc >> 8)) & 0xFFFF


    def send_modbus_read_command(self, device_id, register_address, register_count):
        """发送Modbus RTU读保持寄存器命令(功能码0x03)"""
        # 构建请求数据
        response=None
        ser=self.client
        request = bytearray()
        request.append(device_id)
        request.append(0x04)  # 读保持寄存器功能码
        request.append((register_address >> 8) & 0xFF)
        request.append(register_address & 0xFF)
        request.append((register_count >> 8) & 0xFF)
        request.append(register_count & 0xFF)
        
        # 计算CRC校验
        crc = self.modbus_crc16(request)
        request.append((crc >> 8) & 0xFF)  # CRC高字节
        request.append(crc & 0xFF)         # CRC低字节
        
        # 发送请求
        ser.write(request)
        print(f"发送读请求: {request.hex(' ')}")
        
        # 等待响应
        time.sleep(0.03)  # 等待设备响应
        
        # 读取响应
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"接收读响应: {response.hex(' ')}")
            
            # 简单验证响应
            if len(response) < 5:  # 至少包含: 设备ID(1) + 功能码(1) + 字节数(1) + 数据(n) + CRC(2)
                print("响应数据长度不足")
                return None
                
            # 验证CRC校验
          #  response_data = response[:-2]  # 去掉CRC
          #  response_crc = (response[-2] << 8) | response[-1]
          #  calculated_crc = self.modbus_crc16(response_data)
          #  
          #  if response_crc != calculated_crc:
          #      print("CRC校验失败")
          #      return None
                
            # 解析响应数据
            if response[1] == 0x04:  # 确认功能码
                byte_count = response[2]
                if byte_count == register_count * 2:  # 每个寄存器2字节
                    values = []
                    for i in range(register_count):
                        reg_value = (response[3 + i*2] << 8) | response[4 + i*2]
                        values.append(reg_value)
                    return values
                else:
                    print(f"字节数不匹配: 期望 {register_count*2}, 实际 {byte_count}")
            elif response[1] == 0x83:  # 异常响应
                error_code = response[2]
                print(f"Modbus异常响应: 错误码 {error_code}")
                
        return [10]*6 


    def send_modbus_write_command(self,device_id, register_address, register_values):
        """发送Modbus RTU写多个寄存器命令"""
        # 寄存器数量
        ser=self.client
        register_count = len(register_values)
 
        # 构建请求数据
        request = bytearray()
        request.append(device_id)
        request.append(0x10)  # 写多个寄存器功能码
        request.append((register_address >> 8) & 0xFF)
        request.append(register_address & 0xFF)
        request.append((register_count >> 8) & 0xFF)
        request.append(register_count & 0xFF)
        request.append(register_count * 2)  # 字节数
 
        # 添加寄存器值
        for value in register_values:
            request.append((value >> 8) & 0xFF)  # 高字节
            request.append(value & 0xFF)         # 低字节
 
        # 计算CRC校验
        crc = self.modbus_crc16(request)
        request.append((crc >> 8) & 0xFF)  # CRC高字节
        request.append(crc & 0xFF)         # CRC低字节
 
        # 发送请求
        ser.write(request)
        print(f"发送请求: {request.hex(' ')}")
 
        # 等待响应
        time.sleep(0.03)
 
        # 读取响应
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"接收响应: {response.hex(' ')}")
            return response
        return None

    def enable_TurboMode(self,device_id,en):

        ser=self.client
        # 构建请求数据
        request = bytearray()
        request.append(device_id)
        request.append(0x06)  # 写多个寄存器功能码
        request.append((1030 >> 8) & 0xFF)
        request.append((1030 >>0 ) & 0xFF)

        # 添加寄存器值
        request.append((en >> 8) & 0xFF)  # 高字节
        request.append((en >> 0) & 0xFF)  # 低字节

        # 计算CRC校验
        crc = self.modbus_crc16(request)
        request.append((crc >> 8) & 0xFF)  # CRC高字节
        request.append(crc & 0xFF)         # CRC低字节

        # 发送请求
        ser.write(request)
        print(f"发送请求 en Turbo: {request.hex(' ')}")

        # 等待响应
        time.sleep(0.03)

        # 读取响应
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"接收响应: {response.hex(' ')}")
            return response
         
    def read_position(self, hand_id):
        #response = self.client.read_input_registers(address=2000, count=6, slave=hand_id)
        response = self.send_modbus_read_command(hand_id, 2000, 6)
        return response

    def read_current(self, hand_id):
        #response = self.client.read_input_registers(address=2012, count=6, slave=hand_id)
        response = self.send_modbus_read_command(hand_id, 2012, 6)
        return response
        
    def write_position(self, hand_id, position_list):
        assert len(position_list)==6
        position_list = list(map(lambda x: max(0, min(100, x)), position_list)) #将每个数限制在0-100之间
        response = self.send_modbus_write_command(hand_id, 1010, position_list)

if __name__ == '__main__':
    hands = BraincoHands('/dev/ttyUSB_485')
    hands.enable_TurboMode(1,1)
    hands.enable_TurboMode(2,1)
    for i in range(100):
        print(f"loop : {i}\r\n")
        hands.write_position(1, [100]*6)
        time.sleep(1)
        print('position: ', hands.read_position(1))
        print('current: ', hands.read_current(1))
        #time.sleep(5)

        hands.write_position(1, [0]*6)
        time.sleep(1)
        print('position: ', hands.read_position(1))
        print('current: ', hands.read_current(1))
        #time.sleep(5)

        hands.write_position(2, [100]*6)
        time.sleep(1)
        print('position: ', hands.read_position(2))
        print('current: ', hands.read_current(2))
        #time.sleep(5)

        hands.write_position(2, [0]*6)
        time.sleep(1)
        print('position: ', hands.read_position(2))
        print('current: ', hands.read_current(2))
        #time.sleep(5)
    hands.CloseSerial()

