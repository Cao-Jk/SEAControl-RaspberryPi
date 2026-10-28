# -*- coding: utf-8 -*-
"""
linearEncoder.py
统一线性编码器管理类：
port='/dev/ttyAMA0'对应stm32的A2、A3分别连接树莓派RX、TX
port='/dev/ttyAMA2'对应stm32的A2、A3分别连接树莓派D5、D4
"""

import serial
import time

class LinearEncoder:
    def __init__(self, port='/dev/ttyAMA2', baudrate=1000000, timeout=0.1):
        """
        初始化串口配置
        :param port: 串口端口号
        :param baudrate: 波特率
        :param timeout: 超时时间（秒）
        """
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def read_sensor_data(self):
        """
        从传感器读取数据，并解析为十进制值
        :return: 解析后的单次传感器值
        """
        # 读取数据（2 字节）
        data = self.ser.read(2)
       # print(f"Received data (hex): {data.hex()}")
        
        # 调整字节顺序，低位在前，高位在后
        low_byte, high_byte = data[0], data[1]
        
        # 拼接成 16 进制字符串，低位在前，高位在后
        hex_value = f"{high_byte:02X}{low_byte:02X}"
        
        # 将拼接的 16 进制字符串转换为十进制
        decimal_value = int(hex_value, 16)
        if decimal_value>30000:decimal_value-=65536
        decimal_value/=100
        # 打印转换后的十进制值
        #print(f"Converted decimal value: {decimal_value}")
        self.ser.reset_input_buffer()
        return -decimal_value
        #负数！！！！
        #time.sleep(0.003)
        

    def reset_buffer(self):
        """
        清空串口输入缓存区
        """
        self.ser.reset_input_buffer()

    def close(self):
        """
        关闭串口
        """
        self.ser.close()

# 使用示例
if __name__ == "__main__":
    reader1 = LinearEncoder(port='/dev/ttyAMA0', baudrate=1000000)
    # reader2 = LinearEncoder(port='/dev/ttyAMA2', baudrate=1000000)
    # last_time = 0
    # i=0
    
    try:
        while True:
            # i+=1
            # if i==10:
            #     i=0
            #     current_time=time.time()
            #     delta_time=current_time-last_time
            #     last_time = current_time
                # print(delta_time)
            print(f"SEA1:{reader1.read_sensor_data()}")
            # print(f"SEA2:{reader2.read_sensor_data()}")
            # time.sleep(1)
            
    except KeyboardInterrupt:
        reader1.close()
        # reader2.close()