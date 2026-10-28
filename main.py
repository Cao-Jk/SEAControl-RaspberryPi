# -*- coding: utf-8 -*-
import sys
import os
import time
import numpy as np

# Qt / GUI
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLineEdit, QPushButton, QLineEdit, QLabel, QFrame
)

# 新增：共享 CAN 管理器、SEA 执行器
from canBus import CANBusManager
from seaActuator import SeaActuator
import rotateEncoder as rEncoder

class ForceFollowThread(QThread):
    """力跟随控制线程"""
    update_signal = pyqtSignal(float, float)  # 用于更新GUI显示

    def __init__(self, actuator, rot_encoder, amplitude, bias, mode_flag, assist_parameters):
        super().__init__()
        self.actuator = actuator
        self.rot_encoder = rot_encoder
        self.amplitude = amplitude
        self.bias = bias
        self.mode_flag = mode_flag
        self.theta_s1 = assist_parameters.get('theta_s1')
        self.theta_s2 = assist_parameters.get('theta_s2')
        self.s1 = assist_parameters.get('s1')
        self.s2 = assist_parameters.get('s2')
        self.B1 = assist_parameters.get('B1')
        self.B2 = assist_parameters.get('B2')
        self.transition_start_time = None  # 过渡阶段开始时间
        self.transition_duration = 3.0  # 过渡持续时间(秒)
        self.last_F = 0  # 记录上一次的F值
        self.running = True

    def run(self):
        
        while self.running:
            data = self.rot_encoder.read_encoder_data(
                device_addr=0x00, 
                want_angle=True,
                want_speed=False,
                want_direction=False
            )
            if data.get('angle') <165 and data.get('angle') > 0:  # 只在合理范围内工作
                angle_deg = data.get('angle')
            else:
                angle_deg = 0.0  # 超出范围则视为0度
        
            start_time = time.time()  # 放在while循环内角速度计算正确，但会导致控制不稳定
            speed_deg = self.rot_encoder.estimate_velocity(angle_deg, start_time)
            speed_rad = np.radians(speed_deg) # 角速度转为 rad/s
            theta = np.radians(angle_deg) # 转为弧度

            # 计算目标力
            if self.mode_flag == 1:  # 助力模式
                w = 1 / (1 + np.exp(-self.s1 * (theta - self.theta_s1)) * (1 + np.exp(-self.s1 * (self.theta_s2 - theta)))) # sigmoid函数
                G = self.amplitude * np.sin(theta) # 重力补偿项
                B = self.B1 / (1 + np.exp(-self.s2 * speed_rad)) + self.B2 * (1 - (1 / (1 + np.exp(-self.s2 * speed_rad)))) # 阻抗项
                target_force = -(w * (G + B * speed_rad) + (1 - w) * self.bias)
            else:  # 零力模式
                target_force = -self.bias

            # 平滑过渡处理
            current_time = time.time()
            if self.transition_start_time is None:
                self.transition_start_time = current_time
                transition_progress = 1.0  # 立即完成过渡
            else:
                transition_progress = min(1.0, (current_time - self.transition_start_time) / self.transition_duration)
            
            # 线性过渡
            F = self.last_F + (target_force - self.last_F) * transition_progress

            # 更新last_F
            if transition_progress >= 1.0:
                self.last_F = target_force
            else:
                self.last_F = F

            actual_force, torque_command = self.actuator.force_follow(F)
            self.update_signal.emit(actual_force, torque_command)



    def stop(self):
        self.running = False


class SEAControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # 默认参数：端口、刚度、PID等
        self.can_id = 1
        self.channel = 'can0'
        self.bustype = 'socketcan'
        self.baudrate = 1000000
        self.baudrate_rot = 115200
        self.timeout = 0.1
        self.timeout_rot = 0.005

        self.spring_stiffness = 47
        self.feedforward_parameter = 0.6
        self.pid_parameters = {'kp': 22.5, 'ki': 0.03, 'kd': 79.0}
        self.assist_parameters = {'theta_s1': np.radians(50.0), 'theta_s2': np.radians(130.0), 's1': 2.0, 's2': 5.0, 'B1': 5.0, 'B2': 15.0}
        '''theta_s1: 目标力软切换角度下限阈值
           theta_s2: 目标力软切换角度上限阈值
           s1: 指数函数敏感度系数
           s2: 指数函数敏感度系数
           B1: 阻尼系数，角速度为正时起作用
           B2: 阻尼系数，角速度为负时起作用，越大手臂向下运动时阻力越小
        '''

        self.port = '/dev/ttyAMA0'
        self.port_rot = '/dev/ttySC0'  # 旋转编码器串口

        # 模式标志
        self.mode_flag = 0  # 0: 零力, 1: 助力

        # 共享 CAN 总线实例（初始化时为 None）
        self.can_bus = None
        self.seaActuator = None
        self.rot_encoder = None
        self.init_ui()

    def init_ui(self):
        """初始化界面"""
        main_layout = QVBoxLayout()

        # 创建一个水平布局容器，用于居中按钮
        def create_centered_widget(widget):
            h_layout = QHBoxLayout()
            h_layout.addStretch(1)  # 左侧弹性空间
            h_layout.addWidget(widget)
            h_layout.addStretch(1)  # 右侧弹性空间
            return h_layout

        # 按钮1：初始化电机和传感器
        self.init_button = QPushButton("初始化电机和传感器")
        self.init_button.setFixedHeight(30) # 设置按钮高度
        self.init_button.setFixedWidth(200) # 设置按钮宽度
        self.init_button.setStyleSheet("background-color: lightgreen; color: black;") # 设置按钮颜色
        self.init_button.clicked.connect(self.initialize_devices)
        main_layout.addLayout(create_centered_widget(self.init_button))

        # 按钮2：获取传感器数据
        self.get_sensor_button = QPushButton("获取传感器数据")
        self.get_sensor_button.setFixedHeight(30) # 设置按钮高度
        self.get_sensor_button.setFixedWidth(200) # 设置按钮宽度
        self.get_sensor_button.setStyleSheet("background-color: lightgray; color: black;")  # 设置按钮颜色
        self.get_sensor_button.clicked.connect(self.get_linearEncoder_data)
        main_layout.addLayout(create_centered_widget(self.get_sensor_button))


        # 按钮4：关闭电机
        self.motor_off_button = QPushButton("关闭电机")
        self.motor_off_button.setFixedHeight(30)   # 设置按钮高度
        self.motor_off_button.setFixedWidth(200)  # 设置按钮宽度
        self.motor_off_button.clicked.connect(self.motor_off)
        main_layout.addLayout(create_centered_widget(self.motor_off_button))

        # 按钮5：停止力跟随
        self.stop_force_button = QPushButton("停止力跟随")
        self.stop_force_button.setFixedHeight(30) # 设置按钮高度
        self.stop_force_button.setFixedWidth(200) # 设置按钮宽度
        self.stop_force_button.setStyleSheet("background-color: red; color: white;")  # 设置按钮颜色
        self.stop_force_button.clicked.connect(self.stop_force_follow_thread)
        main_layout.addLayout(create_centered_widget(self.stop_force_button))

        # 添加分隔线
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine) # 水平线
        separator.setFrameShadow(QFrame.Sunken) # 设置阴影效果
        main_layout.addWidget(separator) # 添加分隔线

        # 按钮6：零力模式
        h_layout_zero = QHBoxLayout()
        h_layout_zero.addStretch(1)
        self.bias_input = QLineEdit()
        self.bias_input.setFixedWidth(200)
        self.bias_input.setFixedHeight(30)
        self.bias_input.setPlaceholderText("输入偏置")
        h_layout_zero.addWidget(self.bias_input)
        h_layout_zero.addStretch(1)
        main_layout.addLayout(h_layout_zero)

        self.free_mode_button = QPushButton("零力模式")
        self.free_mode_button.setFixedHeight(30)
        self.free_mode_button.setFixedWidth(200)
        self.free_mode_button.setStyleSheet("background-color: lightyellow; color: black;")
        self.free_mode_button.clicked.connect(self.free_follow)
        main_layout.addLayout(create_centered_widget(self.free_mode_button))

        # 按钮7：助力模式
        h_layout_assist = QHBoxLayout()
        h_layout_assist.addStretch(1)
        self.amplitude_input = QLineEdit()
        self.amplitude_input.setPlaceholderText("输入助力幅值")
        self.amplitude_input.setFixedWidth(200)
        self.amplitude_input.setFixedHeight(30)
        h_layout_assist.addWidget(self.amplitude_input)
        h_layout_assist.addStretch(1)
        main_layout.addLayout(h_layout_assist)

        self.assist_mode_button = QPushButton("助力模式")
        self.assist_mode_button.setFixedHeight(30)
        self.assist_mode_button.setFixedWidth(200)
        self.assist_mode_button.setStyleSheet("background-color: lightblue; color: black;")
        self.assist_mode_button.clicked.connect(self.assist_follow)
        main_layout.addLayout(create_centered_widget(self.assist_mode_button))

        # 设置主窗口
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)
        self.setWindowTitle("肩部助力外骨骼控制系统")
        self.resize(300, 600) 

    # ------------ 关键改动：只创建一次 CANBusManager，并传给 SEA ------------
    def initialize_devices(self):
        """初始化电机和传感器（共享一个 CANBusManager）"""
        try:
            # 仅首次创建共享总线
            if self.can_bus is None:
                self.can_bus = CANBusManager(
                    channel=self.channel,
                    bustype=self.bustype,
                    bitrate=self.baudrate,
                    bring_up=True,      # 仅第一次 bring up
                    manage_iface=True   # 退出时自动 ifconfig down
                )
                print("共享 CAN 总线初始化成功")


            # 构造 SEA（传入共享总线）
            self.seaActuator = SeaActuator(
                can_id=self.can_id,
                can_bus=self.can_bus,
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                spring_stiffness=self.spring_stiffness,
                feedforward_parameter=self.feedforward_parameter,
                pid_parameters=self.pid_parameters
            )
            print("Sea 初始化成功")

            # 构造旋转编码器
            self.rot_encoder = rEncoder.RotateEncoder(
                port=self.port_rot,
                baudrate=self.baudrate_rot,
                timeout=self.timeout_rot
            )
            print("旋转编码器初始化成功")

        except Exception as e:
            print(f"初始化失败: {e}")

    # -------------------------------------------------------------------------

    def motor_on(self):
        """打开电机"""
        if self.seaActuator:

            try:
                self.seaActuator.motor.motor_on()
                print("Sea电机已打开")
            except Exception as e:
                print(f"打开Sea电机失败: {e}")
        else:
            print("请先初始化Sea驱动器")

    def motor_off(self):
        """关闭电机"""
        if self.seaActuator:

            try:
                self.seaActuator.motor.motor_off()
                print("Sea电机已关闭")
            except Exception as e:
                print(f"关闭Sea电机失败: {e}")
        else:
            print("请先初始化Sea驱动器")

    def set_motor_I(self):
        """设置电机电流"""
        if self.seaActuator:
            try:
                target_I = int(float(self.target_I_input.text()))
            except Exception:
                print("目标电流输入无效")
                return

            try:
                self.seaActuator.motor.set_torque(target_I)
                print(f"Sea目标电流设置为: {target_I}")
            except Exception as e:
                print(f"设置Sea目标电流失败: {e}")
        else:
            print("请先初始化Sea驱动器")

    def get_linearEncoder_data(self):
        """获取线性编码器数据"""
        if self.seaActuator:

            try:
                position2 = self.seaActuator.linear_encoder.read_sensor_data()
                print(f"Sea线性编码器位置: {position2}")
            except Exception as e:
                print(f"获取Sea线性编码器数据失败: {e}")
        else:
            print("请先初始化Sea驱动器")

    def assist_follow(self):
        """让执行器以目标力值进行力跟随"""
        self.mode_flag = 1  # 助力模式
        # 停止之前的力跟随线程（如果有）
        self.stop_force_follow_thread()

        if self.seaActuator:
            try:
                self.amplitude = float(self.amplitude_input.text())
                self.bias = float(self.bias_input.text()) if self.bias_input.text() else 10.0
            except ValueError:
                print("目标力输入无效，请输入数字")
                return

            # 创建并启动力跟随线程
            self.assist_thread = ForceFollowThread(
                self.seaActuator, 
                self.rot_encoder, 
                self.amplitude, 
                self.bias, 
                self.mode_flag, 
                self.assist_parameters
            )
            self.assist_thread.start()
            print("助力模式已启动")
        else:
            print("请先初始化Sea驱动器")

    def free_follow(self):
        """让执行器以目标力值进行力跟随"""
        self.mode_flag = 0  # 零力模式
        # 停止之前的力跟随线程（如果有）
        self.stop_force_follow_thread()

        if self.seaActuator:
            try:
                self.amplitude = 0.0
                self.bias = float(self.bias_input.text()) if self.bias_input.text() else 10.0
            except ValueError:
                print("目标力输入无效，请输入数字")
                return

            # 创建并启动力跟随线程
            self.free_thread = ForceFollowThread(
                self.seaActuator,
                self.rot_encoder, 
                self.amplitude, 
                self.bias, 
                self.mode_flag, 
                self.assist_parameters
            )
            self.free_thread.start()
            print("零力模式已启动")
        else:
            print("请先初始化Sea驱动器")

    def stop_force_follow_thread(self):
        """停止力跟随控制线程"""
        try:
            if hasattr(self, 'assist_thread'):
                self.assist_thread.stop()
                self.assist_thread.wait()
                print("助力控制线程已停止")
                self.motor_off()
            if hasattr(self, 'free_thread'):
                self.free_thread.stop()
                self.free_thread.wait()
                print("零力控制线程已停止")
                self.motor_off()
        except Exception as e:
            print(f"停止控制线程失败: {e}")

    # 建议：窗口关闭时，收尾处理（关闭电机 -> 关串口 -> 关 CAN -> 关编码器）
    def closeEvent(self, event):
        try:
            if self.seaActuator:
                self.seaActuator.close()
            if self.can_bus:
                self.can_bus.shutdown()
            if self.rot_encoder:
                self.rot_encoder.close()
            print("资源已关闭")
        except Exception as e:
            print(f"关闭资源时异常: {e}")
        super().closeEvent(event)


if __name__ == '__main__':
    # 强制使用 xcb 平台插件（与原始工程一致）
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

    app = QApplication(sys.argv)
    gui = SEAControllerGUI()
    gui.show()
    sys.exit(app.exec_())
