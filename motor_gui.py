# -*- coding: utf-8 -*-
import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox, QComboBox
)
from PyQt5.QtCore import Qt

# 使用你项目里的总线与电机封装
from canBus import CANBusManager           # 统一 CAN 管理器（只建一次）  ← 依赖你的文件
from motor import Motor                    # 电机命令封装                 ← 依赖你的文件


class MotorGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # —— 默认通信参数（按需改成你的实际环境）——
        self.bustype  = 'socketcan'   # Linux: 'socketcan'; 例如 PCAN/Vector 等请换成相应值
        self.channel  = 'can0'        # Linux 常用 can0；Windows/硬件改成你的通道名
        self.bitrate  = 1_000_000
        self.can_id   = 1             # 和实际电机ID一致

        # 运行期对象
        self.can_bus  = None
        self.motor    = None

        self._build_ui()

    def _build_ui(self):
        self.setWindowTitle("SEA 电机控制（启动/停止/设定扭矩）")
        self.setMinimumWidth(420)

        root = QWidget()
        col  = QVBoxLayout(root)
        col.setSpacing(10)

        # —— 连接/通道设置行 ——
        row_bus = QHBoxLayout()
        row_bus.addWidget(QLabel("总线类型:"))
        self.cmb_bus = QComboBox()
        self.cmb_bus.addItems(["socketcan"])   # 如需 PCAN/Vector，请在此添加
        self.cmb_bus.setCurrentText(self.bustype)
        row_bus.addWidget(self.cmb_bus)

        row_bus.addWidget(QLabel("通道:"))
        self.edt_channel = QLineEdit(self.channel)
        self.edt_channel.setFixedWidth(120)
        row_bus.addWidget(self.edt_channel)

        row_bus.addWidget(QLabel("bitrate:"))
        self.edt_bitrate = QLineEdit(str(self.bitrate))
        self.edt_bitrate.setFixedWidth(100)
        row_bus.addWidget(self.edt_bitrate)

        self.btn_connect = QPushButton("连接 CAN")
        self.btn_connect.clicked.connect(self.on_connect)
        row_bus.addWidget(self.btn_connect)
        col.addLayout(row_bus)

        # —— 电机ID设置行 ——
        row_id = QHBoxLayout()
        row_id.addWidget(QLabel("电机ID:"))
        self.edt_id = QLineEdit(str(self.can_id))
        self.edt_id.setFixedWidth(80)
        row_id.addWidget(self.edt_id)
        self.lbl_status = QLabel("未连接")
        self.lbl_status.setStyleSheet("color: #999;")
        row_id.addWidget(self.lbl_status, 1, Qt.AlignRight)
        col.addLayout(row_id)

        # —— 功能按钮行：启动/停止/急停 ——
        row_btn = QHBoxLayout()
        self.btn_on  = QPushButton("启动电机")
        self.btn_off = QPushButton("关闭电机")
        self.btn_stop = QPushButton("急停")
        self.btn_on.clicked.connect(self.on_motor_on)
        self.btn_off.clicked.connect(self.on_motor_off)
        self.btn_stop.clicked.connect(self.on_motor_stop)
        for b in (self.btn_on, self.btn_off, self.btn_stop):
            b.setEnabled(False)
        row_btn.addWidget(self.btn_on)
        row_btn.addWidget(self.btn_off)
        row_btn.addWidget(self.btn_stop)
        col.addLayout(row_btn)

        # —— 扭矩设定行 ——
        row_t = QHBoxLayout()
        row_t.addWidget(QLabel("设定扭矩/电流 (iq, -2000…2000):"))
        self.edt_torque = QLineEdit("0")
        self.edt_torque.setPlaceholderText("整数，单位与驱动一致")
        self.edt_torque.setFixedWidth(140)
        row_t.addWidget(self.edt_torque)

        self.btn_apply_torque = QPushButton("下发扭矩")
        self.btn_apply_torque.setEnabled(False)
        self.btn_apply_torque.clicked.connect(self.on_set_torque)
        row_t.addWidget(self.btn_apply_torque)
        col.addLayout(row_t)

        self.setCentralWidget(root)

    # ===== 事件处理 =====
    def on_connect(self):
        if self.can_bus is not None:
            QMessageBox.information(self, "提示", "CAN 已连接。")
            return

        # 读取界面参数
        try:
            self.bustype = self.cmb_bus.currentText().strip()
            self.channel = self.edt_channel.text().strip()
            self.bitrate = int(self.edt_bitrate.text().strip())
            self.can_id  = int(self.edt_id.text().strip())
        except Exception:
            QMessageBox.warning(self, "参数错误", "请检查 bustype/channel/bitrate/电机ID。")
            return

        try:
            # 只创建一次 CAN 管理器（内部会 bring up 接口并打开 python-can 总线）
            self.can_bus = CANBusManager(
                channel=self.channel,
                bustype=self.bustype,
                bitrate=self.bitrate,
                bring_up=True,
                manage_iface=True
            )
            # 创建 Motor；注意 motor.__init__ 需要与实际电机ID一致
            self.motor = Motor(can_id=self.can_id, can_bus=self.can_bus)
        except Exception as e:
            self.can_bus = None
            self.motor = None
            QMessageBox.critical(self, "连接失败", f"初始化 CAN/Motor 失败：\n{e}")
            self.lbl_status.setText("未连接")
            self.lbl_status.setStyleSheet("color: #c00;")
            return

        # UI 状态
        self.lbl_status.setText("已连接")
        self.lbl_status.setStyleSheet("color: #090;")
        for b in (self.btn_on, self.btn_off, self.btn_stop, self.btn_apply_torque):
            b.setEnabled(True)

    def on_motor_on(self):
        if not self.motor:
            return
        try:
            self.motor.motor_on()   # 通电使能
            QMessageBox.information(self, "成功", "电机已启动（上电使能）。")
        except Exception as e:
            QMessageBox.critical(self, "失败", f"启动失败：\n{e}")

    def on_motor_off(self):
        if not self.motor:
            return
        try:
            self.motor.motor_off()  # 下电/失能
            QMessageBox.information(self, "成功", "电机已关闭。")
        except Exception as e:
            QMessageBox.critical(self, "失败", f"关闭失败：\n{e}")

    def on_motor_stop(self):
        if not self.motor:
            return
        try:
            self.motor.motor_stop()  # 急停命令
            QMessageBox.information(self, "成功", "已急停。")
        except Exception as e:
            QMessageBox.critical(self, "失败", f"急停失败：\n{e}")

    def on_set_torque(self):
        if not self.motor:
            return
        try:
            iq = int(self.edt_torque.text().strip())
        except Exception:
            QMessageBox.warning(self, "输入无效", "扭矩/电流必须是整数（-2000…2000）。")
            return
        try:
            # Motor.set_torque 内部已校验范围 [-2000, 2000]
            self.motor.set_torque(iq)
            QMessageBox.information(self, "成功", f"已下发扭矩/电流：{iq}")
        except ValueError as ve:
            # 超范围会抛 ValueError（你现有实现即如此）
            QMessageBox.warning(self, "范围错误", str(ve))
        except Exception as e:
            QMessageBox.critical(self, "下发失败", f"发送失败：\n{e}")

    # 资源回收：关电机 -> 关 CAN
    def closeEvent(self, event):
        try:
            if self.motor:
                try:
                    self.motor.motor_off()
                except Exception:
                    pass
            if self.can_bus:
                try:
                    self.can_bus.shutdown()
                except Exception:
                    pass
        finally:
            super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MotorGUI()
    w.show()
    sys.exit(app.exec_())
