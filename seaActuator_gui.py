# -*- coding: utf-8 -*-
import os
import sys
import time
import math
import csv
from datetime import datetime
from collections import deque

from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QGridLayout, QFrame,
    QFileDialog
)

# ===== 依赖你已有的模块 =====
from canBus import CANBusManager
from seaActuator import SeaActuator
from motor import Motor
from linearEncoder import LinearEncoder

# ===== matplotlib 内嵌到 PyQt5 =====
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure


class ForcePlotWidget(QWidget):
    """曲线：目标力 vs 实际力（单轴两条线），带工具栏缩放"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(6, 3), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Force [N]")

        (self.line_target,) = self.ax.plot([], [], label="Target F")
        (self.line_actual,) = self.ax.plot([], [], label="Actual F")
        self.ax.legend(loc="upper right")

        lay = QVBoxLayout(self)
        self.toolbar = NavigationToolbar(self.canvas, self)
        lay.addWidget(self.toolbar)
        lay.addWidget(self.canvas)
        lay.setContentsMargins(0, 0, 0, 0)

    def plot_static(self, t, target, actual):
        """一次性静态绘制（全时域）"""
        self.line_target.set_data(t, target)
        self.line_actual.set_data(t, actual)

        if len(t) >= 2:
            xmin, xmax = t[0], t[-1]
            self.ax.set_xlim(xmin, xmax)

            ymin = min(min(target), min(actual)) if target and actual else 0.0
            ymax = max(max(target), max(actual)) if target and actual else 1.0
            if ymin == ymax:
                ymax = ymin + 1.0
            margin = 0.05 * (ymax - ymin)
            self.ax.set_ylim(ymin - margin, ymax + margin)

        self.canvas.draw_idle()


class ErrorPlotWidget(QWidget):
    """曲线：误差 e(t)=Target-Actual，带工具栏缩放"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(6, 2.6), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Error [N]")

        (self.line_err,) = self.ax.plot([], [], label="Error")
        self.ax.legend(loc="upper right")

        lay = QVBoxLayout(self)
        self.toolbar = NavigationToolbar(self.canvas, self)
        lay.addWidget(self.toolbar)
        lay.addWidget(self.canvas)
        lay.setContentsMargins(0, 0, 0, 0)

    def plot_static(self, t, err):
        self.line_err.set_data(t, err)
        if len(t) >= 2:
            xmin, xmax = t[0], t[-1]
            self.ax.set_xlim(xmin, xmax)

            if len(err) > 0:
                ymin, ymax = min(err), max(err)
            else:
                ymin, ymax = 0.0, 1.0
            if ymin == ymax:
                ymax = ymin + 1.0
            margin = 0.05 * (ymax - ymin)
            self.ax.set_ylim(ymin - margin, ymax + margin)

        self.canvas.draw_idle()


class PlotWindow(QWidget):
    """独立顶层窗口（非模态），上->目标/实际，下->误差。一次性静态绘制"""
    def __init__(self, t, target, actual, err):
        super().__init__(None, Qt.Window)
        self.setWindowModality(Qt.NonModal)
        self.setWindowTitle("波形回放（Target/Actual & Error）")
        self.resize(900, 700)

        vbox = QVBoxLayout(self)
        self.force_plot = ForcePlotWidget(self)
        self.error_plot = ErrorPlotWidget(self)
        vbox.addWidget(self.force_plot)
        vbox.addWidget(self.error_plot)
        self.setLayout(vbox)

        # 绘制
        self.force_plot.plot_static(t, target, actual)
        self.error_plot.plot_static(t, err)


class SineFollowThread(QThread):
    """正弦力目标跟随线程：F(t) = A*sin(2π f t) + b"""
    feedback_signal = pyqtSignal(float, float, float, float)  # target, actual, torque_cmd, t

    def __init__(self, sea: SeaActuator, A: float, f: float, b: float, dt: float = 0.005, parent=None):
        super().__init__(parent)
        self.sea = sea
        self.A = float(A)
        self.f = float(f)
        self.b = float(b)
        self.dt = float(dt)
        self.running = True

    def run(self):
        t0 = time.time()
        while self.running:
            t = time.time() - t0
            target_F = self.A * math.sin(2.0 * math.pi * self.f * t) + self.b
            actual_force, torque_cmd = self.sea.force_follow(target_F)
            self.feedback_signal.emit(target_F, actual_force, torque_cmd, t)
            time.sleep(self.dt)

    def stop(self):
        self.running = False
        self.wait()


class SEAQtGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # --- 默认通信/控制参数 ---
        self.channel = 'can0'
        self.bustype = 'socketcan'
        self.bitrate = 1_000_000
        self.can_id = 1

        self.le_port = '/dev/ttyAMA0'
        self.le_baud = 1_000_000
        self.le_timeout = 0.1

        self.spring_k = 46.0
        self.feedforward = 0.5
        self.pid = dict(kp=0.0, ki=0.0, kd=0.0)

        # 共享资源
        self.can_bus = None
        self.sea = None
        self.motor = None

        # 线程
        the_thread = None
        self.sine_thread = the_thread

        # 数据缓存（只缓存，不绘图）
        self.buf_len = 6000  # 约 30s@dt=0.005
        self.t_buf = deque(maxlen=self.buf_len)
        self.target_buf = deque(maxlen=self.buf_len)
        self.actual_buf = deque(maxlen=self.buf_len)
        self.err_buf = deque(maxlen=self.buf_len)

        # 记忆上次保存目录
        self.last_save_dir = os.path.join(os.getcwd(), "logs")
        os.makedirs(self.last_save_dir, exist_ok=True)

        self._build_ui()

    # ------------------ UI ------------------
    def _build_ui(self):
        self.setWindowTitle("SEA 驱动器控制面板")
        self.resize(900, 680)

        root = QWidget(self)
        lay = QVBoxLayout(root)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(10)

        # ========== 初始化区 ==========
        g_init = QGroupBox("1) 设备初始化")
        grid0 = QGridLayout(g_init)

        self.ed_can_id = QLineEdit(str(self.can_id)); self.ed_can_id.setPlaceholderText("CAN ID")
        self.ed_channel = QLineEdit(self.channel);     self.ed_channel.setPlaceholderText("can 通道，如 can0")
        self.ed_bitrate = QLineEdit(str(self.bitrate));self.ed_bitrate.setPlaceholderText("bitrate")
        self.ed_le_port = QLineEdit(self.le_port);     self.ed_le_port.setPlaceholderText("线性编码器串口")
        self.ed_le_baud = QLineEdit(str(self.le_baud));self.ed_le_baud.setPlaceholderText("波特率")
        self.ed_le_to = QLineEdit(str(self.le_timeout)); self.ed_le_to.setPlaceholderText("超时(s)")

        self.btn_init = QPushButton("一键初始化（电机+线性编码器）")
        self.btn_init.clicked.connect(self.on_init_clicked)

        grid0.addWidget(QLabel("CAN ID"), 0, 0); grid0.addWidget(self.ed_can_id, 0, 1)
        grid0.addWidget(QLabel("CAN 通道"), 0, 2); grid0.addWidget(self.ed_channel, 0, 3)
        grid0.addWidget(QLabel("CAN 速率"), 1, 0); grid0.addWidget(self.ed_bitrate, 1, 1)
        grid0.addWidget(QLabel("线编串口"), 1, 2); grid0.addWidget(self.ed_le_port, 1, 3)
        grid0.addWidget(QLabel("线编波特率"), 2, 0); grid0.addWidget(self.ed_le_baud, 2, 1)
        grid0.addWidget(QLabel("线编超时"), 2, 2); grid0.addWidget(self.ed_le_to, 2, 3)
        grid0.addWidget(self.btn_init, 3, 0, 1, 4)
        lay.addWidget(g_init)

        # 分隔线
        sep1 = QFrame(); sep1.setFrameShape(QFrame.HLine); lay.addWidget(sep1)

        # ========== PID & 前馈 & 扭矩/读数 ==========
        g_pid = QGroupBox("2) PID 参数 / 前馈系数 / 直接扭矩 / 读取线性编码器 / 电机控制")
        grid1 = QGridLayout(g_pid)

        self.ed_kp = QLineEdit(str(self.pid['kp']))
        self.ed_ki = QLineEdit(str(self.pid['ki']))
        self.ed_kd = QLineEdit(str(self.pid['kd']))
        self.ed_ff = QLineEdit(str(self.feedforward))

        self.btn_set_pid = QPushButton("应用 PID + 前馈")
        self.btn_set_pid.clicked.connect(self.on_apply_pid_ff)

        self.ed_iq = QLineEdit("0")
        self.btn_set_iq = QPushButton("设置电机扭矩(iq)")
        self.btn_set_iq.clicked.connect(self.on_set_iq)

        self.btn_motor_off = QPushButton("关闭电机")
        self.btn_motor_off.clicked.connect(self.on_motor_off)

        self.btn_read_le = QPushButton("读取线性编码器")
        self.btn_read_le.clicked.connect(self.on_read_le)
        self.lb_le = QLabel("线编：-- mm")

        # 两行布局
        grid1.addWidget(QLabel("Kp"), 0, 0); grid1.addWidget(self.ed_kp, 0, 1)
        grid1.addWidget(QLabel("Ki"), 0, 2); grid1.addWidget(self.ed_ki, 0, 3)

        grid1.addWidget(QLabel("Kd"), 1, 0); grid1.addWidget(self.ed_kd, 1, 1)
        grid1.addWidget(QLabel("FF"), 1, 2); grid1.addWidget(self.ed_ff, 1, 3)
        grid1.addWidget(self.btn_set_pid, 1, 4)

        grid1.addWidget(QLabel("iq"), 2, 0); grid1.addWidget(self.ed_iq, 2, 1)
        grid1.addWidget(self.btn_set_iq, 2, 2, 1, 2)
        grid1.addWidget(self.btn_motor_off, 2, 4)
        grid1.addWidget(self.btn_read_le, 2, 5)
        grid1.addWidget(self.lb_le, 2, 6, 1, 3)
        lay.addWidget(g_pid)

        # 分隔线
        sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine); lay.addWidget(sep2)

        # ========== 正弦力设定 ==========
        g_sine = QGroupBox("3) 正弦目标力曲线 F(t)=A·sin(2π f t)+b")
        grid2 = QGridLayout(g_sine)

        self.ed_A = QLineEdit("200.0"); self.ed_A.setPlaceholderText("幅值 A [N]")
        self.ed_f = QLineEdit("1.0");  self.ed_f.setPlaceholderText("频率 f [Hz]")
        self.ed_b = QLineEdit("0.0");  self.ed_b.setPlaceholderText("偏置 b [N]")

        self.btn_sine_start = QPushButton("启动跟随")
        self.btn_sine_stop = QPushButton("停止跟随")
        self.btn_sine_start.clicked.connect(self.on_sine_start)
        self.btn_sine_stop.clicked.connect(self.on_sine_stop)

        grid2.addWidget(QLabel("A[N]"), 0, 0); grid2.addWidget(self.ed_A, 0, 1)
        grid2.addWidget(QLabel("f[Hz]"), 0, 2); grid2.addWidget(self.ed_f, 0, 3)
        grid2.addWidget(QLabel("b[N]"), 0, 4); grid2.addWidget(self.ed_b, 0, 5)
        grid2.addWidget(self.btn_sine_start, 0, 6)
        grid2.addWidget(self.btn_sine_stop, 0, 7)
        lay.addWidget(g_sine)

        # 分隔线
        sep3 = QFrame(); sep3.setFrameShape(QFrame.HLine); lay.addWidget(sep3)

        # ========== 波形回放 / 数据保存 ==========
        g_plot_ctrl = QGroupBox("4) 波形回放（停止后绘制） / 数据保存")
        h_ctrl = QHBoxLayout(g_plot_ctrl)

        self.btn_plot_once = QPushButton("绘制上一次曲线")
        self.btn_plot_once.clicked.connect(self.draw_last_curve_once)
        h_ctrl.addWidget(self.btn_plot_once)

        self.btn_save_csv = QPushButton("保存当前曲线到CSV...")
        self.btn_save_csv.clicked.connect(self.save_csv_now)
        h_ctrl.addWidget(self.btn_save_csv)

        lay.addWidget(g_plot_ctrl)

        root.setLayout(lay)
        self.setCentralWidget(root)

    # ------------------ 逻辑 ------------------
    def on_init_clicked(self):
        """一键初始化：CAN 总线 -> SEA（含 Motor & LinearEncoder）"""
        try:
            can_id = int(self.ed_can_id.text().strip())
            channel = self.ed_channel.text().strip()
            bitrate = int(self.ed_bitrate.text().strip())
            le_port = self.ed_le_port.text().strip()
            le_baud = int(self.ed_le_baud.text().strip())
            le_to = float(self.ed_le_to.text().strip())

            if self.can_bus is None:
                self.can_bus = CANBusManager(
                    channel=channel, bustype=self.bustype,
                    bitrate=bitrate, bring_up=True, manage_iface=True
                )

            self.sea = SeaActuator(
                can_id=can_id, can_bus=self.can_bus,
                port=le_port, baudrate=le_baud, timeout=le_to,
                spring_stiffness=self.spring_k,
                feedforward_parameter=self.feedforward,
                pid_parameters=self.pid
            )

            self.motor = self.sea.motor
            self.statusBar().showMessage("初始化成功", 3000)
        except Exception as e:
            self.statusBar().showMessage(f"初始化失败：{e}", 5000)

    def on_apply_pid_ff(self):
        """同时应用 PID 与前馈系数"""
        if not self.sea:
            self.statusBar().showMessage("请先初始化设备", 3000); return
        try:
            kp = float(self.ed_kp.text()); ki = float(self.ed_ki.text()); kd = float(self.ed_kd.text())
            ff = float(self.ed_ff.text())
            self.sea.set_pid_parameters(kp, ki, kd)
            self.pid.update(kp=kp, ki=ki, kd=kd)

            applied = False
            try:
                if hasattr(self.sea, "set_feedforward_parameter"):
                    self.sea.set_feedforward_parameter(ff)
                    applied = True
            except Exception:
                pass
            if not applied and hasattr(self.sea, "feedforward_parameter"):
                try:
                    setattr(self.sea, "feedforward_parameter", ff)
                    applied = True
                except Exception:
                    pass

            self.feedforward = ff
            if applied:
                self.statusBar().showMessage(f"PID/FF 应用成功：Kp={kp}, Ki={ki}, Kd={kd}, FF={ff}", 4000)
            else:
                self.statusBar().showMessage(f"PID 已应用（FF={ff} 未能设置：请确认 SeaActuator 是否提供接口）", 5000)
        except Exception as e:
            self.statusBar().showMessage(f"应用 PID/FF 失败：{e}", 5000)

    def on_set_iq(self):
        if not self.motor:
            self.statusBar().showMessage("请先初始化设备", 3000); return
        try:
            iq = int(float(self.ed_iq.text()))
            self.motor.set_torque(iq)
            self.statusBar().showMessage(f"已设置电机扭矩(iq) = {iq}", 3000)
        except Exception as e:
            self.statusBar().showMessage(f"设置 iq 失败：{e}", 5000)

    def on_motor_off(self):
        if not self.motor:
            self.statusBar().showMessage("请先初始化设备", 3000); return
        try:
            self.motor.motor_off()
            self.statusBar().showMessage("电机已关闭（motor_off）", 4000)
        except Exception as e:
            self.statusBar().showMessage(f"关闭电机失败：{e}", 5000)

    def on_read_le(self):
        if self.sine_thread and self.sine_thread.isRunning():
            self.statusBar().showMessage("请先停止跟随，再读取线性编码器", 4000)
            return
        if not self.sea:
            self.statusBar().showMessage("请先初始化设备", 3000); return
        try:
            x_mm = self.sea.linear_encoder.read_sensor_data()
            self.lb_le.setText(f"线编：{x_mm:.3f} mm")
        except Exception as e:
            self.statusBar().showMessage(f"读取线编失败：{e}", 5000)

    def on_sine_start(self):
        if not self.sea:
            self.statusBar().showMessage("请先初始化设备", 3000); return
        # 停掉旧线程
        self.on_sine_stop()

        try:
            A = float(self.ed_A.text()); f = float(self.ed_f.text()); b = float(self.ed_b.text())
        except Exception:
            self.statusBar().showMessage("请输入合法的 A/f/b 数值", 4000); return

        # 清空缓存
        self.t_buf.clear(); self.target_buf.clear(); self.actual_buf.clear(); self.err_buf.clear()

        self.sine_thread = SineFollowThread(self.sea, A, f, b, dt=0.005)
        self.sine_thread.feedback_signal.connect(self.on_feedback, Qt.QueuedConnection)
        self.sine_thread.start()
        self.statusBar().showMessage("正弦力跟随已启动（离线缓存，停止后再绘制）", 4000)

    def on_feedback(self, target_F: float, actual_F: float, u: float, t: float):
        # 只缓存，不做任何实时显示
        self.t_buf.append(t)
        self.target_buf.append(target_F)
        self.actual_buf.append(actual_F)
        self.err_buf.append(target_F - actual_F)

    def on_sine_stop(self):
        if self.sine_thread:
            try:
                self.sine_thread.stop()
            except Exception:
                pass
            self.sine_thread = None
        self.statusBar().showMessage("正弦力跟随已停止", 2000)

    def draw_last_curve_once(self):
        """停止后，一次性绘制最近一次缓存曲线"""
        if self.sine_thread and self.sine_thread.isRunning():
            self.statusBar().showMessage("请先停止跟随，再绘制曲线。", 4000)
            return
        if len(self.t_buf) < 2:
            self.statusBar().showMessage("没有足够数据可绘制，请先运行一次跟随。", 4000)
            return

        t = list(self.t_buf)
        target = list(self.target_buf)
        actual = list(self.actual_buf)
        err = list(self.err_buf)

        self.plot_window = PlotWindow(t, target, actual, err)
        self.plot_window.show()
        self.statusBar().showMessage("已绘制上一次曲线（独立窗口可缩放/保存）。", 4000)

    def save_csv_now(self):
        """弹出文件对话框，自定义保存路径；写出 t/target/actual/error."""
        if len(self.t_buf) == 0:
            self.statusBar().showMessage("没有数据可保存，请先运行一次跟随。", 4000)
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"sea_log_{ts}.csv"
        out_path, _ = QFileDialog.getSaveFileName(
            self, "保存曲线到 CSV",
            os.path.join(self.last_save_dir, default_name),
            "CSV 文件 (*.csv)"
        )
        if not out_path:
            return

        self.last_save_dir = os.path.dirname(out_path)
        if not out_path.lower().endswith(".csv"):
            out_path += ".csv"

        t_list = list(self.t_buf)
        tgt_list = list(self.target_buf)
        act_list = list(self.actual_buf)
        err_list = list(self.err_buf)

        try:
            with open(out_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "target_N", "actual_N", "error_N"])
                for row in zip(t_list, tgt_list, act_list, err_list):
                    w.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}"])
            self.statusBar().showMessage(f"CSV 已保存：{out_path}", 6000)
        except Exception as e:
            self.statusBar().showMessage(f"保存 CSV 失败：{e}", 6000)

    # ------------------ 资源回收 ------------------
    def closeEvent(self, e):
        try:
            self.on_sine_stop()
            if self.sea:
                self.sea.close()     # motor_off + 关闭线编
            if self.can_bus:
                self.can_bus.shutdown()
        except Exception:
            pass
        super().closeEvent(e)


if __name__ == "__main__":
    # 如在树莓派遇到 Wayland/GL 报错，可在启动前设置：
    #   export QT_QPA_PLATFORM=xcb
    #   export QT_OPENGL=software
    # 或在此处启用：
    # os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    # os.environ.setdefault('QT_OPENGL', 'software')

    app = QApplication(sys.argv)
    w = SEAQtGUI()
    w.show()
    sys.exit(app.exec_())
