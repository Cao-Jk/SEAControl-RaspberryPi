# -*- coding: utf-8 -*-
"""
seaActuator.py
SEA 执行器：组合 Motor 和 LinearEncoder；通过共享的 CANBusManager 工作。
"""
from typing import Dict, Any, Tuple
import linearEncoder as lEncoder
from motor import Motor

class SeaActuator:
    def __init__(
        self,
        can_id: int,
        can_bus,
        port: str = '/dev/ttyAMA0',
        baudrate: int = 1000000,
        timeout: float = 0.1,
        spring_stiffness: float = 46.0,
        feedforward_parameter: float = 0.45,
        pid_parameters: Dict[str, float] = None,
    ) -> None:
        # 传感器
        self.linear_encoder = lEncoder.LinearEncoder(port, baudrate, timeout)
        # 电机
        self.motor = Motor(can_id=can_id, can_bus=can_bus)

        self.spring_stiffness = float(spring_stiffness)
        self.feedforward_parameter = float(feedforward_parameter)
        self.pid_parameters = pid_parameters or {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}

        self._pid_integral = 0.0
        self._prev_e = 0.0

    def set_pid_parameters(self, kp, ki, kd) -> None:
        self.pid_parameters['kp'] = float(kp)
        self.pid_parameters['ki'] = float(ki)
        self.pid_parameters['kd'] = float(kd)

    def force_follow(self, target_force: float) -> Tuple[float, float]:
        # 读取位移 -> 力
        x = self.linear_encoder.read_sensor_data()
        actual_force = self.spring_stiffness * x

        # PID + 前馈
        kp = float(self.pid_parameters.get('kp', 0.0))
        ki = float(self.pid_parameters.get('ki', 0.0))
        kd = float(self.pid_parameters.get('kd', 0.0))

        error = float(target_force) - float(actual_force)
        self._pid_integral = max(min(self._pid_integral + error, 1000.0), -1000.0)
        delta_e = error - self._prev_e
        self._prev_e = error

        u = self.feedforward_parameter * float(target_force) + kp*error + ki*self._pid_integral + kd*delta_e
        u = max(min(u, 1200.0), -1200.0)

        # 下发到电机
        self.motor.set_torque(int(u))
        return actual_force, u

    def close(self) -> None:
        try:
            self.linear_encoder.close()
        except Exception:
            pass
        try:
            self.motor.motor_off()
        except Exception:
            pass
