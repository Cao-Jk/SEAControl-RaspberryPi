# -*- coding: utf-8 -*-
"""
motor.py
电机底层命令类：依赖统一的 CANBusManager，不再自行初始化 can 接口。
SEA1的电机ID已设置为1
SEA2的电机ID已设置为2
初始化Motor实例时设置的can_id必须与实际电机ID一致，否则无法正常通信
"""
from typing import Optional
import can
from canBus import CANBusManager

class Motor:
    def __init__(self, can_id: int, can_bus: CANBusManager):
        self.can_id = int(can_id)
        self.can_bus = can_bus
        self.current_torque = 0

    # --------- 基础发送工具 ---------
    def _send_hex(self, hex_str: str) -> None:
        data = bytes.fromhex(hex_str)
        msg = can.Message(arbitration_id=0x140 + self.can_id, data=data, is_extended_id=False)
        self.can_bus.send(msg)

    def _send_data(self, arb_id: int, data_bytes) -> None:
        msg = can.Message(arbitration_id=arb_id, data=bytes(data_bytes), is_extended_id=False)
        self.can_bus.send(msg)

    # --------- 电机命令 ---------
    def motor_on(self) -> None:
        self._send_hex("88 00 00 00 00 00 00 00")

    def motor_off(self) -> None:
        self._send_hex("80 00 00 00 00 00 00 00")

    def motor_stop(self) -> None:
        self._send_hex("81 00 00 00 00 00 00 00")

    def set_torque(self, iq_control: int) -> None:
        if not -2000 <= iq_control <= 2000:
            raise ValueError("转矩值超出范围（-2000 ~ 2000）")
        self.current_torque = int(iq_control)
        lo = iq_control & 0xFF
        hi = (iq_control >> 8) & 0xFF
        self._send_hex(f"A1 00 00 00 {lo:02X} {hi:02X} 00 00")

    def read_motor_id(self, verbose: bool = False):
        """读取电机实际的 CAN ID(1-32)"""
        arb_id = 0x300
        data = [0x79, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._send_data(arb_id, data)
        for _ in range(5):
            resp = self.can_bus.recv(timeout=0.5)
            if resp:
                if verbose:
                    print(f"[CAN] resp: {resp}, data={[hex(b) for b in resp.data]}")
                if len(resp.data) >= 7 and resp.data[0] == 0x79:
                    return resp.data[6] - 0x40
        return None

    def change_motor_id(self, old_id: int, new_id: int) -> bool:
        """修改电机的CAN ID, !!!注意：务必确保只连接一个电机
        :param old_id: 旧的电机ID
        :param new_id: 新的电机ID (1-32)
        :return: 成功返回True，失败返回False
        """
        if not (1 <= new_id <= 32):
            raise ValueError("新ID必须在1~32之间")
        data = [0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, int(new_id)]
        self._send_data(0x300, data)
        resp = self.can_bus.recv(timeout=1.0)
        return bool(resp)


# 使用示例
if __name__ == "__main__":
    can_bus = CANBusManager()
    motor = Motor(can_id=1, can_bus=can_bus)

    id = motor.read_motor_id(verbose=True)
    print(f"电机ID：{id}")

    # motor.change_motor_id(old_id=id, new_id=2)
    # print(f"修改电机ID成功")
    # print(f"新电机ID：{motor.read_motor_id()}")