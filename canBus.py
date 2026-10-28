# -*- coding: utf-8 -*-
"""
can_bus.py
统一的 CAN 总线管理类：
- 只初始化一次 python-can Bus
- 提供线程安全的 send()/recv()
- 可按需 bring up/down 物理 canX 接口（socketcan）
"""
import os
import threading
import can
import time
from typing import Optional, Sequence

class CANBusManager:
    def __init__(
        self,
        channel: str = 'can0',
        bustype: str = 'socketcan',
        bitrate: int = 1000000,
        txqueuelen: int = 1024,
        bring_up: bool = True,
        manage_iface: bool = True,
    ) -> None:
        """
        :param manage_iface: True=允许在 close() 时自动 ifconfig down
        """
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.txqueuelen = txqueuelen
        self.manage_iface = manage_iface

        if bring_up and bustype == 'socketcan':
            os.system(f'sudo ip link set {channel} up type can bitrate {bitrate}')
            os.system(f'sudo ifconfig {channel} txqueuelen {txqueuelen}')

        self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        self._send_lock = threading.Lock()

        # 可选：设置过滤器，减少用户态负担（默认不过滤）
        # self.bus.set_filters([{'can_id': 0x000, 'can_mask': 0x000, 'extended': False}])

        print(f"[CAN] opened: channel={self.channel}, bustype={self.bustype}, bitrate={self.bitrate}")

    # 线程安全发送
    def send(self, msg: can.Message, retries: int = 3, backoff_s: float = 0.002) -> None:
        for i in range(retries):
            try:
                with self._send_lock:
                    self.bus.send(msg)
                return
            except can.CanOperationError as e:
                # TX 队列满时短暂退避
                time.sleep(backoff_s)
        # 最终仍失败，抛出以便你看到（也可换成日志+丢包）
        raise

    # 直接发送数据帧（便捷）
    def send_data(self, arbitration_id: int, data: Sequence[int], is_extended_id: bool = False) -> None:
        msg = can.Message(arbitration_id=arbitration_id, data=bytes(data), is_extended_id=is_extended_id)
        self.send(msg)

    # 接收（非阻塞/阻塞取决于 timeout）
    def recv(self, timeout: Optional[float] = 0.0) -> Optional[can.Message]:
        return self.bus.recv(timeout=timeout)

    def shutdown(self) -> None:
        try:
            if self.bus:
                self.bus.shutdown()
                print("[CAN] bus.shutdown()")
        finally:
            if self.manage_iface and self.bustype == 'socketcan':
                os.system(f"sudo ifconfig {self.channel} down")
                print(f"[CAN] iface {self.channel} down")
