# -*- coding: utf-8 -*-
"""
new_Rot_Encoder_RS485.py
旋转编码器管理类：
port='/dev/ttySC0' 对应树莓派 RS485_0 接口
port='/dev/ttySC1' 对应树莓派 RS485_1 接口
"""
import struct
import time
from collections import deque

import serial

# ---- CRC16：优先用 crcmod；若缺失则用后备实现 ----
try:
    import crcmod
    _crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
except Exception:
    def _crc16(data: bytes) -> int:
        # CRC-16/IBM (反射) 的等价实现：poly=0xA001, init=0xFFFF, xorOut=0x0000
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF


class RotateEncoder:
    """
    读取角度/速度方向，并用滑动窗口估计角速度，抑制静止抖动。
    """
    REQ_HEADER = 0x2E  # 请求帧头
    RSP_HEADER = 0x2C  # 响应帧头（按文档）
    CMD_READ   = 0x2F  # 读取角度/速度/方向的命令

    def __init__(self,
                 port='/dev/ttySC0',
                 baudrate=115200,
                 timeout=0.005,
                 vel_window_ms=80,          # 速度估计窗口（ms）
                 zero_deadband_dps=0.4):    # 零速死区（°/s）
        # 串口
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )

        # 角速度估计缓冲
        self._buf = deque()                 # 存 (t, angle_deg_unwrapped)
        self._win = vel_window_ms / 1000.0
        self._zero_db = float(zero_deadband_dps)

        # 单圈角度系数（15-bit 单圈 -> 0..32767）
        self._deg_per_lsb = 360.0 / 32768.0

    # ---------- 工具 ----------
    @staticmethod
    def _unwrap_to(prev_deg_unwrapped: float, cur_deg: float) -> float:
        """
        将当前角度(0..360)展开到与 prev 相邻的分支，避免 359->0 回绕假差
        返回：新的“展开角度”（可超出 0..360）
        """
        if prev_deg_unwrapped is None:
            return cur_deg
        d = cur_deg - (prev_deg_unwrapped % 360.0)
        while d >= 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        return prev_deg_unwrapped + d

    def _read_exact(self, n: int) -> bytes:
        """严格读取 n 字节（受 timeout 影响，可能返回不足 n 字节）"""
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                break
            buf += chunk
        return bytes(buf)

    # ---------- 协议 ----------
    def build_packet(self, device_addr: int, cmd: int, data=b'', seq=0x01) -> bytes:
        pkt = bytearray()
        pkt.append(self.REQ_HEADER)              # 帧头 0x2E
        pkt.append(seq & 0xFF)                   # 包序号
        pkt.append(device_addr & 0xFF)           # 设备地址
        pkt.append(cmd & 0xFF)                   # 命令码
        pkt.append(len(data) & 0xFF)             # 数据长度（1B）
        pkt += data                               # 数据
        crc = _crc16(pkt)
        pkt += struct.pack('<H', crc)            # CRC16 小端
        return bytes(pkt)

    def read_response(self):
        """
        响应帧格式（依据文档常见布局）：
        [0]=0x2C | [1]=SEQ | [2]=ADDR | [3]=CMD | [4]=LEN | [5:5+LEN]=DATA | [5+LEN:7+LEN]=CRC(LE)
        """
        # 找帧头 0x2C
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b[0] == self.RSP_HEADER:
                break

        # 读 SEQ, ADDR, CMD, LEN
        head_rest = self._read_exact(4)
        if len(head_rest) != 4:
            return None
        seq, addr, cmd, data_len = head_rest

        # 合理性检查（防止异常 LEN）
        if data_len > 64:  # 保护上限，可按文档调整
            return None

        # 读 DATA + CRC(2)
        tail = self._read_exact(data_len + 2)
        if len(tail) != data_len + 2:
            return None

        frame = bytes([self.RSP_HEADER, seq, addr, cmd, data_len]) + tail
        # CRC 校验
        if _crc16(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
            return None
        return frame

    # ---------- 高层读取 ----------
    def read_encoder_data(self,
                          device_addr=0x00,
                          want_angle=True,
                          want_speed=True,
                          want_direction=True):
        """
        返回 dict: {'angle':deg, 'speed_rpm':rpm, 'speed_dps':deg/s, 'direction':str}
        其中 speed_rpm/dir 依赖设备是否回报；speed_dps 为由 rpm 换算的近似值
        """
        pkt = self.build_packet(device_addr, self.CMD_READ)
        self.ser.reset_input_buffer()
        self.ser.write(pkt)

        resp = self.read_response()
        if not resp:
            return None

        # 基础检查
        if resp[3] != self.CMD_READ:
            return None

        data_off = 5
        data_len = resp[4]
        data = resp[data_off:data_off + data_len]

        out = {}
        try:
            # ---- 角度（大端 16-bit 单圈）----
            if want_angle and data_len >= 2:
                raw = struct.unpack('>H', data[0:2])[0]
                out['angle'] = raw * self._deg_per_lsb

            # ---- 速度（协议文档常见：0.1 RPM 分辨率；你原代码：×0.1）----
            # 若你的设备字段偏移不同，请按文档调整以下偏移
            if want_speed and data_len >= 12:
                spd_raw = struct.unpack('>H', data[10:12])[0]  # 例：偏移 10..11
                speed_rpm = spd_raw * 0.1
                out['speed_rpm'] = speed_rpm
                out['speed_dps'] = speed_rpm * 6.0  # 1 RPM = 6 °/s

            # ---- 方向 ----
            if want_direction and data_len >= 13:
                direction = data[12]
                out['direction'] = ['反方向', '正方向', '静止'][direction] if direction < 3 else '未知'
        except Exception:
            return None

        return out

    # ---------- 角速度估计（滑动窗口 + 死区） ----------
    def estimate_velocity(self, angle_deg: float, t_now: float) -> float:
        """
        返回°/s；用窗口（默认 80ms）首末差分估计，并施加零速死区。
        """
        if angle_deg is None:
            return 0.0

        # 展开角度
        last_unwrapped = self._buf[-1][1] if self._buf else None
        angle_unwrapped = self._unwrap_to(last_unwrapped, angle_deg)

        # 入缓冲
        self._buf.append((t_now, angle_unwrapped))

        # 窗口裁剪
        t_cut = t_now - self._win
        while len(self._buf) > 1 and self._buf[0][0] < t_cut:
            self._buf.popleft()

        # 首末差分
        t0, a0 = self._buf[0]
        tn, an = self._buf[-1]
        dt = tn - t0
        if dt <= 0:
            return 0.0
        omega = (an - a0) / dt  # °/s

        # 零速死区
        if abs(omega) < self._zero_db:
            omega = 0.0
        return omega

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

# ---- 测试代码 ----
if __name__ == '__main__':
    enc = RotateEncoder(port='/dev/ttySC0', baudrate=115200,
                        timeout=0.005, vel_window_ms=80, zero_deadband_dps=0.4)

    while True:
        data = enc.read_encoder_data(
            device_addr=0x00, 
            want_angle=True,
            want_speed=False,
            want_direction=False
        )

        start_time = time.time()

        speed = enc.estimate_velocity(data.get('angle'), start_time)
        print(f"角度: {data.get('angle'):.3f}°")
        print(f"速度: {speed:.3f}°/s")

