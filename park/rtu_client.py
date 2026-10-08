"""PyModbus 3.3 的 FC04 串口接收扩展：完整帧匹配与固定响应截止时间。"""

from __future__ import annotations

import logging
import time

import serial
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ConnectionException
from pymodbus.utilities import ModbusTransactionState

LOG = logging.getLogger(__name__)


def crc16(data: bytes) -> int:
    """计算 Modbus CRC，返回线上的低字节在前所对应的数值。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def valid_crc(frame: bytes) -> bool:
    return len(frame) >= 4 and crc16(frame[:-2]) == int.from_bytes(frame[-2:], "little")


def extract_response(buffer: bytearray, slave: int, count: int) -> bytes | None:
    """忽略其他站响应、FC03/04 请求或回显，只返回匹配的 FC04 完整帧。"""
    expected_bytes = count * 2
    while len(buffer) >= 2:
        function = buffer[1]
        if function == 0x84:
            if len(buffer) < 5:
                return None
            candidate = bytes(buffer[:5])
            if valid_crc(candidate):
                del buffer[:5]
                if candidate[0] == slave:
                    return candidate
                LOG.warning("忽略站号 %d 的异常响应，等待站号 %d：%s", candidate[0], slave, candidate.hex(" ").upper())
                continue
        elif function in (0x03, 0x04):
            if len(buffer) < 3:
                return None
            byte_count = buffer[2]
            frame_size = byte_count + 5
            # 先识别完整正常响应，再识别 8 字节请求，避免把响应当作请求。
            if function == 0x04 and 2 <= byte_count <= 250 and byte_count % 2 == 0:
                if len(buffer) >= frame_size:
                    candidate = bytes(buffer[:frame_size])
                    if valid_crc(candidate):
                        del buffer[:frame_size]
                        if candidate[0] == slave and byte_count == expected_bytes:
                            return candidate
                        LOG.warning(
                            "忽略不匹配响应：站号=%d、数据字节=%d；期望站号=%d、数据字节=%d：%s",
                            candidate[0], byte_count, slave, expected_bytes, candidate.hex(" ").upper(),
                        )
                        continue
            if len(buffer) >= 8 and valid_crc(bytes(buffer[:8])):
                request = bytes(buffer[:8])
                del buffer[:8]
                LOG.warning(
                    "忽略请求/回显帧：站号=%d、功能码=%02X、地址=%d、数量=%d：%s",
                    request[0], request[1], int.from_bytes(request[2:4], "big"),
                    int.from_bytes(request[4:6], "big"), request.hex(" ").upper(),
                )
                continue
            if function == 0x04 and byte_count == expected_bytes and len(buffer) < frame_size:
                return None
            if len(buffer) < 8:
                return None
        # 坏 CRC、错误功能码或噪声：逐字节重新寻找帧头，截止时间不变。
        LOG.debug("跳过无法匹配的字节：%02X", buffer[0])
        del buffer[0]
    return None


class LoggedModbusSerialClient(ModbusSerialClient):
    """仅用于本项目 FC04 读取；为 PyModbus 提供完整、经过校验的响应。"""

    def __init__(self, *args, query_interval: float, **kwargs):
        # 重试在主机业务层执行，避免同一读取出现两套重试计数。
        kwargs.update(retry_on_empty=False, retry_on_invalid=False, reset_socket=False)
        super().__init__(*args, **kwargs)
        # PyModbus 3.3 的 RTU 帧头白名单未包含异常功能码；客户端解码器
        # 已支持 ExceptionResponse，需允许 FC84 通过帧头筛选。
        self.framer.function_codes.add(0x84)
        if query_interval <= 0 or self.params.timeout <= 0:
            raise ValueError("查询间隔和响应超时必须大于 0")
        self.query_interval = query_interval
        self._reply = b""
        self._collect_pending = False

    def _wait_for_quiet(self) -> None:
        """读取并记录发送前残留数据，最后一个 RX 后留出完整查询间隔。"""
        now = time.monotonic()
        quiet_deadline = now + self.query_interval
        limit = now + self.params.timeout + self.query_interval
        while True:
            now = time.monotonic()
            if now >= quiet_deadline:
                return
            if now >= limit:
                raise RuntimeError("串口持续有接收数据，无法获得发送前静默间隔，停止本轮读取")
            self.socket.timeout = min(quiet_deadline, limit) - now
            data = self.socket.read(max(1, self.socket.in_waiting))
            if data:
                LOG.warning("TX 前 RX（残留/迟到数据或现场流量，%d 字节）：%s", len(data), data.hex(" ").upper())
                quiet_deadline = time.monotonic() + self.query_interval

    def send(self, request: bytes) -> int:
        if not self.socket:
            raise ConnectionException("串口尚未连接")
        if len(request) != 8 or request[1] != 0x04:
            raise ValueError("此客户端仅支持 FC04 读输入寄存器")
        self._slave = request[0]
        self._address = int.from_bytes(request[2:4], "big")
        self._count = int.from_bytes(request[4:6], "big")
        if not 1 <= self._count <= 125:
            raise ValueError("读取数量必须为 1..125")
        # 本地静默并不代表 4G 链路中已没有在途回包。
        self._wait_for_quiet()
        self._reply = b""
        self._collect_pending = False
        LOG.info(
            "TX 站号=%d、FC04、地址=%d、数量=%d（%d 字节）：%s",
            self._slave, self._address, self._count, len(request), request.hex(" ").upper(),
        )
        self.state = ModbusTransactionState.SENDING
        self.socket.write_timeout = self.params.timeout
        size = self.socket.write(request)
        if size != len(request):
            raise serial.SerialTimeoutException("请求未完整写入串口")
        # 只确认本地发送队列排空，不能证明无线终端或从机已收到请求。
        self.socket.flush()
        self._started = time.monotonic()
        self._deadline = self._started + self.params.timeout
        self._collect_pending = True
        return size

    def _receive_response(self) -> bytes:
        buffer = bytearray()
        received = 0
        while True:
            response = extract_response(buffer, self._slave, self._count)
            if response is not None:
                LOG.info(
                    "匹配%s响应：站号=%d、当前请求地址=%d，用时 %.3f 秒：%s",
                    "异常" if response[1] == 0x84 else "正常", self._slave, self._address,
                    time.monotonic() - self._started, response.hex(" ").upper(),
                )
                if buffer:
                    LOG.warning("同一接收块的剩余数据：%s", buffer.hex(" ").upper())
                return response
            remaining = self._deadline - time.monotonic()
            if remaining <= 0:
                LOG.warning(
                    "响应超时：站号=%d、地址=%d、数量=%d，等待 %.3f 秒，收到 %d 字节但没有匹配帧；残留=%s",
                    self._slave, self._address, self._count, time.monotonic() - self._started,
                    received, buffer.hex(" ").upper() or "无",
                )
                return b""
            self.socket.timeout = remaining
            data = self.socket.read(max(1, self.socket.in_waiting))
            if data:
                received += len(data)
                LOG.info(
                    "RX 等待站号=%d、地址=%d，已过 %.3f 秒（%d 字节）：%s",
                    self._slave, self._address, time.monotonic() - self._started,
                    len(data), data.hex(" ").upper(),
                )
                buffer.extend(data)

    def recv(self, size: int | None) -> bytes:
        # PyModbus 3.3 先要 4 字节头，再要剩余字节；先组装完整有效帧再分给它，
        # 防止现场请求或分片头触发一次事务的提前解码失败。
        if self._collect_pending:
            self._reply = self._receive_response()
            self._collect_pending = False
        size = len(self._reply) if size is None else size
        result, self._reply = self._reply[:size], self._reply[size:]
        return result
