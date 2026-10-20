"""读取 KSG1-250K Modbus RTU 仿真从机的基础点位（测试用，窗口捕获两个从机的响应）。

两个从机（COM3 从机A、COM4 从机B）站号相同，都会响应同一条请求。
本脚本对每个点位发送一条请求，然后在窗口内捕获串口上出现的所有响应帧，
按数值奇偶区分来源（单数=从机A，双数=从机B）并同时显示两边的值。
"""

from __future__ import annotations

import logging
import time

from pymodbus.client import ModbusSerialClient

from config import (
    BAUDRATE,
    BYTESIZE,
    INTER_REQUEST_DELAY,
    MASTER_PORT,
    PARITY,
    READ_ATTEMPTS,
    RESPONSE_TIMEOUT,
    SLAVE_ID,
    STOPBITS,
)
from ksg1_registers import (
    POINTS,
    RegisterPoint,
    decode_point,
    format_point_value,
)

# 测试用：只读取最基础的几个点位（不超过 10 个）。
TEST_POINT_NAMES = (
    "逆变器型号",        # 03H 设备身份
    "正母线电压 PBUS",   # 04H 母线遥测
    "电网 RS 线电压",    # 04H 电网遥测
    "交流有功功率",      # 04H
    "功率因数",          # 04H
    "运行状态",          # 04H
    "当日发电量",        # 04H
    "累计发电量",        # 04H
    "额定功率",          # 04H
)

# 发送请求后捕获响应的窗口时长（秒）；无线链路空口速率慢时可能需要加大。
CAPTURE_WINDOW = 3.0
# 窗口内每次串口读取的超时（秒）。
READ_CHUNK = 0.1
# 集齐两个从机的响应后，再观察这段静默期以接住可能迟到的帧（秒）。
QUIET_PERIOD = 0.3
# 总线上同站号从机的数量；集齐后提前结束窗口。
EXPECTED_RESPONSES = 2


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC-16（poly 0xA001、init 0xFFFF），低字节在前。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, "little")


def build_request(function_code: int, address: int, count: int) -> bytes:
    """构造一条 Modbus RTU 读请求帧。"""
    request = bytes(
        (
            SLAVE_ID,
            function_code,
            address >> 8,
            address & 0xFF,
            count >> 8,
            count & 0xFF,
        )
    )
    return request + crc16(request)


def flush_input_buffer(client: ModbusSerialClient) -> None:
    """清空串口接收缓冲，丢弃无线链路延迟或重复送达的旧帧。"""
    socket = getattr(client, "socket", None)
    if socket is not None:
        socket.reset_input_buffer()


def scan_frames(
    buffer: bytes,
    function_code: int,
    byte_count: int,
) -> tuple[list[bytes], list[bytes]]:
    """在字节流中滑窗扫描 CRC 有效的响应帧，返回 (读响应帧, 异常响应帧)。"""
    reads: list[bytes] = []
    exceptions: list[bytes] = []
    offset = 0
    while offset + 4 < len(buffer):
        if buffer[offset] != SLAVE_ID:
            offset += 1
            continue
        frame_function_code = buffer[offset + 1]
        if frame_function_code == function_code:
            if buffer[offset + 2] == byte_count:
                frame_length = 3 + byte_count + 2
                if offset + frame_length <= len(buffer):
                    frame = buffer[offset : offset + frame_length]
                    if crc16(frame[:-2]) == frame[-2:]:
                        reads.append(frame)
        elif frame_function_code == function_code | 0x80:
            # 异常响应：站号 + (功能码|0x80) + 异常码 + CRC，共 5 字节。
            frame = buffer[offset : offset + 5]
            if crc16(frame[:-2]) == frame[-2:]:
                exceptions.append(frame)
        offset += 1
    return reads, exceptions


def capture_responses(
    client: ModbusSerialClient,
    function_code: int,
    address: int,
    count: int,
) -> tuple[list[bytes], list[bytes]]:
    """发送一条请求，在窗口内收集全部有效响应；集齐后静默一小段提前结束。"""
    socket = client.socket
    socket.write(build_request(function_code, address, count))

    buffer = bytearray()
    reads: list[bytes] = []
    exceptions: list[bytes] = []
    quiet_since: float | None = None
    deadline = time.monotonic() + CAPTURE_WINDOW
    old_timeout = socket.timeout
    socket.timeout = READ_CHUNK
    try:
        while time.monotonic() < deadline:
            data = socket.read(256)
            now = time.monotonic()
            if data:
                buffer.extend(data)
                quiet_since = None
                reads, exceptions = scan_frames(bytes(buffer), function_code, 2 * count)
            elif quiet_since is None:
                quiet_since = now
            if (
                len(reads) + len(exceptions) >= EXPECTED_RESPONSES
                and quiet_since is not None
                and now - quiet_since >= QUIET_PERIOD
            ):
                break
    finally:
        socket.timeout = old_timeout
    return reads, exceptions


def frame_registers(point: RegisterPoint, frame: bytes) -> dict[int, int]:
    """把响应帧的数据字节按文档地址还原成寄存器字典。"""
    data = frame[3:-2]
    return {
        point.address + index: int.from_bytes(data[index * 2 : index * 2 + 2], "big")
        for index in range(len(data) // 2)
    }


def point_header(point: RegisterPoint) -> str:
    """格式化为“功能码 地址 名称”的固定宽度表头。"""
    end_address = point.address + point.register_count - 1
    address_text = (
        str(point.address)
        if end_address == point.address
        else f"{point.address}–{end_address}"
    )
    return f"{point.function_code:02X}H 地址 {address_text:<9} {point.name:<20}"


def select_test_points() -> list[RegisterPoint]:
    """从点表中挑出测试需要的点位，按功能码与地址排序。"""
    wanted = set(TEST_POINT_NAMES)
    return sorted(
        (point for point in POINTS if point.name in wanted),
        key=lambda point: (point.function_code, point.address, point.byte_offset),
    )


def display_point(point: RegisterPoint, frames: list[bytes]) -> None:
    """解析所有响应帧，按数值奇偶归入从机A/B后同时显示。"""
    header = point_header(point)

    if point.data_type in {"ascii", "mac"}:
        # 身份类字段两从机一致：去重后显示一次。
        values: list[str] = []
        for frame in frames:
            value = decode_point(point, frame_registers(point, frame))
            text = format_point_value(point, value)
            if text not in values:
                values.append(text)
        print(f"  {header} {' / '.join(values)}（两从机一致）")
        return

    by_source: dict[str, int | float] = {}
    for frame in frames:
        value = decode_point(point, frame_registers(point, frame))
        raw_int = round(value / point.scale)
        by_source["A" if raw_int & 1 else "B"] = value
    a_text = format_point_value(point, by_source["A"]) if "A" in by_source else "—"
    b_text = format_point_value(point, by_source["B"]) if "B" in by_source else "—"
    print(f"  {header} 从机A: {a_text:<14} 从机B: {b_text}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    client = ModbusSerialClient(
        port=MASTER_PORT,
        # PyModbus 3.3 的串口客户端默认使用 RTU framer。
        baudrate=BAUDRATE,
        parity=PARITY,
        bytesize=BYTESIZE,
        stopbits=STOPBITS,
        timeout=RESPONSE_TIMEOUT,
    )

    points = select_test_points()
    print(
        f"连接 KSG1-250K Modbus RTU 仿真从机：端口={MASTER_PORT}，"
        f"站号={SLAVE_ID}，串口={BAUDRATE}/{BYTESIZE}{PARITY}{STOPBITS}"
    )
    print(f"测试读取 {len(points)} 个基础点位（窗口捕获，同时显示从机A/B）")

    try:
        if not client.connect():
            logging.error(
                "无法打开 %s。请检查 COM 端口、驱动和端口占用。",
                MASTER_PORT,
            )
            return

        for point in points:
            # 无线透传可能把上一笔事务的旧响应延迟或重复送达。
            # 先停顿让在途旧帧到达，再清空接收缓冲，避免旧帧混入本次窗口。
            time.sleep(INTER_REQUEST_DELAY)
            flush_input_buffer(client)

            frames: list[bytes] = []
            exceptions: list[bytes] = []
            for _ in range(READ_ATTEMPTS):
                frames, exceptions = capture_responses(
                    client, point.function_code, point.address, point.register_count
                )
                if frames or exceptions:
                    break

            header = point_header(point)
            if exceptions and not frames:
                exception_text = " ".join(
                    f"{byte:02X}" for frame in exceptions for byte in frame
                )
                logging.warning("%s 收到异常响应：%s", point.name, exception_text)
                print(f"  {header} 从机A: —  从机B: —（异常响应 {exception_text}）")
                continue
            if not frames:
                logging.warning("%s 无有效响应（重试 %d 次）", point.name, READ_ATTEMPTS)
                print(f"  {header} 从机A: —  从机B: —（无响应）")
                continue
            display_point(point, frames)
    except (OSError, ValueError, RuntimeError) as error:
        logging.error(
            "Modbus 读取失败：%s。请检查从机是否运行、站号、RS-485 接线和串口参数。",
            error,
        )
    except Exception:
        logging.exception("读取 KSG1-250K 仿真点位时发生意外错误。")
    finally:
        client.close()


if __name__ == "__main__":
    main()
