"""单请求 + 窗口捕获：验证无线数采主站能否收到两个从机的响应并区分来源。

拓扑：COM5 主站 ←数采主机模块→（无线广播）←数采从机模块→ COM3 从机A / COM4 从机B。
两个从机站号相同，收到同一条请求都会响应。本脚本只发一条请求（04H 地址 3012
PBUS），随后在一个时间窗口内收集串口上出现的全部字节，按 CRC 提取有效
Modbus RTU 响应帧，再按数值奇偶分类：单数=从机A，双数=从机B。
"""

from __future__ import annotations

import time

from pymodbus.client import ModbusSerialClient

from config import (
    BAUDRATE,
    BYTESIZE,
    MASTER_PORT,
    PARITY,
    READ_INPUT_REGISTERS,
    RESPONSE_TIMEOUT,
    SLAVE_ID,
    STOPBITS,
)
from ksg1_registers import POINTS, decode_point, format_point_value

# 测试请求：04H 读地址 3012（正母线电压 PBUS），1 个寄存器。
# 期望响应帧为 7 字节：01 04 02 XX XX CRC CRC。
ADDRESS = 3012
COUNT = 1
FUNCTION_CODE = READ_INPUT_REGISTERS

# 发送请求后捕获响应的窗口时长（秒）。无线链路空口速率慢时可能需要加大。
CAPTURE_WINDOW = 4.0
# 窗口内每次串口读取的超时（秒）。
READ_CHUNK = 0.1


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC-16（poly 0xA001、init 0xFFFF），低字节在前。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, "little")


def extract_frames(buffer: bytes) -> list[tuple[int, bytes, str]]:
    """在字节流中滑窗扫描 CRC 有效的响应帧，返回 (偏移, 帧, 类型)。"""
    frames: list[tuple[int, bytes, str]] = []
    offset = 0
    while offset + 4 < len(buffer):
        if buffer[offset] != SLAVE_ID:
            offset += 1
            continue
        function_code = buffer[offset + 1]
        if function_code == FUNCTION_CODE:
            byte_count = buffer[offset + 2]
            if byte_count % 2 == 0 and 2 <= byte_count <= 250:
                frame_length = 3 + byte_count + 2
                if offset + frame_length <= len(buffer):
                    frame = buffer[offset : offset + frame_length]
                    if crc16(frame[:-2]) == frame[-2:]:
                        frames.append((offset, frame, "读响应"))
        elif function_code == FUNCTION_CODE | 0x80:
            # 异常响应：站号 + (功能码|0x80) + 异常码 + CRC，共 5 字节。
            frame = buffer[offset : offset + 5]
            if crc16(frame[:-2]) == frame[-2:]:
                frames.append((offset, frame, "异常响应"))
        offset += 1
    return frames


def classify_frame(frame: bytes) -> str:
    """按首个寄存器的原始值奇偶判断来源并格式化显示。"""
    raw = int.from_bytes(frame[3:5], "big")
    point = next(
        point
        for point in POINTS
        if point.function_code == FUNCTION_CODE
        and point.address == ADDRESS
        and point.byte_offset == 0
    )
    value = decode_point(point, {ADDRESS: raw})
    text = format_point_value(point, value)
    source = "从机A（单数）" if raw & 1 else "从机B（双数）"
    return f"{source}  原始值 {raw} = {text}"


def main() -> None:
    client = ModbusSerialClient(
        port=MASTER_PORT,
        baudrate=BAUDRATE,
        parity=PARITY,
        bytesize=BYTESIZE,
        stopbits=STOPBITS,
        timeout=RESPONSE_TIMEOUT,
    )
    print(
        f"连接数采主机模块：端口={MASTER_PORT}，站号={SLAVE_ID}，"
        f"串口={BAUDRATE}/{BYTESIZE}{PARITY}{STOPBITS}"
    )
    try:
        if not client.connect():
            print(f"无法打开 {MASTER_PORT}，请检查 COM 端口。")
            return

        socket = client.socket
        # 停顿让在途旧帧到达，再清空接收缓冲，避免把历史帧计入本次捕获。
        time.sleep(1.0)
        socket.reset_input_buffer()

        request = bytes(
            (
                SLAVE_ID,
                FUNCTION_CODE,
                ADDRESS >> 8,
                ADDRESS & 0xFF,
                COUNT >> 8,
                COUNT & 0xFF,
            )
        )
        request += crc16(request)
        print(f"发送请求：{' '.join(f'{b:02X}' for b in request)}（04H 地址 {ADDRESS} PBUS）")
        socket.write(request)

        # 捕获窗口：连续读串口字节直到窗口结束，期间每次读取只等 READ_CHUNK 秒。
        chunks: list[tuple[float, bytes]] = []
        deadline = time.monotonic() + CAPTURE_WINDOW
        old_timeout = socket.timeout
        socket.timeout = READ_CHUNK
        try:
            while time.monotonic() < deadline:
                data = socket.read(256)
                if data:
                    chunks.append((time.monotonic(), data))
        finally:
            socket.timeout = old_timeout

        buffer = b"".join(data for _, data in chunks)
        print(
            f"窗口内收到 {len(buffer)} 字节："
            f"{' '.join(f'{b:02X}' for b in buffer) or '（无）'}"
        )

        frames = extract_frames(buffer)
        print(f"其中 CRC 有效的响应帧：{len(frames)} 条")
        for offset, frame, kind in frames:
            detail = classify_frame(frame) if kind == "读响应" else f"异常码 {frame[2]:02X}"
            print(f"  偏移 {offset:3d}  {' '.join(f'{b:02X}' for b in frame)}  →  {detail}")

        # 不属于任何有效帧的字节，视为碰撞/噪声残留。
        covered = bytearray(len(buffer))
        for offset, frame, _ in frames:
            for i in range(len(frame)):
                covered[offset + i] = 1
        residue = sum(1 for flag in covered if not flag)
        print(f"无法解析为有效帧的字节：{residue}（碰撞/噪声残留）")

        if not chunks:
            print("窗口内未收到任何字节：链路未通，或两个从机上行在无线层完全碰撞。")
        elif len(frames) == 0:
            print("有字节但无有效帧：很可能两从机上行在无线层碰撞，帧全部损坏。")
        elif len(frames) == 1:
            print("只收到 1 条有效帧：另一路要么在无线层碰撞丢失，要么被捕获效应压掉。")
        else:
            print(f"收到 {len(frames)} 条有效帧：两从机响应都到达，可按奇偶区分。")
    finally:
        client.close()


if __name__ == "__main__":
    main()
