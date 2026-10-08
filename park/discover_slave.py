"""发现 Solis 逆变器的从站地址（Slave ID）。

逐个扫描站号 1..8，读取 FC04 产品型号。每次只发送一条请求，并在配置的
超时时间内等待匹配的响应；收到其他站号、噪声或坏 CRC 时继续等待。
发送前读取并记录残留数据，等待串口静默，方便排查无线链路的迟到回包。

用法：
    1. 按 config.py 配好串口参数（波特率等）与 MASTER_PORT；
    2. 运行 `python discover_slave.py`；
    3. 把打印出的站号填回 config.py 的 SLAVE_ID。

依赖：pyserial（已在 requirements.txt 中）。
"""

from __future__ import annotations

import logging
import time

import serial

from config import (
    BAUDRATE,
    BYTESIZE,
    MASTER_PORT,
    PARITY,
    QUERY_INTERVAL_S,
    REG_PRODUCT_MODEL,
    RESPONSE_TIMEOUT,
    STOPBITS,
)

PARITY_MAP = {
    "N": serial.PARITY_NONE,
    "E": serial.PARITY_EVEN,
    "O": serial.PARITY_ODD,
}


def crc16(data: bytes) -> int:
    """Modbus CRC-16（多项式 0xA001，初值 0xFFFF）。返回低字节在前的整型。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_input_registers(slave: int, address: int, count: int) -> bytes:
    """构造 FC04 读输入寄存器请求帧（含 CRC，低字节在前）。"""
    frame = bytes(
        [
            slave,
            0x04,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ]
    )
    crc = crc16(frame)
    return frame + bytes([crc & 0xFF, crc >> 8])


def open_port(timeout: float) -> serial.Serial:
    return serial.Serial(
        port=MASTER_PORT,
        baudrate=BAUDRATE,
        bytesize=BYTESIZE,
        parity=PARITY_MAP[PARITY],
        stopbits=STOPBITS,
        timeout=timeout,
        write_timeout=timeout,
    )


def wait_for_quiet(port: serial.Serial, timeout: float) -> None:
    """读取残留数据，等本地 RX 静默满查询间隔；持续有数据时停止扫描。"""
    now = time.monotonic()
    quiet_deadline = now + QUERY_INTERVAL_S
    limit = now + timeout + QUERY_INTERVAL_S
    while True:
        now = time.monotonic()
        if now >= quiet_deadline:
            return
        if now >= limit:
            raise RuntimeError("串口持续收到数据，未获得发送前的静默间隔，停止扫描")
        port.timeout = min(quiet_deadline, limit) - now
        data = port.read(max(1, port.in_waiting))
        if data:
            logging.warning(
                "TX 前 RX（残留/迟到数据或噪声，%d 字节）：%s；重新等待静默",
                len(data), data.hex(" ").upper(),
            )
            quiet_deadline = time.monotonic() + QUERY_INTERVAL_S


def extract_response(buffer: bytearray, slave: int) -> bytes | None:
    """从字节流中提取匹配的 7 字节正常响应或 5 字节异常响应。"""
    while len(buffer) >= 2:
        function = buffer[1]
        if function == 0x04:
            if len(buffer) < 3:
                return None
            if buffer[2] != 2:
                logging.warning("忽略 FC04 字节数 %d，期望 2", buffer[2])
                del buffer[0]
                continue
            frame_size = 7
        elif function == 0x84:
            frame_size = 5
        else:
            del buffer[0]
            continue
        if len(buffer) < frame_size:
            return None
        frame = bytes(buffer[:frame_size])
        if crc16(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
            logging.warning("忽略 CRC 错误的候选帧：%s", frame.hex(" ").upper())
            del buffer[0]
            continue
        del buffer[:frame_size]
        if frame[0] != slave:
            logging.warning(
                "忽略站号 %d 的响应，当前等待站号 %d：%s",
                frame[0], slave, frame.hex(" ").upper(),
            )
            continue
        return frame
    return None


def probe(port: serial.Serial, slave: int, timeout: float) -> bytes | None:
    """顺序发送一次探测，直到截止时间只接受该站号的 FC04 响应。"""
    if timeout <= 0 or QUERY_INTERVAL_S <= 0:
        raise ValueError("响应超时和查询间隔必须大于 0")
    # 本地静默只用于留出收发间隔，不能证明无线链路已没有在途数据。
    wait_for_quiet(port, timeout)
    request = build_read_input_registers(slave, REG_PRODUCT_MODEL, 1)
    logging.info("TX 站号 %d（%d 字节）：%s", slave, len(request), request.hex(" ").upper())
    port.write_timeout = timeout
    if port.write(request) != len(request):
        raise serial.SerialTimeoutException("探测请求未完整写入串口")
    # flush 只等待本地发送队列排空，不代表无线模块或从机已收到请求。
    port.flush()
    started = time.monotonic()
    deadline = started + timeout
    buffer = bytearray()
    received_count = 0
    while True:
        response = extract_response(buffer, slave)
        if response is not None:
            logging.info(
                "站号 %d 收到匹配%s响应，用时 %.3f 秒：%s",
                slave, "异常" if response[1] == 0x84 else "正常",
                time.monotonic() - started, response.hex(" ").upper(),
            )
            if buffer:
                logging.warning("同一接收块的剩余数据：%s", buffer.hex(" ").upper())
            return response
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            logging.warning(
                "站号 %d 等待 %.3f 秒后超时：收到 %d 字节，无匹配响应；残留字节：%s",
                slave, time.monotonic() - started, received_count,
                buffer.hex(" ").upper() or "无",
            )
            return None
        port.timeout = remaining
        data = port.read(max(1, port.in_waiting))
        if data:
            received_count += len(data)
            logging.info(
                "RX 等待站号 %d，已过 %.3f 秒（%d 字节）：%s",
                slave, time.monotonic() - started, len(data), data.hex(" ").upper(),
            )
            buffer.extend(data)


def scan(port: serial.Serial) -> list[int]:
    """逐个扫描 1..8，返回成功读出产品型号的站号。"""
    found: list[int] = []
    print("扫描站号 1..8")
    for slave in range(1, 9):
        response = probe(port, slave, RESPONSE_TIMEOUT)
        if response is None:
            continue
        if response[1] == 0x84:
            print(f"  站号 {slave} 返回 Modbus 异常码 0x{response[2]:02X}，产品型号未读出")
        else:
            found.append(slave)
            print(f"  站号 {slave} 有应答（产品型号 0x{(response[3] << 8) | response[4]:04X}）")
    return found


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )
    print(
        f"扫描从站：端口={MASTER_PORT}，串口={BAUDRATE}/{BYTESIZE}{PARITY}{STOPBITS}，"
        f"响应超时={RESPONSE_TIMEOUT} 秒，发送前静默间隔={QUERY_INTERVAL_S} 秒"
    )
    try:
        with open_port(timeout=RESPONSE_TIMEOUT) as port:
            found = scan(port)
            if found:
                print(f"扫描完成，成功读取产品型号的站号：{found}")
            else:
                print("未读出任何产品型号。请结合收发日志检查站号、接线、串口参数与无线链路。")
    except (serial.SerialException, OSError, ValueError, RuntimeError) as error:
        logging.error("扫描失败：%s", error)
    except KeyboardInterrupt:
        print("\n扫描已停止。")


if __name__ == "__main__":
    main()
