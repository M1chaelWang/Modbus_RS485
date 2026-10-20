"""读取 KSG1-250K Modbus RTU 只读仿真从机的基础点位（测试用，仅少量点位）。"""

from __future__ import annotations

import logging
import time
from collections.abc import Iterator

from pymodbus.client import ModbusSerialClient

from config import (
    BAUDRATE,
    BYTESIZE,
    INTER_REQUEST_DELAY,
    MASTER_PORT,
    MAX_REGISTERS_PER_REQUEST,
    PARITY,
    READ_ATTEMPTS,
    READ_HOLDING_REGISTERS,
    READ_INPUT_REGISTERS,
    RESPONSE_TIMEOUT,
    SLAVE_ID,
    STOPBITS,
)
from ksg1_registers import (
    POINTS,
    ReadBlock,
    RegisterPoint,
    decode_point,
    format_point_value,
)

# 测试用：只读取最基础的几个点位（不超过 10 个），用于快速验证通信链路。
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


def split_read_block(block: ReadBlock) -> Iterator[tuple[int, int]]:
    """把一个读取块切成大小尽量均匀、且不超过 125 寄存器的请求。

    不用"125 + 余数"的方式，避免末尾出现 1 个寄存器这样的极小帧——
    无线透传模块对末尾的小帧容易丢包或积压。
    """
    chunk_count = (block.count + MAX_REGISTERS_PER_REQUEST - 1) // MAX_REGISTERS_PER_REQUEST
    base, extra = divmod(block.count, chunk_count)
    address = block.address
    remaining = block.count
    while remaining:
        size = base + (1 if extra > 0 else 0)
        if extra > 0:
            extra -= 1
        yield address, size
        address += size
        remaining -= size


def flush_input_buffer(client: ModbusSerialClient) -> None:
    """清空串口接收缓冲，丢弃无线链路延迟或重复送达的旧帧。"""
    socket = getattr(client, "socket", None)
    if socket is not None:
        socket.reset_input_buffer()


def read_block(
    client: ModbusSerialClient,
    block: ReadBlock,
) -> dict[int, int]:
    """读取一个文档地址块并返回以文档/PDU 地址索引的寄存器值。"""
    values: dict[int, int] = {}
    for address, count in split_read_block(block):
        response = _read_registers(client, block, address, count)
        values.update(zip(range(address, address + count), response.registers))
    return values


def _read_registers(
    client: ModbusSerialClient,
    block: ReadBlock,
    address: int,
    count: int,
):
    """发送一条读请求，失败时自动重试；返回一条有效响应。"""
    request_name = (
        "03H Holding Registers"
        if block.function_code == READ_HOLDING_REGISTERS
        else "04H Input Registers"
    )
    last_message = ""
    for _ in range(READ_ATTEMPTS):
        # 无线透传可能把上一笔事务的旧响应延迟或重复送达。
        # 先停顿让在途旧帧到达，再清空接收缓冲，避免把旧帧当成本次响应。
        time.sleep(INTER_REQUEST_DELAY)
        flush_input_buffer(client)

        if block.function_code == READ_HOLDING_REGISTERS:
            response = client.read_holding_registers(
                address=address,
                count=count,
                slave=SLAVE_ID,
            )
        elif block.function_code == READ_INPUT_REGISTERS:
            response = client.read_input_registers(
                address=address,
                count=count,
                slave=SLAVE_ID,
            )
        else:  # 防御性检查：共享点表只能声明 03H 或 04H。
            raise ValueError(f"不支持的读取功能码：{block.function_code:02X}H")

        if response.isError():
            last_message = f"返回异常：{response}"
            continue
        if response.function_code != block.function_code:
            last_message = (
                f"收到功能码 {response.function_code:02X}H 的响应，期望 "
                f"{block.function_code:02X}H；总线上可能有另一个主机同时在轮询"
            )
            continue
        if len(response.registers) != count:
            last_message = f"收到 {len(response.registers)} 个寄存器，响应长度不完整"
            continue
        return response

    raise RuntimeError(
        f"{request_name} 地址 {address}、数量 {count} 重试 {READ_ATTEMPTS} 次仍失败："
        f"{last_message or '无有效响应'}"
    )


def select_test_points() -> list[RegisterPoint]:
    """从点表中挑出测试需要的点位，按功能码与地址排序。"""
    wanted = set(TEST_POINT_NAMES)
    return sorted(
        (point for point in POINTS if point.name in wanted),
        key=lambda point: (point.function_code, point.address, point.byte_offset),
    )


def display_point(point: RegisterPoint, registers: dict[int, int]) -> None:
    """解析并显示单个业务点。"""
    value = decode_point(point, registers)
    end_address = point.address + point.register_count - 1
    address_text = (
        str(point.address)
        if end_address == point.address
        else f"{point.address}–{end_address}"
    )
    print(
        f"  {point.function_code:02X}H 地址 {address_text:<9} "
        f"{point.name:<20} {format_point_value(point, value)}"
    )


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
    print(f"测试读取 {len(points)} 个基础点位")

    try:
        if not client.connect():
            logging.error(
                "无法打开 %s。请检查 COM 端口、驱动和端口占用。",
                MASTER_PORT,
            )
            return

        for point in points:
            block = ReadBlock(
                point.function_code,
                point.address,
                point.register_count,
                point.name,
            )
            registers = read_block(client, block)
            display_point(point, registers)
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
