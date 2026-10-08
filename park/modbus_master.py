"""运行 Modbus RTU 主机，读取 Solis 并网逆变器的运行信息（FC04）。"""

from __future__ import annotations

import logging
import sys

from pymodbus.exceptions import ModbusIOException
from pymodbus.pdu import ExceptionResponse

from rtu_client import LoggedModbusSerialClient

from config import (
    BAUDRATE,
    BYTESIZE,
    MASTER_PORT,
    PARITY,
    QUERY_INTERVAL_S,
    READ_ATTEMPTS,
    REG_ACTIVE_POWER,
    REG_GRID_FREQ,
    REG_PRODUCT_MODEL,
    REG_STATUS,
    REG_TOTAL_ENERGY,
    RESPONSE_TIMEOUT,
    SLAVE_ID,
    STOPBITS,
)


def to_s32(registers: list[int]) -> int:
    """两个寄存器拼成有符号 32 位整数（高位字在前）。"""
    value = (registers[0] << 16) | registers[1]
    return value - 0x1_0000_0000 if value >= 0x8000_0000 else value


def to_u32(registers: list[int]) -> int:
    """两个寄存器拼成无符号 32 位整数（高位字在前）。"""
    return (registers[0] << 16) | registers[1]


def read_input_registers(client: LoggedModbusSerialClient, address: int, count: int):
    """失败只重试当前地址；设备异常与本地超时分别记录。"""
    if READ_ATTEMPTS < 1:
        raise ValueError("READ_ATTEMPTS 必须至少为 1")
    for attempt in range(1, READ_ATTEMPTS + 1):
        logging.info("读取地址 %d、数量 %d，第 %d/%d 次尝试", address, count, attempt, READ_ATTEMPTS)
        response = client.read_input_registers(address=address, count=count, slave=SLAVE_ID)
        if isinstance(response, ExceptionResponse):
            logging.error("站号 %d 对地址 %d 返回设备异常码 0x%02X：%s", SLAVE_ID, address, response.exception_code, response)
            return None
        if response is None or isinstance(response, ModbusIOException):
            logging.warning("读地址 %d 未取得有效响应（第 %d/%d 次）：%s", address, attempt, READ_ATTEMPTS, response)
            continue
        if response.isError() or response.function_code != 0x04 or response.slave_id != SLAVE_ID or len(response.registers) != count:
            logging.warning("读地址 %d 收到不匹配响应（第 %d/%d 次）：%s", address, attempt, READ_ATTEMPTS, response)
            continue
        return response.registers
    # FC04 响应没有寄存器地址，后续相同长度的读取无法区分超时旧包。
    raise RuntimeError(f"地址 {address}、数量 {count} 尝试 {READ_ATTEMPTS} 次仍失败，停止本轮读取，避免迟到回包混入下一地址")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )

    client = LoggedModbusSerialClient(
        port=MASTER_PORT,
        # PyModbus 3.3 的串口客户端默认使用 RTU framer。
        baudrate=BAUDRATE,
        parity=PARITY,
        bytesize=BYTESIZE,
        stopbits=STOPBITS,
        timeout=RESPONSE_TIMEOUT,
        query_interval=QUERY_INTERVAL_S,
    )

    print(
        f"连接 Modbus RTU 从机：端口={MASTER_PORT}，站号={SLAVE_ID}，"
        f"串口={BAUDRATE}/{BYTESIZE}{PARITY}{STOPBITS}，响应超时={RESPONSE_TIMEOUT} 秒，最多尝试={READ_ATTEMPTS} 次"
    )

    try:
        if not client.connect():
            logging.error(
                "无法打开 %s。请检查 COM 端口、驱动和端口占用。",
                MASTER_PORT,
            )
            return

        # 第一步连通性探针：读产品型号（FC04 地址 2999）。
        model = read_input_registers(client, REG_PRODUCT_MODEL, 1)
        if model is None:
            print("连通性测试失败：读不到产品型号。请检查站号、A/B 接线和串口参数。")
            return
        print(f"连通性 OK：产品型号 = 0x{model[0]:04X}")

        # 客户端每次发送前自动等待串口静默满 QUERY_INTERVAL_S。
        active_power = None
        total_energy = None
        grid_freq = None
        status = None

        regs = read_input_registers(client, REG_ACTIVE_POWER, 2)
        if regs is not None:
            active_power = to_s32(regs)

        regs = read_input_registers(client, REG_TOTAL_ENERGY, 2)
        if regs is not None:
            total_energy = to_u32(regs)

        regs = read_input_registers(client, REG_GRID_FREQ, 1)
        if regs is not None:
            grid_freq = regs[0]

        regs = read_input_registers(client, REG_STATUS, 1)
        if regs is not None:
            status = regs[0]

        print("读取结果：")
        if active_power is not None:
            print(f"  有功功率   = {active_power} W")
        if total_energy is not None:
            print(f"  总发电量   = {total_energy} kWh")
        if grid_freq is not None:
            print(f"  电网频率   = {grid_freq / 100:.2f} Hz")
        if status is not None:
            print(f"  当前状态   = 0x{status:04X}")

    except (OSError, ValueError, RuntimeError) as error:
        logging.error(
            "串口通信失败：%s。请检查从机是否运行、站号、RS-485 接线和串口参数。",
            error,
        )
    except Exception:
        logging.exception("读取运行信息时发生意外错误。")
    finally:
        client.close()


if __name__ == "__main__":
    main()
