"""运行 KSG1-250K Modbus RTU 只读仿真从机。"""

from __future__ import annotations

import logging

from pymodbus.datastore import (
    ModbusServerContext,
    ModbusSlaveContext,
    ModbusSparseDataBlock,
)
from pymodbus.framer.rtu_framer import ModbusRtuFramer
from pymodbus.server import StartSerialServer

from config import (
    BAUDRATE,
    BYTESIZE,
    PARITY,
    READ_HOLDING_REGISTERS,
    READ_INPUT_REGISTERS,
    SLAVE_ID,
    SLAVE_B_PORT,
    STOPBITS,
)
from ksg1_registers import READ_BLOCKS, build_pymodbus_datastore_values


class LoggingRtuFramer(ModbusRtuFramer):
    """在 ModbusRtuFramer 基础上，把收到的请求与发出的响应打印为日志。"""

    def processIncomingPacket(self, data, callback, slave, **kwargs):
        # data 是串口一次读到的数据块，可能包含不完整帧或多个帧。
        logging.info(
            "收到数据（%d 字节）：%s",
            len(data),
            " ".join(f"{byte:02X}" for byte in data),
        )
        super().processIncomingPacket(data, callback, slave, **kwargs)

    def buildPacket(self, message):
        packet = super().buildPacket(message)
        logging.info(
            "发送响应（%d 字节）：%s",
            len(packet),
            " ".join(f"{byte:02X}" for byte in packet),
        )
        return packet


def create_server_context() -> ModbusServerContext:
    """创建包含 KSG1 03H Holding 与 04H Input Registers 的只读数据区。"""
    slave_context = ModbusSlaveContext(
        hr=ModbusSparseDataBlock(
            # 从机 B：数值编码值统一为双数，便于与从机 A 区分。
            build_pymodbus_datastore_values(READ_HOLDING_REGISTERS, "B"),
            mutable=False,
        ),
        ir=ModbusSparseDataBlock(
            build_pymodbus_datastore_values(READ_INPUT_REGISTERS, "B"),
            mutable=False,
        ),
    )
    return ModbusServerContext(slaves={SLAVE_ID: slave_context}, single=False)


def _format_blocks(function_code: int) -> str:
    """格式化已暴露的文档地址块。"""
    blocks = [block for block in READ_BLOCKS if block.function_code == function_code]
    return ", ".join(f"{block.address}–{block.end_address}" for block in blocks)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    print(
        f"启动 KSG1-250K Modbus RTU 仿真从机：端口={SLAVE_B_PORT}，"
        f"站号={SLAVE_ID}，串口={BAUDRATE}/{BYTESIZE}{PARITY}{STOPBITS}"
    )
    print(f"03H Holding Registers：{_format_blocks(READ_HOLDING_REGISTERS)}")
    print(f"04H Input Registers：{_format_blocks(READ_INPUT_REGISTERS)}")
    print("仅模拟协议中明确的可读地址块；未定义或保留地址将返回 Illegal Address。")
    print("从机收到请求、发送响应时都会打印十六进制帧日志。")

    try:
        StartSerialServer(
            context=create_server_context(),
            port=SLAVE_B_PORT,
            # PyModbus 3.3 的 StartAsyncSerialServer 默认 ASCII，必须显式指定 RTU；
            # LoggingRtuFramer 在 RTU 基础上打印收发帧日志。
            framer=LoggingRtuFramer,
            baudrate=BAUDRATE,
            parity=PARITY,
            bytesize=BYTESIZE,
            stopbits=STOPBITS,
        )
    except (OSError, ValueError) as error:
        logging.error(
            "无法在 %s 启动从机：%s。请检查 COM 端口、驱动、端口占用和 RS-485 接线。",
            SLAVE_B_PORT,
            error,
        )
    except KeyboardInterrupt:
        print("\n从机已停止。")
    except Exception:
        logging.exception("从机意外退出。")


if __name__ == "__main__":
    main()
