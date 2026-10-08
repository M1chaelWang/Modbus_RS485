"""运行 Modbus RTU 从机（逆变器模拟器），在串口上应答 FC04 读输入寄存器请求。"""

from __future__ import annotations

import logging
import sys

from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusServerContext,
    ModbusSlaveContext,
)
from pymodbus.framer.rtu_framer import ModbusRtuFramer
from pymodbus.server import StartSerialServer

from config import (
    BAUDRATE,
    BYTESIZE,
    IR_BLOCK_SIZE,
    IR_BLOCK_START,
    PARITY,
    SLAVE_ID,
    SLAVE_INPUT_REGISTER_VALUES,
    SLAVE_PORT,
    STOPBITS,
)


class LoggingRtuFramer(ModbusRtuFramer):
    """记录串口原始数据、成功解析的请求以及生成的完整响应帧。"""

    def processIncomingPacket(self, data, callback, slave, **kwargs):
        # 一次回调的数据可能只是片段，也可能包含多个请求。
        logging.info("RX 收到串口数据（%d 字节）：%s", len(data), data.hex(" ").upper())

        def log_request(request):
            logging.info(
                "请求解析成功：站号=%d、功能码=%02X、地址=%s、数量=%s",
                request.slave_id, request.function_code,
                getattr(request, "address", "未知"), getattr(request, "count", "未知"),
            )
            return callback(request)

        return super().processIncomingPacket(data, log_request, slave, **kwargs)

    def buildPacket(self, message):
        packet = super().buildPacket(message)
        # 此处是帧生成日志，不代表对端已收到或无线链路已完成传输。
        logging.info(
            "TX 响应帧已生成（待发送，%d 字节）：%s",
            len(packet), packet.hex(" ").upper(),
        )
        return packet


def create_server_context() -> ModbusServerContext:
    """在协议地址上暴露输入寄存器（FC04），模拟逆变器运行信息。"""
    # PyModbus 3.3 的数据块存在 +1 寻址偏移：数据块起点声明为 START+1 时，
    # 主机的 PDU 地址 START 恰好落到 values[0]。因此 values[addr-START] 即地址 addr 的值。
    ir_values = [0] * IR_BLOCK_SIZE
    for address, value in SLAVE_INPUT_REGISTER_VALUES.items():
        ir_values[address - IR_BLOCK_START] = value

    slave_context = ModbusSlaveContext(
        ir=ModbusSequentialDataBlock(IR_BLOCK_START + 1, ir_values),
    )
    return ModbusServerContext(slaves={SLAVE_ID: slave_context}, single=False)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )

    print(
        f"启动 Modbus RTU 从机（逆变器模拟器）：端口={SLAVE_PORT}，站号={SLAVE_ID}，"
        f"串口={BAUDRATE}/{BYTESIZE}{PARITY}{STOPBITS}"
    )
    print(f"暴露输入寄存器（FC04）：{sorted(SLAVE_INPUT_REGISTER_VALUES)}")
    print("控制台记录 RX 原始数据、匹配本机站号的完整请求与 TX 响应帧。")

    try:
        StartSerialServer(
            context=create_server_context(),
            port=SLAVE_PORT,
            # StartAsyncSerialServer 的默认 framer 是 ASCII，必须显式指定 RTU。
            framer=LoggingRtuFramer,
            baudrate=BAUDRATE,
            parity=PARITY,
            bytesize=BYTESIZE,
            stopbits=STOPBITS,
        )
    except (OSError, ValueError) as error:
        logging.error(
            "无法在 %s 启动从机：%s。请检查 COM 端口、驱动、端口占用和 RS-485 接线。",
            SLAVE_PORT,
            error,
        )
    except KeyboardInterrupt:
        print("\n从机已停止。")
    except Exception:
        logging.exception("从机意外退出。")


if __name__ == "__main__":
    main()
