"""使用内存串口及真实 PyModbus 编解码器验证 park 主从通信，不打开 COM 口。"""

from __future__ import annotations

import contextlib
import io
import logging
import struct
import types
import unittest
from unittest import mock

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException
from pymodbus.factory import ServerDecoder
from pymodbus.framer.rtu_framer import ModbusRtuFramer
from pymodbus.pdu import ExceptionResponse
from pymodbus.register_read_message import ReadInputRegistersResponse
from pymodbus.utilities import computeCRC
import pymodbus.client.serial as serial_backend

import modbus_master as master
import modbus_slave as slave
import rtu_client


FIELD_REQUEST = bytes.fromhex("05 04 88 B8 00 01 9B CB")


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSerial:
    def __init__(self, clock, plans=(), initial=(), short_write=False):
        self.clock = clock
        self.plans = list(plans)
        self.events = [(t, bytearray(data)) for t, data in initial]
        self.timeout = 1.0
        self.write_timeout = 1.0
        self.writes = []
        self.flushes = 0
        self.short_write = short_write
        self.is_open = True

    @property
    def in_waiting(self):
        return sum(len(data) for t, data in self.events if t <= self.clock.now)

    def read(self, size):
        self.events.sort(key=lambda item: item[0])
        deadline = self.clock.now + self.timeout
        if not self.events or self.events[0][0] > deadline:
            self.clock.now = deadline
            return b""
        self.clock.now = max(self.clock.now, self.events[0][0])
        result = bytearray()
        while len(result) < size:
            if not self.events or self.events[0][0] > deadline:
                if len(result) < size:
                    self.clock.now = deadline
                break
            self.clock.now = max(self.clock.now, self.events[0][0])
            _, data = self.events[0]
            take = min(size - len(result), len(data))
            result.extend(data[:take])
            del data[:take]
            if not data:
                self.events.pop(0)
        return bytes(result)

    def write(self, request):
        self.writes.append((self.clock.now, request))
        if self.plans:
            plan = self.plans.pop(0)
            events = plan(request) if callable(plan) else plan
            for delay, data in events:
                self.events.append((self.clock.now + delay, bytearray(data)))
        return len(request) - 1 if self.short_write else len(request)

    def flush(self):
        self.flushes += 1

    def close(self):
        self.is_open = False

    def reset_input_buffer(self):
        raise AssertionError("不得无日志清空接收缓冲")


class CommunicationTests(unittest.TestCase):
    def setUp(self):
        previous_disable = logging.root.manager.disable
        logging.disable(logging.CRITICAL)
        self.addCleanup(logging.disable, previous_disable)
        self.clock = Clock()
        self.time_patch = mock.patch.object(rtu_client, "time", types.SimpleNamespace(monotonic=self.clock.monotonic))
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.context = slave.create_server_context()
        self.server_framer = slave.LoggingRtuFramer(ServerDecoder())

    def reply(self, request):
        packets = []

        def execute(decoded):
            response = decoded.execute(self.context[decoded.slave_id])
            response.slave_id = decoded.slave_id
            packets.append(self.server_framer.buildPacket(response))

        self.server_framer.processIncomingPacket(request, execute, [master.SLAVE_ID])
        self.assertEqual(len(packets), 1)
        return packets[0]

    def response(self, words, station=None):
        message = ReadInputRegistersResponse(words, slave=master.SLAVE_ID if station is None else station)
        return self.server_framer.buildPacket(message)

    def client(self, plans=(), initial=(), short_write=False, timeout=1.0):
        client = rtu_client.LoggedModbusSerialClient(port="MEMORY", timeout=timeout, query_interval=0.35)
        socket = FakeSerial(self.clock, plans, initial, short_write)
        socket.timeout = timeout
        client.socket = socket
        self.addCleanup(client.close)
        return client, socket

    def normal_plan(self, request):
        return [(0.05, self.reply(request))]

    def read(self, client, address=master.REG_PRODUCT_MODEL, count=1):
        return client.read_input_registers(address=address, count=count, slave=master.SLAVE_ID)

    def test_all_simulated_points_through_real_pymodbus_codec(self):
        client, socket = self.client([self.normal_plan] * 5)
        expected = [
            (master.REG_PRODUCT_MODEL, 1, [0x1121]),
            (master.REG_ACTIVE_POWER, 2, [0, 2500]),
            (master.REG_TOTAL_ENERGY, 2, [0, 12345]),
            (master.REG_GRID_FREQ, 1, [5000]),
            (master.REG_STATUS, 1, [3]),
        ]
        for address, count, words in expected:
            result = self.read(client, address, count)
            self.assertFalse(result.isError())
            self.assertEqual(result.slave_id, master.SLAVE_ID)
            self.assertEqual(result.registers, words)
        self.assertEqual(len(socket.writes), 5)
        self.assertEqual(socket.flushes, 5)
        self.assertEqual(master.to_s32([0, 2500]), 2500)
        self.assertEqual(master.to_s32([0xFFFF, 0xFFFF]), -1)
        self.assertEqual(master.to_u32([0, 12345]), 12345)

    def test_field_request_does_not_end_wait_for_station_nine(self):
        def plan(request):
            response = self.reply(request)
            return [(0.05, FIELD_REQUEST), (0.30, response[:4]), (0.80, response[4:])]

        client, socket = self.client([plan])
        result = self.read(client, master.REG_ACTIVE_POWER, 2)
        self.assertFalse(result.isError())
        self.assertEqual(result.registers, [0, 2500])
        self.assertAlmostEqual(self.clock.now, 1.15)
        self.assertEqual(len(socket.writes), 1)

    def test_original_client_fails_with_same_foreign_request_ordering(self):
        baseline_clock = Clock()
        baseline = ModbusSerialClient(port="MEMORY", timeout=1.0, reset_socket=False, retry_on_empty=False, retry_on_invalid=False)

        def plan(request):
            return [(0.05, FIELD_REQUEST), (0.30, self.reply(request))]

        socket = FakeSerial(baseline_clock, [plan])
        baseline.socket = socket
        fake_time = types.SimpleNamespace(time=baseline_clock.monotonic, sleep=baseline_clock.sleep)
        try:
            with mock.patch.object(serial_backend, "time", fake_time):
                result = self.read(baseline, master.REG_ACTIVE_POWER, 2)
            self.assertIsInstance(result, ModbusIOException)
        finally:
            baseline.close()

    def test_foreign_response_then_expected_response(self):
        other_station = 1 if master.SLAVE_ID != 1 else 2
        foreign = self.response([0x1234], other_station)
        client, _ = self.client([[(0.05, foreign), (0.20, self.response([0x1121]))]])
        self.assertEqual(self.read(client).registers, [0x1121])
        self.assertAlmostEqual(self.clock.now, 0.55)

    def test_wrong_register_count_is_not_accepted(self):
        client, _ = self.client([[(0.05, self.response([0x1121])), (0.20, self.response([0, 2500]))]])
        self.assertEqual(self.read(client, master.REG_ACTIVE_POWER, 2).registers, [0, 2500])

    def test_request_echo_then_fragmented_response(self):
        def plan(request):
            response = self.reply(request)
            return [(0.01, request), (0.10, response[:1]), (0.20, response[1:3]), (0.40, response[3:])]

        client, _ = self.client([plan])
        self.assertEqual(self.read(client).registers, [0x1121])

    def test_bad_crc_noise_then_valid_frame(self):
        response = self.response([0x1121])
        bad = response[:-1] + bytes([response[-1] ^ 1])
        client, _ = self.client([[(0.05, b"\xFF\xFF" + bad + response)]])
        self.assertEqual(self.read(client).registers, [0x1121])

    def test_five_byte_device_exception_returns_immediately(self):
        client, _ = self.client([self.normal_plan])
        result = self.read(client, 5000, 1)
        self.assertIsInstance(result, ExceptionResponse)
        self.assertEqual(result.exception_code, 2)
        self.assertAlmostEqual(self.clock.now, 0.40)

    def test_device_exception_is_not_retried_or_logged_as_local_timeout(self):
        client, socket = self.client([self.normal_plan])
        logging.disable(logging.NOTSET)
        with self.assertLogs(level="ERROR") as captured:
            result = master.read_input_registers(client, 5000, 1)
        self.assertIsNone(result)
        self.assertEqual(len(socket.writes), 1)
        self.assertIn("设备异常码 0x02", "\n".join(captured.output))

    def test_partial_frame_waits_until_configured_deadline(self):
        client, _ = self.client([[(0.05, self.response([0x1121])[:4])]], timeout=2.0)
        self.assertIsInstance(self.read(client), ModbusIOException)
        self.assertAlmostEqual(self.clock.now, 2.35)

    def test_foreign_input_does_not_restart_response_deadline(self):
        client, _ = self.client([[(0.80, FIELD_REQUEST), (1.10, self.response([0x1121]))]])
        self.assertIsInstance(self.read(client), ModbusIOException)
        self.assertAlmostEqual(self.clock.now, 1.35)

    def test_pre_send_input_extends_quiet_interval_and_is_logged(self):
        client, socket = self.client([self.normal_plan], initial=[(0.10, FIELD_REQUEST), (0.40, FIELD_REQUEST)])
        logging.disable(logging.NOTSET)
        with self.assertLogs("rtu_client", level="INFO") as captured:
            result = self.read(client)
        self.assertEqual(result.registers, [0x1121])
        self.assertAlmostEqual(socket.writes[0][0], 0.75)
        self.assertIn("TX 前 RX", "\n".join(captured.output))

    def test_timeout_retries_only_same_address(self):
        client, socket = self.client([[], self.normal_plan])
        with mock.patch.object(master, "READ_ATTEMPTS", 3):
            words = master.read_input_registers(client, master.REG_ACTIVE_POWER, 2)
        self.assertEqual(words, [0, 2500])
        self.assertEqual(len(socket.writes), 2)
        self.assertEqual(socket.writes[0][1], socket.writes[1][1])

    def test_late_reply_is_drained_before_retry(self):
        stale = self.response([0xAAAA])
        client, socket = self.client([[(1.10, stale)], self.normal_plan])
        words = master.read_input_registers(client, master.REG_PRODUCT_MODEL, 1)
        self.assertEqual(words, [0x1121])
        self.assertEqual(len(socket.writes), 2)
        self.assertAlmostEqual(socket.writes[1][0], 1.80)

    def test_retry_exhaustion_stops_main_before_next_address(self):
        client, socket = self.client([self.normal_plan, [], []])
        with mock.patch.object(master, "LoggedModbusSerialClient", return_value=client), mock.patch.object(master, "READ_ATTEMPTS", 2), contextlib.redirect_stdout(io.StringIO()):
            master.main()
        addresses = [int.from_bytes(request[2:4], "big") for _, request in socket.writes]
        self.assertEqual(addresses, [master.REG_PRODUCT_MODEL, master.REG_ACTIVE_POWER, master.REG_ACTIVE_POWER])
        self.assertFalse(socket.is_open)

    def test_busy_input_stops_without_sending(self):
        events = [(0.20 * i, b"\xFF") for i in range(1, 9)]
        client, socket = self.client(initial=events)
        with self.assertRaises(RuntimeError):
            self.read(client)
        self.assertEqual(socket.writes, [])

    def test_short_write_does_not_start_response_wait(self):
        client, socket = self.client(short_write=True)
        # PyModbus 事务层将串口写入异常包装为 ModbusIOException。
        self.assertIsInstance(self.read(client), ModbusIOException)
        self.assertEqual(socket.flushes, 0)

    def test_slave_logs_raw_chunks_decoded_request_and_response(self):
        client, _ = self.client()
        request_body = bytes([master.SLAVE_ID, 4, 0x0B, 0xBC, 0, 2])
        request = request_body + struct.pack(">H", computeCRC(request_body))
        decoded = []
        logging.disable(logging.NOTSET)
        with self.assertLogs(level="INFO") as captured:
            self.server_framer.processIncomingPacket(request[:3], decoded.append, [master.SLAVE_ID])
            self.assertEqual(decoded, [])
            self.server_framer.processIncomingPacket(request[3:], decoded.append, [master.SLAVE_ID])
            response = decoded[0].execute(self.context[master.SLAVE_ID])
            response.slave_id = master.SLAVE_ID
            packet = self.server_framer.buildPacket(response)
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].address, 3004)
        self.assertEqual(response.registers, [0, 2500])
        self.assertTrue(rtu_client.valid_crc(packet))
        logs = "\n".join(captured.output)
        self.assertIn("RX 收到串口数据", logs)
        self.assertIn("请求解析成功", logs)
        self.assertIn("地址=3004、数量=2", logs)
        self.assertIn("TX 响应帧已生成", logs)

    def test_slave_does_not_answer_industrial_station_request(self):
        decoded = []
        self.server_framer.processIncomingPacket(FIELD_REQUEST, decoded.append, [master.SLAVE_ID])
        self.assertEqual(decoded, [])

    def test_logged_field_request_matches_supplied_hex_frame(self):
        client, _ = self.client([[(0.05, FIELD_REQUEST), (0.10, self.response([0x1121]))]])
        logging.disable(logging.NOTSET)
        with self.assertLogs("rtu_client", level="INFO") as captured:
            self.assertEqual(self.read(client).registers, [0x1121])
        logs = "\n".join(captured.output)
        self.assertIn("站号=5、功能码=04、地址=35000、数量=1", logs)
        self.assertIn("05 04 88 B8 00 01 9B CB", logs)


if __name__ == "__main__":
    unittest.main()
