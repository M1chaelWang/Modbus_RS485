"""KSTAR KSG1-250K V3.6 的只读 Modbus 点表、模拟数据与编解码工具。"""

from __future__ import annotations

from dataclasses import dataclass
from struct import pack, unpack
from typing import Final, Iterable, Mapping

from config import (
    PYMODBUS_33_SERVER_ADDRESS_OFFSET,
    READ_HOLDING_REGISTERS,
    READ_INPUT_REGISTERS,
)


@dataclass(frozen=True)
class ReadBlock:
    """一个可连续读取、且在协议中明确列出的地址块。"""

    function_code: int
    address: int
    count: int
    name: str

    @property
    def end_address(self) -> int:
        return self.address + self.count - 1


@dataclass(frozen=True)
class RegisterPoint:
    """一个协议点位；地址始终为 KSG1 文档与 Modbus PDU 使用的零基地址。"""

    function_code: int
    address: int
    name: str
    data_type: str
    value: int | float | str | bytes
    scale: float = 1.0
    unit: str = ""
    width: int | None = None
    byte_offset: int = 0

    @property
    def byte_count(self) -> int:
        if self.data_type in {"ascii", "mac"}:
            if self.width is None:
                raise ValueError(f"{self.name} 必须指定字节宽度")
            return self.width
        return {"u8": 1, "u16": 2, "s16": 2, "u32": 4, "s32": 4}[self.data_type]

    @property
    def register_count(self) -> int:
        return (self.byte_offset + self.byte_count + 1) // 2


# 只读取文档中有明确字段定义的地址块；不读取 3154–3193、3320–4999、7400–10224 等保留区。
READ_BLOCKS: Final[tuple[ReadBlock, ...]] = (
    ReadBlock(READ_HOLDING_REGISTERS, 3200, 33, "03H 设备身份、版本、SN 与 PLC MAC"),
    ReadBlock(READ_INPUT_REGISTERS, 2964, 190, "04H PV、直流母线、电网遥测、状态与能量"),
    ReadBlock(READ_INPUT_REGISTERS, 3194, 126, "04H PLC、保护与 PQ/DRM 配置镜像"),
    ReadBlock(READ_INPUT_REGISTERS, 5000, 2400, "04H PV1–PV12 共 1200 路组串电压/电流"),
)


def _point(
    function_code: int,
    address: int,
    name: str,
    data_type: str,
    value: int | float | str | bytes,
    scale: float = 1.0,
    unit: str = "",
    width: int | None = None,
    byte_offset: int = 0,
) -> RegisterPoint:
    return RegisterPoint(
        function_code,
        address,
        name,
        data_type,
        value,
        scale,
        unit,
        width,
        byte_offset,
    )


def _pv_values(channel: int) -> tuple[float, float, int]:
    """为 PV 输入生成稳定且符合 G110KT 额定范围的模拟值。"""
    voltage = (10_700 + ((channel * 17) % 45)) / 10
    current = (810 + ((channel * 11) % 40)) / 100
    power = round(voltage * current)
    return voltage, current, power


def _build_points() -> tuple[RegisterPoint, ...]:
    points: list[RegisterPoint] = [
        # 03H：文档 3.2.1，两个 U8/ASCII 字节装入一个 16-bit register，字节序为高字节在前。
        _point(READ_HOLDING_REGISTERS, 3200, "逆变器型号", "ascii", "G110KT", width=10),
        _point(READ_HOLDING_REGISTERS, 3205, "DSP1 主版本", "u8", 3, byte_offset=0),
        _point(READ_HOLDING_REGISTERS, 3205, "ARM1 主版本", "u8", 6, byte_offset=1),
        _point(
            READ_HOLDING_REGISTERS,
            3206,
            "设备序列号",
            "ascii",
            "G110KT-SIM-20231010-01",
            width=22,
        ),
        _point(
            READ_HOLDING_REGISTERS,
            3217,
            "PLC MAC 地址",
            "mac",
            bytes((0x02, 0x4B, 0x53, 0x47, 0x31, 0x01)),
            width=6,
        ),
        _point(READ_HOLDING_REGISTERS, 3220, "并网规范编码", "u8", 0, byte_offset=0),
        _point(READ_HOLDING_REGISTERS, 3220, "保留状态字节", "u8", 0, byte_offset=1),
        _point(READ_HOLDING_REGISTERS, 3221, "软件标识", "ascii", "SIM-KSG1-RTU-V36", width=16),
        _point(READ_HOLDING_REGISTERS, 3229, "DSP1 次版本", "u8", 2, byte_offset=0),
        _point(READ_HOLDING_REGISTERS, 3229, "ARM1 次版本", "u8", 1, byte_offset=1),
        _point(READ_HOLDING_REGISTERS, 3230, "DSP2 主版本", "u8", 1, byte_offset=0),
        _point(READ_HOLDING_REGISTERS, 3230, "DSP2 次版本", "u8", 0, byte_offset=1),
        _point(READ_HOLDING_REGISTERS, 3231, "生产序号", "u32", 20231010),
    ]

    # 04H：文档 2964–2999，PV12 到 PV4，每路为电压、电流、功率（功率占两个寄存器）。
    for pv in range(12, 3, -1):
        address = 2964 + (12 - pv) * 4
        voltage, current, power = _pv_values(pv)
        points.extend(
            (
                _point(READ_INPUT_REGISTERS, address, f"PV{pv} 电压", "u16", voltage, 0.1, "V"),
                _point(READ_INPUT_REGISTERS, address + 1, f"PV{pv} 电流", "s16", current, 0.01, "A"),
                _point(READ_INPUT_REGISTERS, address + 2, f"PV{pv} 功率", "s32", power, 1, "W"),
            )
        )

    # 文档 3000–3011：PV1–PV3 的电压/电流/功率布局与 PV4–PV12 不同。
    for pv in range(1, 4):
        voltage, _, _ = _pv_values(pv)
        points.append(_point(READ_INPUT_REGISTERS, 2999 + pv, f"PV{pv} 电压", "u16", voltage, 0.1, "V"))
    for pv in range(1, 4):
        _, current, _ = _pv_values(pv)
        points.append(_point(READ_INPUT_REGISTERS, 3002 + pv, f"PV{pv} 电流", "s16", current, 0.01, "A"))
    for pv in range(1, 4):
        _, _, power = _pv_values(pv)
        points.append(_point(READ_INPUT_REGISTERS, 3004 + pv * 2, f"PV{pv} 功率", "s32", power, 1, "W"))

    # 文档 3012–3153：设备运行时遥测与状态。未在文档中标名的保留字仍返回 0，
    # 但不会显示为业务点，避免将保留字段误解为真实逆变器语义。
    points.extend(
        (
            _point(READ_INPUT_REGISTERS, 3012, "正母线电压 PBUS", "u16", 610.2, 0.1, "V"),
            _point(READ_INPUT_REGISTERS, 3013, "负母线电压 NBUS", "u16", 608.7, 0.1, "V"),
            _point(READ_INPUT_REGISTERS, 3014, "电网 RS 线电压", "u16", 800.4, 0.1, "V"),
            _point(READ_INPUT_REGISTERS, 3015, "电网 ST 线电压", "u16", 798.9, 0.1, "V"),
            _point(READ_INPUT_REGISTERS, 3016, "电网 TR 线电压", "u16", 801.1, 0.1, "V"),
            _point(READ_INPUT_REGISTERS, 3017, "电网 RS 频率", "u16", 50.00, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3018, "电网 ST 频率", "u16", 50.01, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3019, "电网 TR 频率", "u16", 49.99, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3020, "R 相电流", "s16", 138.42, 0.01, "A"),
            _point(READ_INPUT_REGISTERS, 3021, "S 相电流", "s16", 137.96, 0.01, "A"),
            _point(READ_INPUT_REGISTERS, 3022, "T 相电流", "s16", 138.17, 0.01, "A"),
            _point(READ_INPUT_REGISTERS, 3023, "交流有功功率", "s32", 96_500, 1, "W"),
            _point(READ_INPUT_REGISTERS, 3025, "功率因数", "s16", 0.98, 0.01, ""),
            _point(READ_INPUT_REGISTERS, 3027, "DSP 告警字", "u16", 0, 1, ""),
            _point(READ_INPUT_REGISTERS, 3028, "DSP 故障字", "u32", 0, 1, ""),
            _point(READ_INPUT_REGISTERS, 3030, "运行状态", "u8", 1, byte_offset=0),
            _point(READ_INPUT_REGISTERS, 3030, "机型状态", "u8", 0, byte_offset=1),
            _point(READ_INPUT_REGISTERS, 3031, "风机 A 转速", "u16", 1_420, 1, "r/min"),
            _point(READ_INPUT_REGISTERS, 3032, "风机 B 转速", "u16", 1_405, 1, "r/min"),
            _point(READ_INPUT_REGISTERS, 3033, "风机 C 转速", "u16", 1_398, 1, "r/min"),
            _point(READ_INPUT_REGISTERS, 3035, "额定功率", "u16", 110, 1, "kW"),
            _point(READ_INPUT_REGISTERS, 3036, "ARM 运行状态", "u8", 1, byte_offset=0),
            _point(READ_INPUT_REGISTERS, 3036, "ARM 控制状态", "u8", 0, byte_offset=1),
            _point(READ_INPUT_REGISTERS, 3038, "当日发电量", "u32", 638.4, 0.1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3040, "累计发电量", "u32", 1_284_760, 1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3042, "当日运行时长", "u16", 8, 1, "h"),
            _point(READ_INPUT_REGISTERS, 3043, "额定并网电压", "u16", 800.0, 0.1, "V"),
            _point(READ_INPUT_REGISTERS, 3044, "并网延时", "u16", 60, 1, "s"),
            _point(READ_INPUT_REGISTERS, 3047, "额定频率", "u16", 50.00, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3049, "有功限功率设定", "u16", 100, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3050, "无功设定", "u16", 0, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3051, "无功设定百分比", "u8", 0, 1, "%", byte_offset=0),
            _point(READ_INPUT_REGISTERS, 3052, "交流视在功率", "s32", 98_100, 1, "VA"),
            _point(READ_INPUT_REGISTERS, 3054, "交流无功功率", "s32", 1_900, 1, "var"),
            _point(READ_INPUT_REGISTERS, 3057, "绝缘阻抗", "u16", 2_500, 1, "kΩ"),
            _point(READ_INPUT_REGISTERS, 3059, "频率设定", "u16", 50.00, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3062, "ARM2 告警字", "u8", 0, byte_offset=0),
            _point(READ_INPUT_REGISTERS, 3062, "ARM2 状态", "u8", 1, byte_offset=1),
            _point(READ_INPUT_REGISTERS, 3063, "G110KT 机型编码", "u16", 0x100),
        )
    )

    for channel in range(1, 25):
        points.append(
            _point(
                READ_INPUT_REGISTERS,
                3063 + channel,
                f"ARM 电流通道 {channel}",
                "s16",
                0.0,
                0.01,
                "A",
            )
        )

    points.extend(
        (
            _point(READ_INPUT_REGISTERS, 3090, "累计运行时长", "u32", 12_876, 1, "h"),
            _point(READ_INPUT_REGISTERS, 3092, "累计并网时长", "u32", 12_400, 1, "h"),
            _point(READ_INPUT_REGISTERS, 3096, "额定光伏功率", "u32", 132_000, 1, "W"),
            _point(READ_INPUT_REGISTERS, 3098, "当前直流功率", "u32", 102_400, 1, "W"),
            _point(READ_INPUT_REGISTERS, 3100, "正向累计电量", "u32", 1_284_760, 1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3102, "反向累计电量", "u32", 0, 1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3104, "并网累计电量", "u32", 1_284_760, 1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3117, "RS485-2 从站地址", "u16", 1),
            _point(READ_INPUT_REGISTERS, 3118, "RS485-2 波特率编码", "u16", 2),
            _point(READ_INPUT_REGISTERS, 3119, "RS485 通讯协议", "u16", 1),
            _point(READ_INPUT_REGISTERS, 3120, "RS485-1 从站地址", "u16", 1),
            _point(READ_INPUT_REGISTERS, 3121, "RS485-1 波特率编码", "u16", 2),
            _point(READ_INPUT_REGISTERS, 3122, "机型编码", "u16", 0x100),
            _point(READ_INPUT_REGISTERS, 3124, "额定电流", "u16", 140, 1, "A"),
            _point(READ_INPUT_REGISTERS, 3125, "告警记录", "u16", 0),
            _point(READ_INPUT_REGISTERS, 3126, "I/V 扫描状态", "u16", 0),
        )
    )

    # 文档 3194–3319：只为具名的 PLC / PQ / DRM 及保护曲线相关点赋模拟值。
    points.extend(
        (
            _point(READ_INPUT_REGISTERS, 3194, "PLC 状态", "u16", 1),
            _point(READ_INPUT_REGISTERS, 3195, "PLC 使能", "u16", 1),
            _point(READ_INPUT_REGISTERS, 3196, "有功功率限制", "u16", 110, 1, "kW"),
            _point(READ_INPUT_REGISTERS, 3197, "漏电流保护阈值", "u16", 1.20, 0.01, "A"),
            _point(READ_INPUT_REGISTERS, 3198, "残余电流保护阈值", "u16", 0.30, 0.01, "A"),
            _point(READ_INPUT_REGISTERS, 3203, "机型/电网配置字", "u16", 0x1000),
            _point(READ_INPUT_REGISTERS, 3204, "CT 功率校正", "s16", 0, 1, "W"),
            _point(READ_INPUT_REGISTERS, 3205, "CT 配置", "u16", 1),
            _point(READ_INPUT_REGISTERS, 3226, "爬坡率", "u16", 10, 1, "%/s"),
            _point(READ_INPUT_REGISTERS, 3228, "并网延时", "u16", 60, 1, "s"),
            _point(READ_INPUT_REGISTERS, 3231, "过频保护值", "u16", 50.00, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3232, "欠频保护值", "u16", 49.00, 0.01, "Hz"),
            _point(READ_INPUT_REGISTERS, 3235, "有功上限", "u16", 100, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3236, "有功下限", "u16", 0, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3241, "当日能量复位值", "u16", 0, 0.1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3242, "能量计数器", "u32", 1_284_760, 1, "kWh"),
            _point(READ_INPUT_REGISTERS, 3248, "SVG 延时", "u16", 20, 1, "s"),
            _point(READ_INPUT_REGISTERS, 3249, "SVG 使能", "u16", 0),
            _point(READ_INPUT_REGISTERS, 3250, "I/V 扫描使能", "u16", 0),
            _point(READ_INPUT_REGISTERS, 3251, "R 相 CT 比", "u16", 4096),
            _point(READ_INPUT_REGISTERS, 3252, "DRM 阈值 1", "u16", 0, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3253, "DRM 阈值 2", "u16", 50, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3254, "DRM 阈值 3", "u16", 75, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3255, "DRM 阈值 4", "u16", 100, 1, "%"),
            _point(READ_INPUT_REGISTERS, 3270, "ARM2 告警字", "u8", 0, byte_offset=0),
            _point(READ_INPUT_REGISTERS, 3270, "ARM2 状态", "u8", 1, byte_offset=1),
            _point(READ_INPUT_REGISTERS, 3271, "G110KT 机型编码", "u16", 0x100),
            _point(READ_INPUT_REGISTERS, 3281, "PQ 起始点", "u16", 20.00, 0.01, "%Pmax"),
            _point(READ_INPUT_REGISTERS, 3282, "PQ 结束点", "u16", 80.00, 0.01, "%Pmax"),
            _point(READ_INPUT_REGISTERS, 3283, "PQ 斜率", "u16", 1.00, 0.01, "%Q/%Pmax"),
        )
    )

    # 协议 5000–7399：12 个 MPPT、每个 100 路组串；每路两个寄存器（电压/电流）。
    for pv in range(1, 13):
        for string_number in range(1, 101):
            address = 5000 + (pv - 1) * 200 + (string_number - 1) * 2
            voltage = (10_600 + pv * 15 + (string_number % 10) * 2) / 10
            current = (800 + (pv % 4) * 7 + (string_number % 5) * 3) / 100
            points.extend(
                (
                    _point(
                        READ_INPUT_REGISTERS,
                        address,
                        f"PV{pv} 组串 {string_number} 电压",
                        "u16",
                        voltage,
                        0.1,
                        "V",
                    ),
                    _point(
                        READ_INPUT_REGISTERS,
                        address + 1,
                        f"PV{pv} 组串 {string_number} 电流",
                        "s16",
                        current,
                        0.01,
                        "A",
                    ),
                )
            )

    return tuple(points)


POINTS: Final[tuple[RegisterPoint, ...]] = _build_points()


def _block_addresses(function_code: int) -> set[int]:
    addresses: set[int] = set()
    for block in READ_BLOCKS:
        if block.function_code == function_code:
            addresses.update(range(block.address, block.end_address + 1))
    return addresses


def force_parity(raw_value: int, *, odd: bool) -> int:
    """把整数调整到指定奇偶性；已是目标奇偶则原样返回，否则加 1 翻转。"""
    if (raw_value & 1) == int(odd):
        return raw_value
    return raw_value + 1


def _encode_point(point: RegisterPoint, variant: str | None = None) -> bytes:
    """将一个工程值编码为文档要求的 Big-Endian 字节序。

    variant 为 "A" 时把数值编码值调成单数、"B" 时调成双数，用于区分两个从机；
    ascii/mac 等非数值类型不受影响，保持两从机一致。
    """
    if point.data_type == "ascii":
        raw = str(point.value).encode("ascii")
        assert point.width is not None
        if len(raw) > point.width:
            raise ValueError(f"{point.name} 超过 {point.width} 字节")
        return raw.ljust(point.width, b" ")
    if point.data_type == "mac":
        raw = bytes(point.value)
        assert point.width is not None
        if len(raw) != point.width:
            raise ValueError(f"{point.name} 必须正好为 {point.width} 字节")
        return raw

    raw_value = round(float(point.value) / point.scale)
    if variant is not None:
        raw_value = force_parity(raw_value, odd=(variant == "A"))
    formats = {"u8": ">B", "u16": ">H", "s16": ">h", "u32": ">I", "s32": ">i"}
    try:
        return pack(formats[point.data_type], raw_value)
    except (KeyError, OverflowError) as error:
        raise ValueError(f"无法编码 {point.name} 的值 {point.value}") from error


def build_document_registers(
    function_code: int,
    variant: str | None = None,
) -> dict[int, int]:
    """构造按文档地址索引的 16-bit 值；未具名的已定义字节默认返回 0。

    variant 透传给 _encode_point，用于让两个从机返回不同奇偶性的数值。
    """
    addresses = _block_addresses(function_code)
    words = {address: bytearray(2) for address in addresses}

    for point in POINTS:
        if point.function_code != function_code:
            continue
        payload = _encode_point(point, variant)
        start_byte = point.address * 2 + point.byte_offset
        for index, value in enumerate(payload):
            absolute_byte = start_byte + index
            address, byte_index = divmod(absolute_byte, 2)
            if address not in words:
                raise ValueError(
                    f"{point.name} 写入了未声明的 {function_code:02X}H 地址 {address}"
                )
            words[address][byte_index] = value

    return {address: int.from_bytes(value, byteorder="big") for address, value in words.items()}


def build_pymodbus_datastore_values(
    function_code: int,
    variant: str | None = None,
) -> dict[int, int]:
    """构造 PyModbus 3.3 数据块地址；该版本服务端访问时会额外加 1。"""
    return {
        address + PYMODBUS_33_SERVER_ADDRESS_OFFSET: value
        for address, value in build_document_registers(function_code, variant).items()
    }


def decode_point(point: RegisterPoint, registers: Mapping[int, int]) -> int | float | str | bytes:
    """从以文档地址索引的主机响应中解码一个点位。"""
    raw = bytearray()
    start_byte = point.address * 2 + point.byte_offset
    for offset in range(point.byte_count):
        absolute_byte = start_byte + offset
        address, byte_index = divmod(absolute_byte, 2)
        try:
            word = registers[address]
        except KeyError as error:
            raise ValueError(f"响应缺少 {point.name} 所需的地址 {address}") from error
        raw.append(word.to_bytes(2, byteorder="big")[byte_index])

    if point.data_type == "ascii":
        return raw.decode("ascii", errors="replace").rstrip(" \x00")
    if point.data_type == "mac":
        return bytes(raw)

    formats = {"u8": ">B", "u16": ">H", "s16": ">h", "u32": ">I", "s32": ">i"}
    raw_value = unpack(formats[point.data_type], raw)[0]
    value = raw_value * point.scale
    return int(value) if point.scale == 1 else value


def format_point_value(point: RegisterPoint, value: int | float | str | bytes) -> str:
    """以稳定、可读的格式显示工程量。"""
    if isinstance(value, str):
        return value
    if isinstance(value, bytes):
        return ":".join(f"{byte:02X}" for byte in value)
    if point.scale == 1:
        text = f"{value:,}"
    elif point.scale == 0.1:
        text = f"{value:,.1f}"
    elif point.scale == 0.01:
        text = f"{value:,.2f}"
    else:
        text = f"{value:,.4f}".rstrip("0").rstrip(".")
    return f"{text} {point.unit}".rstrip()


def iter_points(function_code: int | None = None) -> Iterable[RegisterPoint]:
    """按功能码、文档地址与字节偏移顺序枚举业务点。"""
    return iter(
        sorted(
            (
                point
                for point in POINTS
                if function_code is None or point.function_code == function_code
            ),
            key=lambda point: (point.function_code, point.address, point.byte_offset),
        )
    )
