# KSTAR KSG1-250K Modbus RTU 只读仿真器

本项目基于项目内的 [KSG1－250K逆变器MODBUS通讯协议V3.6标准_对外版本20231010.pdf](KSG1－250K逆变器MODBUS通讯协议V3.6标准_对外版本20231010.pdf)，使用 PyModbus 3.3 模拟一台 **KSTAR G110KT / KSG1-250K 逆变器**。

- **从机**：在当前机器的 RS-485 `COM3` 上监听 Modbus RTU 请求。
- **主机**：在另一台机器的本地 COM 口上，对仿真从机发送协议规定的 `03H` 与 `04H` 读取请求。
- **安全边界**：本项目仅实现读取；不发送 `06H`、`10H` 或其他写入请求，因此不会写入真实逆变器参数。

协议串口参数为 **9600 baud、8 数据位、无校验、1 停止位（9600 8N1）**，仿真从站地址为 **1**，多寄存器数值采用协议规定的 **Big-Endian（高字在前）**。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| [config.py](config.py) | COM 端口、串口格式、从站地址和 Modbus 请求上限。 |
| [ksg1_registers.py](ksg1_registers.py) | KSG1 点表、模拟工程值、U8/U16/S16/U32/S32/ASCII 编解码和读取块定义。 |
| [modbus_slave.py](modbus_slave.py) | KSG1 从机仿真服务端；收到请求、发送响应时打印十六进制帧日志。 |
| [modbus_master.py](modbus_master.py) | 按协议分块发送 `03H` / `04H` 请求并解析、打印工程量。 |
| [requirements.txt](requirements.txt) | 与当前环境匹配的 Python 依赖。 |

## 已模拟的文档读取块

主机只读取协议中明确列出的有效块，**不会**扫描保留、未定义或型号不适用的地址。

| 功能码 | 文档 / PDU 地址 | 内容 | 请求方式 |
| --- | ---: | --- | --- |
| `03H` | `3200–3232` | 逆变器型号、DSP/ARM 版本、序列号、PLC MAC、软件标识、生产序号 | Holding Registers |
| `04H` | `2964–3153` | PV12–PV1 电压/电流/功率、直流母线、三相电网电压/频率/电流、功率、故障码、风机、能量、运行计数等 | Input Registers |
| `04H` | `3194–3319` | PLC、保护、限功率、PQ、DRM、配置镜像等明确点位 | Input Registers |
| `04H` | `5000–7399` | PV1–PV12，每路 100 个组串的电压和电流，共 1200 路组串 | Input Registers |

单次 Modbus 读取不得超过 125 个 16-bit register。主机自动把大块拆成合法请求；例如 `5000–7399` 会拆为 20 个 `04H` 请求。

## 代表性模拟点

下表是模拟点表中的代表性值。完整点表和所有模拟数据可见 [ksg1_registers.py](ksg1_registers.py)。

| 功能码 | 地址 | 点位 | 类型 / 比例 | 模拟工程值 |
| --- | ---: | --- | --- | ---: |
| `03H` | 3200–3204 | 逆变器型号 | ASCII，10 字节 | `G110KT` |
| `03H` | 3206–3216 | 设备序列号 | ASCII，22 字节 | `G110KT-SIM-20231010-01` |
| `03H` | 3217–3219 | PLC MAC | 6 字节 | `02:4B:53:47:31:01` |
| `04H` | 3000 | PV1 电压 | U16，0.1 V | 1071.7 V |
| `04H` | 3003 | PV1 电流 | S16，0.01 A | 8.21 A |
| `04H` | 3006–3007 | PV1 功率 | S32，1 W | 8,799 W |
| `04H` | 3012 | 正母线电压 PBUS | U16，0.1 V | 610.2 V |
| `04H` | 3014–3016 | RS / ST / TR 线电压 | U16，0.1 V | 800.4 / 798.9 / 801.1 V |
| `04H` | 3017–3019 | RS / ST / TR 频率 | U16，0.01 Hz | 50.00 / 50.01 / 49.99 Hz |
| `04H` | 3020–3022 | R / S / T 相电流 | S16，0.01 A | 138.42 / 137.96 / 138.17 A |
| `04H` | 3023–3024 | 交流有功功率 | S32，1 W | 96,500 W |
| `04H` | 3028–3029 | DSP 故障字 | U32 | 0（无故障） |
| `04H` | 3038–3039 | 当日发电量 | U32，0.1 kWh | 638.4 kWh |
| `04H` | 3040–3041 | 累计发电量 | U32，1 kWh | 1,284,760 kWh |
| `04H` | 5000 | PV1 组串 1 电压 | U16，0.1 V | 1061.7 V |
| `04H` | 5001 | PV1 组串 1 电流 | S16，0.01 A | 8.10 A |

## 安装

在两台机器上分别进入项目目录并安装依赖：

```powershell
python -m pip install -r requirements.txt
```

本项目使用并测试过 `pymodbus==3.3.0` 和 `pyserial`。若启动串口服务时报错 `NameError: name 'serial' is not defined`，说明当前 Python / Conda 环境缺少 `pyserial`，执行上面的安装命令即可。

## 启动与测试

1. 通过两个 RS-485 转换器将两台机器连接到同一条 RS-485 总线。
2. 在**从机机器**启动仿真器：

   ```powershell
   python .\modbus_slave.py
   ```

   正常启动会打印暴露的 `03H` 与 `04H` 地址块。

3. 在**主机机器**启动读取程序：

   ```powershell
   python .\modbus_master.py
   ```

   主机会依次请求全部模拟块，然后输出各个定义点的地址、名称、已经换算过比例的工程值及单位。由于包含 1200 路 PV 组串，完整输出会较长，这是预期行为。

4. 从机默认就会打印每次收发帧的十六进制日志，例如：

   ```text
   收到数据（8 字节）：01 03 0C 80 00 21 87 6A
   发送响应（9 字节）：01 03 04 00 47 00 31 8B F2
   ```

   `收到数据` 是串口一次读到的数据块，可能包含不完整帧或多个帧；`发送响应` 是从机发出的完整 RTU 回包。若还需查看 PyModbus 内部细节，可在从机机器用 DEBUG 级别运行：

   ```powershell
   python -c "import logging; logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s %(name)s: %(message)s'); import modbus_slave; modbus_slave.main()"
   ```

   此时日志还会显示 `Handling data: ...` 和 `send: ...` 等内部信息。

## 地址、功能码与字节序

### 文档地址就是请求地址

协议文件的 `Register Address` 是 Modbus PDU 地址。例如协议示例中读取 `04H` 地址 `3000` 的请求帧地址字段就是 `0x0BB8`（十进制 3000）。所以主机会直接调用：

```python
client.read_input_registers(address=3000, count=..., slave=1)
```

而不是把地址转成 `4xxxx` 或额外加减 1。

PyModbus 3.3 的服务端数据块内部存在默认的 `+1` 访问偏移。本项目仅在 [ksg1_registers.py](ksg1_registers.py) 构建数据块时处理它；主机和协议点表始终使用文档原始地址。

### 字段编码

- `U8` / ASCII：每个 Modbus register 包含两个字节，高字节在前。例如 `G110KT` 占用地址 `3200–3202` 中的前 6 字节，剩余字节以空格填充。
- `U16`、`S16`：占用一个 16-bit register；`S16` 按二进制补码处理。
- `U32`、`S32`：占用两个连续 registers，按高字在前（Big-Endian）编码和解析。
- 带单位的比例值（如 `0.1 V`、`0.01 A`、`0.01 Hz`）在从机中编码为原始整数，在主机输出时自动换算为工程值。

## 修改 COM 端口

默认配置位于 [config.py](config.py)：

```python
SLAVE_PORT = "COM3"   # 从机机器上的本地 RS-485 端口
MASTER_PORT = "COM4"  # 主机机器上的本地 RS-485 端口
```

两台机器的本地端口号可以相同或不同；必须以各自 Windows 设备管理器显示的实际端口为准。不要让两个 Python 程序占用同一台机器上的同一个物理 COM 口。

## RS-485 排查

- 两端通常连接 `A → A`、`B → B`。若设备厂商的 `A/B` 或 `D+/D-` 标记定义相反，可只在一端对调 A/B 后复测。
- 设备要求时连接两端的信号地（GND）。
- 长距离线路只在总线两端配置 120 Ω 终端电阻。
- 主机报 `No response received` 时，先确认从机 DEBUG 日志是否有 `Handling data`：有接收但没有 `send` 多为协议帧/服务端配置问题；两者都有而主机仍超时则优先检查 RS-485 回传线路、A/B 极性和自动收发方向。
- 对未映射、保留或未明确的地址读取时，从机会返回 Modbus `Illegal Address`，这是该仿真器的预期保护行为。
