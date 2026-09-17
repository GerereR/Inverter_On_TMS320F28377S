#!/usr/bin/env python3
"""InverterBase SCI只读上位机工具。"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Optional


SCI_BAUDRATE = 9600
SCI_FRAME_SOF = b"\xAA\x55"
SCI_FRAME_MAX_PAYLOAD = 64
SCI_DEFAULT_TIMEOUT_SECONDS = 2.0

SCI_CMD_READ_MACHINE_AVG = 0x02
SCI_CMD_READ_SYS_PROBLEM = 0x04
SCI_CMD_READ_MACHINE_RMS = 0x05
SCI_CMD_READ_MACHINE_POWER = 0x07
SCI_CMD_READ_SYSTEM_STATE = 0x08
SCI_CMD_READ_VERSION = 0x30

SCI_STATUS_OK = 0x00
SCI_STATUS_BAD_LENGTH = 0x01
SCI_STATUS_BAD_COMMAND = 0x02

STATUS_TEXT = {
    SCI_STATUS_OK: "成功",
    SCI_STATUS_BAD_LENGTH: "请求长度错误",
    SCI_STATUS_BAD_COMMAND: "未知或未开放的命令",
}

ADC_FIELD_NAMES = (
    "grid_voltage", "inductor_current", "gfci_current", "dc_bus_voltage",
    "grid_dc_current", "inverter_voltage", "pv1_current", "pv2_current",
    "pv1_voltage", "pv2_voltage", "pv1_isolation_voltage",
    "pv2_isolation_voltage", "inverter_temperature", "boost_temperature",
)


class ProtocolError(RuntimeError):
    """响应帧、CRC或MCU状态错误。"""


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for data_byte in data:
        crc ^= data_byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF or not 0 <= sequence <= 0xFF:
        raise ValueError("命令号和序号必须在0到255之间")
    if len(payload) > SCI_FRAME_MAX_PAYLOAD:
        raise ValueError("Payload超过64字节")
    body = bytes((command, sequence)) + struct.pack("<H", len(payload)) + payload
    return SCI_FRAME_SOF + body + struct.pack("<H", crc16_modbus(body))


def get_pyserial():
    try:
        import serial
        import serial.tools.list_ports
    except ImportError as error:
        raise RuntimeError("缺少pyserial，请执行：py -m pip install pyserial") from error
    return serial


class InverterSerialClient:
    def __init__(self, port: str, timeout_seconds: float) -> None:
        serial = get_pyserial()
        self._serial = serial.Serial(
            port=port, baudrate=SCI_BAUDRATE, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=0.05, write_timeout=1.0,
        )
        self._timeout_seconds = timeout_seconds
        self._sequence = 0

    def __enter__(self) -> "InverterSerialClient":
        return self

    def __exit__(self, *_args) -> None:
        self._serial.close()

    def _read_exact(self, count: int, deadline: float) -> bytes:
        data = bytearray()
        while len(data) < count and time.monotonic() < deadline:
            data.extend(self._serial.read(count - len(data)))
        if len(data) != count:
            raise TimeoutError("等待MCU响应超时")
        return bytes(data)

    def _read_frame(self, deadline: float) -> tuple[int, int, bytes]:
        previous: Optional[int] = None
        while time.monotonic() < deadline:
            current_data = self._serial.read(1)
            if not current_data:
                continue
            current = current_data[0]
            if previous == SCI_FRAME_SOF[0] and current == SCI_FRAME_SOF[1]:
                header = self._read_exact(4, deadline)
                command, sequence = header[0], header[1]
                payload_length = struct.unpack_from("<H", header, 2)[0]
                if payload_length > SCI_FRAME_MAX_PAYLOAD:
                    raise ProtocolError("响应Payload超过64字节")
                tail = self._read_exact(payload_length + 2, deadline)
                payload = tail[:payload_length]
                received_crc = struct.unpack_from("<H", tail, payload_length)[0]
                calculated_crc = crc16_modbus(header + payload)
                if received_crc != calculated_crc:
                    raise ProtocolError(
                        f"CRC错误：收到0x{received_crc:04X}，计算值0x{calculated_crc:04X}"
                    )
                return command, sequence, payload
            previous = current
        raise TimeoutError("等待MCU响应超时")

    def request(self, command: int) -> bytes:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        deadline = time.monotonic() + self._timeout_seconds
        self._serial.reset_input_buffer()
        self._serial.write(build_frame(command, sequence))
        self._serial.flush()

        while time.monotonic() < deadline:
            response_command, response_sequence, payload = self._read_frame(deadline)
            if response_command != (command | 0x80) or response_sequence != sequence:
                continue
            if not payload:
                raise ProtocolError("响应中没有状态字节")
            if payload[0] != SCI_STATUS_OK:
                text = STATUS_TEXT.get(payload[0], "未知状态")
                raise ProtocolError(f"MCU返回错误：{text}，0x{payload[0]:02X}")
            return payload
        raise TimeoutError("没有收到匹配的MCU响应")

    def _read_adc_values(self, command: int) -> dict[str, float]:
        payload = self.request(command)
        if len(payload) != 57:
            raise ProtocolError(f"ADC数据响应应为57字节，实际为{len(payload)}字节")
        return dict(zip(ADC_FIELD_NAMES, struct.unpack_from("<14f", payload, 1)))

    def read_machine_average(self) -> dict[str, float]:
        return self._read_adc_values(SCI_CMD_READ_MACHINE_AVG)

    def read_machine_rms(self) -> dict[str, float]:
        return self._read_adc_values(SCI_CMD_READ_MACHINE_RMS)

    def read_machine_power(self) -> dict[str, float | int]:
        payload = self.request(SCI_CMD_READ_MACHINE_POWER)
        if len(payload) != 33:
            raise ProtocolError(f"功率数据响应应为33字节，实际为{len(payload)}字节")
        values = struct.unpack_from("<6fHHI", payload, 1)
        return {
            "grid_active_power": values[0],
            "grid_reactive_power": values[1],
            "grid_apparent_power": values[2],
            "grid_pf": values[3],
            "pv1_power": values[4],
            "pv2_power": values[5],
            "ecap_frequency_hz": values[6] / 100.0,
            "pll_frequency_hz": values[7] / 100.0,
            "measure_sequence": values[8],
        }

    def read_sys_problem(self) -> dict[str, int]:
        payload = self.request(SCI_CMD_READ_SYS_PROBLEM)
        if len(payload) != 13:
            raise ProtocolError(f"SysProblem响应应为13字节，实际为{len(payload)}字节")
        warning, recoverable, permanent = struct.unpack_from("<III", payload, 1)
        return {
            "warning": warning,
            "recoverable_faults": recoverable,
            "permanent_faults": permanent,
        }

    def read_system_state(self) -> dict[str, int]:
        payload = self.request(SCI_CMD_READ_SYSTEM_STATE)
        if len(payload) != 17:
            raise ProtocolError(f"系统状态响应应为17字节，实际为{len(payload)}字节")
        names = (
            "state", "check_stage", "start_request", "source_ready",
            "grid_ready", "bus_ready", "reload_flag", "reload_count",
        )
        return dict(zip(names, struct.unpack_from("<8H", payload, 1)))

    def read_version(self) -> tuple[int, int]:
        payload = self.request(SCI_CMD_READ_VERSION)
        if len(payload) != 3:
            raise ProtocolError(f"版本响应应为3字节，实际为{len(payload)}字节")
        return payload[1], payload[2]


def print_mapping(values: dict[str, float | int]) -> None:
    for name, value in values.items():
        if isinstance(value, float):
            print(f"{name}: {value:.4f}")
        elif name in {"warning", "recoverable_faults", "permanent_faults"}:
            print(f"{name}: 0x{value:08X}")
        else:
            print(f"{name}: {value}")


def print_machine(client: InverterSerialClient) -> None:
    for title, values in (
        ("average", client.read_machine_average()),
        ("rms", client.read_machine_rms()),
        ("power", client.read_machine_power()),
    ):
        print(f"[{title}]")
        print_mapping(values)


def run_interactive(client: InverterSerialClient) -> None:
    menu = (
        "1 读取MachineData\n"
        "2 读取SysProblem\n"
        "3 读取状态机和检查子状态\n"
        "4 读取协议版本\n"
        "q 退出\n"
    )
    while True:
        print(menu)
        selection = input("请选择：").strip().lower()
        try:
            if selection == "1":
                print_machine(client)
            elif selection == "2":
                print_mapping(client.read_sys_problem())
            elif selection == "3":
                print_mapping(client.read_system_state())
            elif selection == "4":
                major, minor = client.read_version()
                print(f"协议版本：{major}.{minor}")
            elif selection in {"q", "quit", "exit"}:
                return
            else:
                print("无法识别该选项")
        except (ProtocolError, TimeoutError, ValueError) as error:
            print(f"操作失败：{error}")


def list_ports() -> None:
    serial = get_pyserial()
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("没有发现串口")
    for port in ports:
        print(f"{port.device}: {port.description}")


def run_self_test() -> None:
    if crc16_modbus(b"123456789") != 0x4B37:
        raise AssertionError("CRC-16/MODBUS标准测试失败")
    frame = build_frame(SCI_CMD_READ_VERSION, 0x2A)
    if frame[:2] != SCI_FRAME_SOF:
        raise AssertionError("帧头测试失败")
    if struct.unpack_from("<H", frame, len(frame) - 2)[0] != crc16_modbus(frame[2:-2]):
        raise AssertionError("请求帧CRC测试失败")
    if struct.calcsize("<14f") != 56 or struct.calcsize("<6fHHI") != 32:
        raise AssertionError("响应结构尺寸测试失败")
    print("协议自检通过")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="InverterBase SCI只读上位机工具")
    parser.add_argument("--port", help="串口名称，例如COM3")
    parser.add_argument("--timeout", type=float, default=SCI_DEFAULT_TIMEOUT_SECONDS,
                        help="响应超时秒数，默认2秒")
    subparsers = parser.add_subparsers(dest="operation")
    subparsers.add_parser("interactive", help="进入只读交互菜单（默认）")
    subparsers.add_parser("list-ports", help="列出电脑上的串口")
    subparsers.add_parser("machine", help="读取完整MachineData")
    subparsers.add_parser("problem", help="读取SysProblem")
    subparsers.add_parser("state", help="读取状态机和检查子状态")
    subparsers.add_parser("version", help="读取协议版本")
    subparsers.add_parser("self-test", help="离线检查CRC和帧格式")
    return parser


def main() -> int:
    parser = build_argument_parser()
    arguments = parser.parse_args()
    operation = arguments.operation or "interactive"

    if operation == "list-ports":
        list_ports()
        return 0
    if operation == "self-test":
        run_self_test()
        return 0
    if not arguments.port:
        parser.error("该操作需要使用--port指定串口，例如：--port COM3")

    try:
        with InverterSerialClient(arguments.port, arguments.timeout) as client:
            if operation == "interactive":
                run_interactive(client)
            elif operation == "machine":
                print_machine(client)
            elif operation == "problem":
                print_mapping(client.read_sys_problem())
            elif operation == "state":
                print_mapping(client.read_system_state())
            elif operation == "version":
                major, minor = client.read_version()
                print(f"协议版本：{major}.{minor}")
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
