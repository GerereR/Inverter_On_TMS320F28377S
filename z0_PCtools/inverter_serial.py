#!/usr/bin/env python3
"""InverterBase SCI 上位机工具。

串口参数与 MCU 当前配置一致：9600 波特率、8 数据位、无校验、1 停止位。

使用示例：
    py -m pip install pyserial
    py pc_tools/inverter_serial.py list-ports
    py pc_tools/inverter_serial.py --port COM3
    py pc_tools/inverter_serial.py --port COM3 real
    py pc_tools/inverter_serial.py --port COM3 rms
    py pc_tools/inverter_serial.py --port COM3 set-current 0.5
"""

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

SCI_CMD_READ_REAL_MEASUREMENTS = 0x02
SCI_CMD_READ_PLL_STATUS = 0x03
SCI_CMD_READ_FAULT_STATUS = 0x04
SCI_CMD_READ_RMS_MEASUREMENTS = 0x05
SCI_CMD_SET_INDUCTOR_CURRENT_AMP = 0x10
SCI_CMD_CLEAR_FAULT = 0x20
SCI_CMD_READ_VERSION = 0x30

SCI_STATUS_OK = 0x00
SCI_STATUS_BAD_LENGTH = 0x01
SCI_STATUS_BAD_COMMAND = 0x02
SCI_STATUS_BAD_PARAMETER = 0x03

STATUS_TEXT = {
    SCI_STATUS_OK: "成功",
    SCI_STATUS_BAD_LENGTH: "数据长度错误",
    SCI_STATUS_BAD_COMMAND: "未知命令",
    SCI_STATUS_BAD_PARAMETER: "参数错误",
}


class ProtocolError(RuntimeError):
    """表示接收帧格式错误、CRC错误或MCU拒绝命令。"""


def crc16_modbus(data: bytes) -> int:
    """使用与 MCU 相同的 CRC-16/MODBUS 算法计算校验值。"""
    crc = 0xFFFF

    for data_byte in data:
        crc ^= data_byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def build_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    """按照 MCU 协议组装请求帧，CRC采用小端顺序。"""
    if not 0 <= command <= 0xFF:
        raise ValueError("命令号必须在0～255之间")
    if not 0 <= sequence <= 0xFF:
        raise ValueError("序号必须在0～255之间")
    if len(payload) > SCI_FRAME_MAX_PAYLOAD:
        raise ValueError(f"Payload超过{SCI_FRAME_MAX_PAYLOAD}字节")

    frame_body = bytes((command, sequence)) + struct.pack("<H", len(payload)) + payload
    return SCI_FRAME_SOF + frame_body + struct.pack("<H", crc16_modbus(frame_body))


def get_pyserial():
    """延迟导入pyserial，使协议自检无需安装串口依赖。"""
    try:
        import serial
        import serial.tools.list_ports
    except ImportError as error:
        raise RuntimeError(
            "缺少pyserial，请先执行：py -m pip install pyserial"
        ) from error

    return serial


class InverterSerialClient:
    """负责串口收发、帧校验和命令响应匹配。"""

    def __init__(self, port: str, timeout_seconds: float) -> None:
        serial = get_pyserial()
        self._serial = serial.Serial(
            port=port,
            baudrate=SCI_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
        )
        self._timeout_seconds = timeout_seconds
        self._sequence = 0

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "InverterSerialClient":
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.close()

    def _next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        return sequence

    def _read_one_byte(self, deadline: float) -> Optional[int]:
        while time.monotonic() < deadline:
            received = self._serial.read(1)
            if received:
                return received[0]
        return None

    def _read_exact(self, byte_count: int, deadline: float) -> bytes:
        received = bytearray()

        while len(received) < byte_count and time.monotonic() < deadline:
            block = self._serial.read(byte_count - len(received))
            if block:
                received.extend(block)

        if len(received) != byte_count:
            raise TimeoutError("等待MCU响应超时")

        return bytes(received)

    def _read_frame(self, deadline: float) -> tuple[int, int, bytes]:
        """寻找帧头并读取一帧完整响应，忽略帧头之前的杂散字节。"""
        previous_byte: Optional[int] = None

        while time.monotonic() < deadline:
            current_byte = self._read_one_byte(deadline)
            if current_byte is None:
                break
            if previous_byte == SCI_FRAME_SOF[0] and current_byte == SCI_FRAME_SOF[1]:
                header = self._read_exact(4, deadline)
                command = header[0]
                sequence = header[1]
                payload_length = struct.unpack_from("<H", header, 2)[0]

                if payload_length > SCI_FRAME_MAX_PAYLOAD:
                    previous_byte = None
                    continue

                payload_and_crc = self._read_exact(payload_length + 2, deadline)
                payload = payload_and_crc[:payload_length]
                received_crc = struct.unpack_from("<H", payload_and_crc, payload_length)[0]
                frame_body = header + payload
                calculated_crc = crc16_modbus(frame_body)

                if received_crc != calculated_crc:
                    raise ProtocolError(
                        f"CRC错误：收到0x{received_crc:04X}，计算得到0x{calculated_crc:04X}"
                    )

                return command, sequence, payload

            previous_byte = current_byte

        raise TimeoutError("等待MCU响应超时")

    def request(self, command: int, payload: bytes = b"") -> bytes:
        """发送请求并等待命令号、序号均匹配的响应。"""
        sequence = self._next_sequence()
        request_frame = build_frame(command, sequence, payload)
        expected_response_command = command | 0x80
        deadline = time.monotonic() + self._timeout_seconds

        # 丢弃上次操作遗留的响应，确保本次序号匹配简单明确。
        self._serial.reset_input_buffer()
        self._serial.write(request_frame)
        self._serial.flush()

        while time.monotonic() < deadline:
            response_command, response_sequence, response_payload = self._read_frame(deadline)
            if (response_command == expected_response_command and
                    response_sequence == sequence):
                self._check_status(response_payload)
                return response_payload

        raise TimeoutError("没有收到与请求匹配的MCU响应")

    @staticmethod
    def _check_status(payload: bytes) -> None:
        if not payload:
            raise ProtocolError("响应Payload中缺少状态字节")

        status = payload[0]
        if status != SCI_STATUS_OK:
            status_text = STATUS_TEXT.get(status, "未知状态")
            raise ProtocolError(f"MCU返回错误：{status_text}（0x{status:02X}）")

    def read_real_measurements(self) -> dict[str, float | int]:
        payload = self.request(SCI_CMD_READ_REAL_MEASUREMENTS)
        if len(payload) != 63:
            raise ProtocolError(f"real测量响应长度应为63字节，实际为{len(payload)}字节")

        values = struct.unpack_from("<14fHHH", payload, 1)
        names = (
            "grid_voltage",
            "inductor_current",
            "gfci_current",
            "dc_bus_voltage",
            "grid_dc_current",
            "inverter_voltage",
            "pv1_current",
            "pv2_current",
            "pv1_voltage",
            "pv2_voltage",
            "pv1_isolation_voltage",
            "pv2_isolation_voltage",
            "inverter_temperature",
            "boost_temperature",
        )
        result = dict(zip(names, values[:14]))
        result["ecap_frequency_hz"] = values[14] / 100.0
        result["pll_frequency_hz"] = values[15] / 100.0
        result["inductor_current_amp"] = values[16] / 4096.0
        return result

    def read_rms_measurements(self) -> dict[str, float]:
        payload = self.request(SCI_CMD_READ_RMS_MEASUREMENTS)
        if len(payload) != 49:
            raise ProtocolError(f"RMS测量响应长度应为49字节，实际为{len(payload)}字节")

        values = struct.unpack_from("<12f", payload, 1)
        names = (
            "grid_voltage",
            "inductor_current",
            "gfci_current",
            "dc_bus_voltage",
            "grid_dc_current",
            "inverter_voltage",
            "pv1_current",
            "pv2_current",
            "pv1_voltage",
            "pv2_voltage",
            "pv1_isolation_voltage",
            "pv2_isolation_voltage",
        )
        return dict(zip(names, values))

    def read_pll_status(self) -> dict[str, int | float | bool]:
        payload = self.request(SCI_CMD_READ_PLL_STATUS)
        if len(payload) != 6:
            raise ProtocolError(f"PLL响应长度应为6字节，实际为{len(payload)}字节")

        ecap_freq_cent, pll_freq_cent = struct.unpack_from("<HH", payload, 1)
        return {
            "ecap_frequency_hz": ecap_freq_cent / 100.0,
            "pll_frequency_hz": pll_freq_cent / 100.0,
            "pll_locked": bool(payload[5]),
        }

    def read_fault_status(self) -> bool:
        payload = self.request(SCI_CMD_READ_FAULT_STATUS)
        if len(payload) != 2:
            raise ProtocolError(f"故障响应长度应为2字节，实际为{len(payload)}字节")
        return bool(payload[1])

    def set_inductor_current_amplitude(self, amplitude: float) -> float:
        if not 0.0 <= amplitude <= 1.0:
            raise ValueError("电感电流幅值必须在0.0～1.0之间")

        amplitude_q12 = round(amplitude * 4096.0)
        payload = self.request(
            SCI_CMD_SET_INDUCTOR_CURRENT_AMP,
            struct.pack("<H", amplitude_q12),
        )
        if len(payload) != 3:
            raise ProtocolError(f"设置幅值响应长度应为3字节，实际为{len(payload)}字节")

        accepted_q12 = struct.unpack_from("<H", payload, 1)[0]
        return accepted_q12 / 4096.0

    def clear_fault(self) -> None:
        payload = self.request(SCI_CMD_CLEAR_FAULT)
        if len(payload) != 1:
            raise ProtocolError(f"清除故障响应长度应为1字节，实际为{len(payload)}字节")

    def read_version(self) -> tuple[int, int]:
        payload = self.request(SCI_CMD_READ_VERSION)
        if len(payload) != 3:
            raise ProtocolError(f"版本响应长度应为3字节，实际为{len(payload)}字节")
        return payload[1], payload[2]


def print_real_measurements(measurements: dict[str, float | int]) -> None:
    for name, value in measurements.items():
        if name.endswith("frequency_hz"):
            print(f"{name}: {value:.2f} Hz")
        elif name == "inductor_current_amp":
            print(f"{name}: {value:.4f}")
        else:
            print(f"{name}: {value:.4f}")


def print_rms_measurements(measurements: dict[str, float]) -> None:
    for name, value in measurements.items():
        print(f"{name}: {value:.4f}")


def print_pll_status(pll_status: dict[str, int | float | bool]) -> None:
    print(f"ECAP频率：{pll_status['ecap_frequency_hz']:.2f} Hz")
    print(f"PLL频率：{pll_status['pll_frequency_hz']:.2f} Hz")
    print(f"PLL锁定：{'是' if pll_status['pll_locked'] else '否'}")


def run_interactive(client: InverterSerialClient) -> None:
    """提供无需额外串口助手的简单交互菜单。"""
    menu = (
        "\n1 读取实际平均值\n"
        "2 读取RMS值\n"
        "3 读取PLL状态\n"
        "4 读取故障状态\n"
        "5 设置电感电流幅值\n"
        "6 清除Trip-Zone故障\n"
        "7 读取协议版本\n"
        "q 退出\n"
    )

    while True:
        print(menu)
        selection = input("请选择：").strip().lower()
        try:
            if selection == "1":
                print_real_measurements(client.read_real_measurements())
            elif selection == "2":
                print_rms_measurements(client.read_rms_measurements())
            elif selection == "3":
                print_pll_status(client.read_pll_status())
            elif selection == "4":
                print(f"Trip-Zone故障：{'是' if client.read_fault_status() else '否'}")
            elif selection == "5":
                amplitude = float(input("输入0.0～1.0的归一化幅值："))
                accepted = client.set_inductor_current_amplitude(amplitude)
                print(f"MCU已接受幅值：{accepted:.4f}")
            elif selection == "6":
                client.clear_fault()
                print("清除故障命令已执行")
            elif selection == "7":
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
        return

    for port in ports:
        print(f"{port.device}: {port.description}")


def run_self_test() -> None:
    """离线检查CRC算法、请求帧和响应字段解析。"""
    if crc16_modbus(b"123456789") != 0x4B37:
        raise AssertionError("CRC-16/MODBUS标准测试失败")

    frame = build_frame(SCI_CMD_READ_VERSION, 0x2A)
    frame_body = frame[2:-2]
    frame_crc = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    if frame[:2] != SCI_FRAME_SOF or frame_crc != crc16_modbus(frame_body):
        raise AssertionError("协议帧测试失败")

    print("协议自检通过")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="InverterBase SCI上位机工具")
    parser.add_argument("--port", help="串口名称，例如COM3")
    parser.add_argument(
        "--timeout",
        type=float,
        default=SCI_DEFAULT_TIMEOUT_SECONDS,
        help="响应超时秒数，默认2秒",
    )

    subparsers = parser.add_subparsers(dest="operation")
    subparsers.add_parser("interactive", help="进入交互菜单（默认）")
    subparsers.add_parser("list-ports", help="列出电脑上的串口")
    subparsers.add_parser("real", help="读取校准后的实际平均值")
    subparsers.add_parser("pll", help="读取PLL状态")
    subparsers.add_parser("fault", help="读取Trip-Zone故障状态")
    subparsers.add_parser("rms", help="读取校准后的RMS值")
    set_current_parser = subparsers.add_parser("set-current", help="设置电感电流幅值")
    set_current_parser.add_argument("amplitude", type=float, help="0.0～1.0归一化幅值")
    subparsers.add_parser("clear-fault", help="清除Trip-Zone故障")
    subparsers.add_parser("version", help="读取协议版本")
    subparsers.add_parser("self-test", help="不连接MCU，检查CRC和帧格式")
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
            elif operation == "real":
                print_real_measurements(client.read_real_measurements())
            elif operation == "pll":
                print_pll_status(client.read_pll_status())
            elif operation == "fault":
                print(f"Trip-Zone故障：{'是' if client.read_fault_status() else '否'}")
            elif operation == "rms":
                print_rms_measurements(client.read_rms_measurements())
            elif operation == "set-current":
                accepted = client.set_inductor_current_amplitude(arguments.amplitude)
                print(f"MCU已接受幅值：{accepted:.4f}")
            elif operation == "clear-fault":
                client.clear_fault()
                print("清除故障命令已执行")
            elif operation == "version":
                major, minor = client.read_version()
                print(f"协议版本：{major}.{minor}")
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
