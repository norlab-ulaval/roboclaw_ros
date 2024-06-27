from diagnostic_msgs.msg import DiagnosticStatus
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Sequence, Tuple

ROBOCLAW_ERRORS = {
    0x000000: (DiagnosticStatus.OK, "Normal"),
    0x000001: (DiagnosticStatus.OK, "E-Stop"),
    0x000002: (DiagnosticStatus.ERROR, "Temperature Error"),
    0x000004: (DiagnosticStatus.ERROR, "Temperature 2 Error"),
    0x000008: (DiagnosticStatus.ERROR, "Main Voltage High Error"),
    0x000010: (DiagnosticStatus.ERROR, "Logic Voltage High Error"),
    0x000020: (DiagnosticStatus.ERROR, "Logic Voltage Low Error"),
    0x000040: (DiagnosticStatus.ERROR, "M1 Driver Fault Error"),
    0x000080: (DiagnosticStatus.ERROR, "M2 Driver Fault Error"),
    0x000100: (DiagnosticStatus.ERROR, "M1 Speed Error"),
    0x000200: (DiagnosticStatus.ERROR, "M2 Speed Error"),
    0x000400: (DiagnosticStatus.ERROR, "M1 Position Error"),
    0x000800: (DiagnosticStatus.ERROR, "M2 Position Error"),
    0x001000: (DiagnosticStatus.ERROR, "M1 Current Error"),
    0x002000: (DiagnosticStatus.ERROR, "M2 Current Error"),
    0x010000: (DiagnosticStatus.WARN, "M1 Over Current Warning"),
    0x020000: (DiagnosticStatus.WARN, "M2 Over Current Warning"),
    0x040000: (DiagnosticStatus.WARN, "Main Voltage High Warning"),
    0x080000: (DiagnosticStatus.WARN, "Main Voltage Low Warning"),
    0x100000: (DiagnosticStatus.WARN, "Temperature 1 Warning"),
    0x200000: (DiagnosticStatus.WARN, "Temperature 2 Warning"),
    0x400000: (DiagnosticStatus.WARN, "S4 Signal Triggered"),
    0x800000: (DiagnosticStatus.WARN, "S5 Signal Triggered"),
    0x01000000: (DiagnosticStatus.WARN, "Speed Error Limit Warning"),
    0x02000000: (DiagnosticStatus.WARN, "Position Error Limit Warning"),
}


def decipher_rclaw_status(status: int) -> list[tuple[bytes, str]]:
    """Dechipher RoboClaw unit status

    Args:
        status (int): RoboClaw unit status that is returned by the 90 - Read Status method

    Status Bit Mask : Function
        0x000000 : Normal
        0x000001 : E-Stop
        0x000002 : Temperature Error
        0x000004 : Temperature 2 Error
        0x000008 : Main Voltage High Error
        0x000010 : Logic Voltage High Error
        0x000020 : Logic Voltage Low Error
        0x000040 : M1 Driver Fault Error
        0x000080 : M2 Driver Fault Error
        0x000100 : M1 Speed Error
        0x000200 : M2 Speed Error
        0x000400 : M1 Position Error
        0x000800 : M2 Position Error
        0x001000 : M1 Current Error
        0x002000 : M2 Current Error
        0x010000 : M1 Over Current Warning
        0x020000 : M2 Over Current Warning
        0x040000 : Main Voltage High Warning
        0x080000 : Main Voltage Low Warning
        0x100000 : Temperature Warning
        0x200000 : Temperature 2 Warning
        0x400000 : S4 Signal Triggered
        0x800000 : S5 Signal Triggered
        0x01000000 : Speed Error Limit Warning
        0x02000000 : Position Error Limit Warning

    Returns:
        list[tuple[bytes, str]]: List of RoboClaw error statuses
    """
    status_bits = tuple(key for key in ROBOCLAW_ERRORS.keys() if (status & key) > 0)
    status_bits = status_bits if len(status_bits) > 0 else (0,)
    statuses = [ROBOCLAW_ERRORS[sbit] for sbit in status_bits]
    return statuses
