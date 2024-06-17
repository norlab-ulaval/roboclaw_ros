from diagnostic_msgs.msg import DiagnosticStatus
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Sequence, Tuple

# Status Bit Mask : Function
#     0x000000 : Normal
#     0x000001 : E-Stop
#     0x000002 : Temperature Error
#     0x000004 : Temperature 2 Error
#     0x000008 : Main Voltage High Error
#     0x000010 : Logic Voltage High Error
#     0x000020 : Logic Voltage Low Error
#     0x000040 : M1 Driver Fault Error
#     0x000080 : M2 Driver Fault Error
#     0x000100 : M1 Speed Error
#     0x000200 : M2 Speed Error
#     0x000400 : M1 Position Error
#     0x000800 : M2 Position Error
#     0x001000 : M1 Current Error
#     0x002000 : M2 Current Error
#     0x010000 : M1 Over Current Warning
#     0x020000 : M2 Over Current Warning
#     0x040000 : Main Voltage High Warning
#     0x080000 : Main Voltage Low Warning
#     0x100000 : Temperature Warning
#     0x200000 : Temperature 2 Warning
#     0x400000 : S4 Signal Triggered
#     0x800000 : S5 Signal Triggered
#     0x01000000 : Speed Error Limit Warning
#     0x02000000 : Position Error Limit Warning

ROBOCLAW_ERRORS = {
    0x0000: (DiagnosticStatus.OK, "Normal"),
    0x0001: (DiagnosticStatus.WARN, "M1 over current"),
    0x0002: (DiagnosticStatus.WARN, "M2 over current"),
    0x0004: (DiagnosticStatus.ERROR, "Emergency Stop"),
    0x0008: (DiagnosticStatus.ERROR, "Temperature1"),
    0x0010: (DiagnosticStatus.ERROR, "Temperature2"),
    0x0020: (
        DiagnosticStatus.ERROR,
        "Main batt voltage high",
    ),
    0x0040: (
        DiagnosticStatus.ERROR,
        "Logic batt voltage high",
    ),
    0x0080: (
        DiagnosticStatus.ERROR,
        "Logic batt voltage low",
    ),
    0x0100: (DiagnosticStatus.WARN, "M1 driver fault"),
    0x0200: (DiagnosticStatus.WARN, "M2 driver fault"),
    0x0400: (
        DiagnosticStatus.WARN,
        "Main batt voltage high",
    ),
    0x0800: (
        DiagnosticStatus.WARN,
        "Main batt voltage low",
    ),
    0x1000: (DiagnosticStatus.WARN, "Temperature1"),
    0x2000: (DiagnosticStatus.WARN, "Temperature2"),
    0x4000: (DiagnosticStatus.OK, "M1 home"),
    0x8000: (DiagnosticStatus.OK, "M2 home"),
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
    statuses = [ROBOCLAW_ERRORS[sbit] for sbit in status_bits]
    return statuses
