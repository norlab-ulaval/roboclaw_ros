from __future__ import annotations

import random
import time
from typing import Literal, Sequence

import serial

# Command Enums
class CMD:
    """Command number Enum

    Refer to the user manual to get the list of all commands
    https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf
    """

    M1FORWARD = 0
    M1BACKWARD = 1
    SETMINMB = 2
    SETMAXMB = 3
    M2FORWARD = 4
    M2BACKWARD = 5
    M17BIT = 6
    M27BIT = 7
    MIXEDFORWARD = 8
    MIXEDBACKWARD = 9
    MIXEDRIGHT = 10
    MIXEDLEFT = 11
    MIXEDFB = 12
    MIXEDLR = 13
    SETSERIALTIMEOUT = 14
    GETSERIALTIMEOUT = 15
    GETM1ENC = 16
    GETM2ENC = 17
    GETM1SPEED = 18
    GETM2SPEED = 19
    RESETENC = 20
    GETFWVERSION = 21
    SETM1ENCCOUNT = 22
    SETM2ENCCOUNT = 23
    GETMBATTVOLT = 24
    GETLBATTVOLT = 25
    SETMINLB = 26
    SETMAXLB = 27
    SETM1PID = 28
    SETM2PID = 29
    GETM1ISPEED = 30
    GETM2ISPEED = 31
    M1DUTY = 32
    M2DUTY = 33
    MIXEDDUTY = 34
    M1SPEED = 35
    M2SPEED = 36
    MIXEDSPEED = 37
    M1SPEEDACCEL = 38
    M2SPEEDACCEL = 39
    MIXEDSPEEDACCEL = 40
    BUF_M1SPEEDDIST = 41
    BUF_M2SPEEDDIST = 42
    BUF_MIXEDSPEEDDIST = 43
    BUF_M1SPEEDACCELDIST = 44
    BUF_M2SPEEDACCELDIST = 45
    BUF_MIXEDSPEEDACCELDIST = 46
    GETBUFFERS = 47
    GETPWMS = 48
    GETCURRENTS = 49
    MIXEDSPEED2ACCEL = 50
    BUF_MIXEDSPEED2ACCELDIST = 51
    M1DUTYACCEL = 52
    M2DUTYACCEL = 53
    MIXEDDUTYACCEL = 54
    READM1PID = 55
    READM2PID = 56
    SETMAINVOLTAGES = 57
    SETLOGICVOLTAGES = 58
    GETMINMAXMAINVOLTAGES = 59
    GETMINMAXLOGICVOLTAGES = 60
    SETM1POSPID = 61
    SETM2POSPID = 62
    READM1POSPID = 63
    READM2POSPID = 64
    BUF_M1SPEEDACCELDECCELPOS = 65
    BUF_M2SPEEDACCELDECCELPOS = 66
    BUF_MIXEDSPEEDACCELDECCELPOS = 67
    SETM1DEFAULTACCEL = 68
    SETM2DEFAULTACCEL = 69
    SETM1DEFAULTSPEED = 70
    SETM2DEFAULTSPEED = 71
    GETDEFAULTSPEEDS = 72
    SETPINFUNCTIONS = 74
    GETPINFUNCTIONS = 75
    SETDEADBAND = 76
    GETDEADBAND = 77
    GETENCCOUNTERS = 78
    GETISPEEDCOUNTERS = 79
    RESTOREDEFAULTS = 80
    GETDEFAULTDUTYACCEL = 81
    GETTEMP = 82
    GETTEMP2 = 83
    GETERROR = 90
    GETENCODERMODE = 91
    SETM1ENCODERMODE = 92
    SETM2ENCODERMODE = 93
    WRITENVM = 94
    READNVM = 95
    SETCONFIG = 98
    GETCONFIG = 99
    SETCTRLMODES = 100
    READCTRLMODES = 101
    SETCTRL1 = 102
    SETCTRL2 = 103
    READCTRLSETTINGS = 104
    SETM1AUTOHOMING = 105
    SETM2AUTOHOMING = 106
    GETAUTOHOMING = 107
    GETMOTORAVESPEEDS = 108
    SETSPEEDERRLIMITS = 109
    GETSPEEDERRLIMITS = 110
    GETSPEEDERRORS = 111
    SETPOSERRLIMITS = 112
    GETPOSERRLIMITS = 113
    GETPOSERRORS = 114
    SETBATTVOLTOFFSET = 115
    GETBATTVOLTOFFSET = 116
    SETCURRENTBLANKINGPERCENTAGE = 117
    GETCURRENTBLANKINGPERCENTAGE = 118
    BUF_POSM1 = 119
    BUF_POSM2 = 120
    BUF_POSM1M2 = 121
    BUF_SPEEDPOSM1 = 122
    BUF_SPEEDPOSM2 = 123
    BUF_SPEEDPOSM1M2 = 124
    SETM1MAXCURRENT = 133
    SETM2MAXCURRENT = 134
    GETM1MAXCURRENT = 135
    GETM2MAXCURRENT = 136
    SETPWMMODE = 148
    GETPWMMODE = 149
    READEEPROM = 252
    WRITEEEPROM = 253
    FLAGBOOTLOADER = 255


class RoboclawDriver:
    """RoboClaw Interface Class

    Raises:
        ConnectionError: Serial connection error

    Returns:
        RoboclawDriver: Instance of RoboclawDriver class
    """

    def __init__(
        self,
        comport,
        baudrate: int,
        address: int = 0x80,
        timeout: float = 0.01,
        retries: int = 3,
    ):
        self.comport = comport
        self.baudrate: int = baudrate
        self.address: int = address
        self.timeout: float = timeout
        self._tries_timeout: int = retries
        self._crc = 0
        self._port: serial.Serial | None = None
        self.open()

    def open(self) -> bool:
        """Open Serial port connection with the RoboClaw

            Raises:
            ConnectionError: Serial connection error

            Returns:
            bool: Status
        """
        try:
            self._port = serial.Serial(
                port=self.comport,
                baudrate=self.baudrate,
                timeout=1,
                interCharTimeout=self.timeout,
            )
        except Exception as exc:
            raise ConnectionError("Roboclaw not found on port") from exc
        return True

    # User accessible functions
    def SendRandomData(self, cnt: int) -> None:
        """Write random bytes on the port

        Args:
            cnt (int): Number of bytes to send on the port
        """
        for _ in range(cnt):
            byte = random.getrandbits(8)
            self._port.write(byte.to_bytes(1, "big"))

    def StopMotors(self) -> bool:
        """Stop both motors.

        Returns:
            bool: Command acknowledgement
        """
        return self.SpeedM1M2(0, 0)

    def ForwardM1(self, val: int) -> bool:
        """0 - Drive Forward M1

        Drive motor 1 forward.

        Args:
            val (int): M1 Speed.
            Valid data range is 0 - 127.
            A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.M1FORWARD, val)

    def BackwardM1(self, val: int) -> bool:
        """1 - Drive Backwards M1

        Drive motor 1 backwards.

        Args:
            val (int): M1 Speed.
            Valid data range is 0 - 127.
            A value of 127 = full speed backwards, 64 = about half speed backwards and 0 = full stop.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.M1BACKWARD, val)

    def SetMinVoltageMainBattery(self, val: int) -> bool:
        """2 - Set Minimum Main Voltage (Command 57 Preferred)

        Sets main battery (B- / B+) minimum voltage level.
        If the battery voltages drops below the set voltage level RoboClaw will stop driving the motors.
        The voltage is set in .2 volt increments.
        A value of 0 sets the minimum value allowed which is 6V.

        Args:
            val (int): The valid data range is 0 - 140 (6V - 34V).
            The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
            Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.SETMINMB, val)

    def SetMaxVoltageMainBattery(self, val: int) -> bool:
        """3 - Set Maximum Main Voltage (Command 57 Preferred)

        Sets main battery (B- / B+) maximum voltage level. During regenerative breaking a back voltage is applied to charge the battery. When using a power supply, by setting the maximum voltage level, RoboClaw will, before exceeding it, go into hard braking mode until the voltage drops below the maximum value set. This will prevent overvoltage conditions when using power supplies.

        Args:
            val (int): The valid data range is 30 - 175 (6V - 34V).
            The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
            Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.SETMAXMB, val)

    def ForwardM2(self, val: int) -> bool:
        """4 - Drive Forward M2

        Drive motor 2 forward.

        Args:
            val (int): M2 Speed.
            Valid data range is 0 - 127.
            A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.M2FORWARD, val)

    def BackwardM2(self, val: int) -> bool:
        """5 - Drive Backwards M2

        Drive motor 2 backwards.

        Args:
            val (int): M2 Speed.
            Valid data range is 0 - 127.
            A value of 127 = full speed backwards, 64 = about half speed backward and 0 = full stop.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.M2BACKWARD, val)

    def ForwardBackwardM1(self, val: int) -> bool:
        """6 - Drive M1 (7 Bit)

        Drive motor 1 forward or reverse.

        Args:
            val (int): M1 Speed.
            Valid data range is 0 - 127.
            A value of 0 = full speed reverse, 64 = stop and 127 = full speed forward.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.M17BIT, val)

    def ForwardBackwardM2(self, val: int) -> bool:
        """7 - Drive M2 (7 Bit)

        Drive motor 2 forward or reverse.

        Args:
            val (int): M2 Speed.
            Valid data range is 0 - 127.
            A value of 0 = full speed reverse, 64 = stop and 127 = full speed forward.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.M27BIT, val)

    # Mixed mode
    def MixedForward(self, val: int) -> bool:
        """8 - Drive Forward

        Drive forward in mix mode.

        Args:
            val (int): Forward speed.
            Valid data range is 0 - 127.
            A value of 0 = full stop and 127 = full forward.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.MIXEDFORWARD, val)

    def MixedBackward(self, val: int) -> bool:
        """9 - Drive Backwards

        Drive backards in mix mode.

        Args:
            val (int): Backward speed.
            Valid data range is 0 - 127.
            A value of 0 = full stop and 127 = full reverse.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.MIXEDBACKWARD, val)

    def MixedTurnRight(self, val: int) -> bool:
        """10 - Turn right

        Turn right in mix mode.

        Args:
            val (int): Turn speed.
            Valid data range is 0 - 127.
            A value of 0 = stop turn and 127 = full speed turn.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.MIXEDRIGHT, val)

    def MixedTurnLeft(self, val: int) -> bool:
        """11 - Turn left

        Turn left in mix mode.

        Args:
            val (int): Turn speed.
            Valid data range is 0 - 127.
            A value of 0 = stop turn and 127 = full speed turn.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.MIXEDLEFT, val)

    def MixedForwardBackward(self, val: int) -> bool:
        """12 - Drive Forward or Backward (7 Bit)

        Drive forward or backwards.

        Args:
            val (int): Linear speed.
            Valid data range is 0 - 127.
            A value of 0 = full backward, 64 = stop and 127 = full forward.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.MIXEDFB, val)

    def MixedLeftRight(self, val: int) -> bool:
        """13 - Turn Left or Right (7 Bit)

        Turn left or right.

        Args:
            val (int): Turn speed.
            Valid data range is 0 - 127.
            A value of 0 = full left, 0 = stop turn and 127 = full right.

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.MIXEDLR, val)

    def SetSerialTimeout(self, val: int) -> bool:
        """14 - Set Serial Timeout

        Sets the serial communication timeout in 100ms increments. When serial bytes are received in the time specified both motors will stop automatically.

        Args:
            val (int): Serial communication timeout.
            Range is 0 to 25.5 seconds (0 to 255 in 100ms increments).

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.SETSERIALTIMEOUT, val)

    def ReadSerialTimeout(self) -> Sequence[int, int]:
        """15 - Read Serial Timeout

        Read the current serial timeout setting.

        Returns:
            - int: Command acknowledgement
            - int: Serial timeout setting.
            Range is 0 to 255.
        """
        return self._read1(self.address, CMD.GETSERIALTIMEOUT)

    def ReadEncM1(self) -> Sequence[int, int, int]:
        """16 - Read Encoder Count/Value M1

        Read M1 encoder count/position.

        Quadrature encoders have a range of 0 to 4,294,967,295.
        Absolute encoder values are converted from an analog voltage
        into a value from 0 to 2047 for the full 2V range.

        Status byte tracks counter underflow, direction and overflow.
        The byte value represents:
        Bit 0 - Counter Underflow (1 = Underflow Occurred, Cleared After Reading)
        Bit 1 - Direction (0 = Forward, 1 = Backwards)
        Bit 2 - Counter Overflow (1 = Underflow Occurred, Cleared After Reading)
        Bit 3 - Reserved
        Bit 4 - Reserved
        Bit 5 - Reserved
        Bit 6 - Reserved
        Bit 7 - Reserved

        Returns:
            - int: Command acknowledgement
            - int: M1 encoder count
            - int: Status byte
        """
        return self._read41(self.address, CMD.GETM1ENC)

    def ReadEncM2(self) -> Sequence[int, int, int]:
        """17 - Read Encoder Count/Value M2

        Read M2 encoder count/position.

        Quadrature encoders have a range of 0 to 4,294,967,295.
        Absolute encoder values are converted from an analog voltage
        into a value from 0 to 2047 for the full 2V range.

        Status byte tracks counter underflow, direction and overflow.
        The byte value represents:
        Bit 0 - Counter Underflow (1 = Underflow Occurred, Cleared After Reading)
        Bit 1 - Direction (0 = Forward, 1 = Backwards)
        Bit 2 - Counter Overflow (1 = Underflow Occurred, Cleared After Reading)
        Bit 3 - Reserved
        Bit 4 - Reserved
        Bit 5 - Reserved
        Bit 6 - Reserved
        Bit 7 - Reserved

        Returns:
            - int: Command acknowledgement
            - int: M2 encoder count
            - int: Status byte
        """
        return self._read41(self.address, CMD.GETM2ENC)

    def ReadSpeedM1(self) -> Sequence[int, int, int]:
        """18 - Read Encoder Speed M1

        Read M1 counter speed.
        Returned value is in pulses per second.
        RoboClaw keeps track of how many pulses received per second
        for both encoder channels.

        Status byte indicates the direction.
        0 forwards, 1 backwards.

        Returns:
            - int: Command acknowledgement
            - int: M1 encoder speed
            - int: Status byte - Direction (0: forward / 1 : backward)
        """
        return self._read41(self.address, CMD.GETM1SPEED)

    def ReadSpeedM2(self) -> Sequence[int, int, int]:
        """19 - Read Encoder Speed M2

        Read M2 counter speed.
        Returned value is in pulses per second.
        RoboClaw keeps track of how many pulses received per second
        for both encoder channels.

        Status byte indicates the direction.
        0 forwards, 1 backwards.

        Returns:
            - int: Command acknowledgement
            - int: M2 encoder speed
            - int: Status byte - Direction (0: forward / 1 : backward)
        """
        return self._read41(self.address, CMD.GETM2SPEED)

    def ResetEncoders(self) -> bool:
        """20 - Reset Quadrature Encoder Counters

        Will reset both quadrature decoder counters to zero.
        This command applies to quadrature encoders only.

        Returns:
            bool: Command acknowledgement
        """
        return self._write0(self.address, CMD.RESETENC)

    def ReadVersion(self) -> Sequence[int, str]:
        """21 - Read Firmware Version

        Read RoboClaw firmware version.
        Returns up to 48 bytes(depending on the Roboclaw model) and is terminated by a line feed character and a null character.
        The return string includes the product name and firmware version. The return string is terminated with a line feed (10) and null (0) character.

        Returns:
            - int: Command acknowledgement
            - str: < 48-bytes string, terminated by line feed and null characters
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(self.address, CMD.GETFWVERSION)
            version_str = ""
            passed = True
            for i in range(48):
                data = self._port.read(1)
                if len(data):
                    val = ord(data)
                    self.crc_update(val)
                    if val == 0:
                        break
                    version_str += chr(data[0])
                else:
                    passed = False
                    break
            if passed:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF == crc[1] & 0xFFFF:
                        return (1, version_str)
                    else:
                        time.sleep(0.01)
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def SetEncM1(self, count: int) -> bool:
        """22 - Set Quadrature Encoder 1 Value

        Set the value of the Encoder 1 register.
        Useful when homing motor 1.
        This command applies to quadrature encoders only.

        Args:
            count (int): Value to set to the Encoder 1 register.

        Returns:
            bool: Command acknowledgement
        """
        return self._write4(self.address, CMD.SETM1ENCCOUNT, count)

    def SetEncM2(self, count: int) -> bool:
        """23 - Set Quadrature Encoder 2 Value

        Set the value of the Encoder 2 register.
        Useful when homing motor 2.
        This command applies to quadrature encoders only.

        Args:
            count (int): Value to set to the Encoder 2 register.

        Returns:
            bool: Command acknowledgement
        """
        return self._write4(self.address, CMD.SETM2ENCCOUNT, count)

    def ReadMainBatteryVoltage(self) -> Sequence[int, int]:
        """24 - Read Main Battery Voltage Level

        Read the main battery voltage level connected to B+ and B- terminals.
        The voltage is returned in 10ths of a volt(eg 300 = 30v).

        Returns:
            - int: Command acknowledgement
            - int: Main battery voltage level.
            Voltage is returned in 10ths of a volt (eg 300 = 30V).
        """
        return self._read2(self.address, CMD.GETMBATTVOLT)

    def ReadLogicBatteryVoltage(self) -> Sequence[int, int]:
        """25 - Read Logic Battery Voltage Level

        Read a logic battery voltage level connected to LB+ and LB- terminals.
        The voltage is returned in 10ths of a volt(eg 300 = 30v).

        Returns:
            - int: Command acknowledgement
            - int: Logic battery voltage level.
            Voltage is returned in 10ths of a volt (eg 50 = 5V).
        """
        return self._read2(self.address, CMD.GETLBATTVOLT)

    def SetMinVoltageLogicBattery(self, val: int) -> bool:
        """26 - Set Minimum Logic Voltage Level

        NOTE: THIS COMMAND IS INCLUDED FOR BACKWARDS COMPATIBILITY.
        WE RECOMMEND YOU USE COMMAND 58 INSTEAD.

        Sets logic input (LB- / LB+) minimum voltage level.
        RoboClaw will shut down with an error if the voltage is below this level.

        The voltage is set in .2 volt increments.
        The valid data range is 0 - 140 (6V - 34V).
        A value of 0 sets the minimum value allowed which is 6V.
        The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
        Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.

        Args:
            - val (int): Minimum logic voltage level.
            Voltage is set in .2 Volt increments.
            A value of 0 sets the minimum value allowed which is 6V.
            The valid data range is 0 - 140 (6V - 34V).
            The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
            Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write1(self.address, CMD.SETMINLB, val)

    def SetMaxVoltageLogicBattery(self, val: int) -> bool:
        """27 - Set Maximum Logic Voltage Level

        NOTE: THIS COMMAND IS INCLUDED FOR BACKWARDS COMPATIBILITY.
        WE RECOMMEND YOU USE COMMAND 58 INSTEAD.

        Sets logic input (LB- / LB+) maximum voltage level.
        RoboClaw will shutdown with an error if the voltage is above this level.

        The valid data range is 30 - 175 (6V - 34V).
        The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
        Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.

        Args:
            - val (int): Maximum logic voltage level.
            The valid data range is 30 - 175 (6V - 34V).
            The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
            Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write1(self.address, CMD.SETMAXLB, val)

    def SetM1VelocityPID(
        self,
        p: int,
        i: int,
        d: int,
        qpps: int,
    ) -> bool:
        """28 - Set Velocity PID Constants M1

        Several motor and quadrature combinations can be used with RoboClaw.
        In some cases, the default PID values will need to be tuned for the driven system.
        This gives greater flexibility in what motor and encoder combinations can be used.
        The RoboClaw PID system consist of four constants starting with QPPS, P = Proportional, I= Integral and D= Derivative.
        QPPS (quadrature pulses per second) is the speed of the encoder when the motor is at 100% power. P, I, D are the default values used after a reset.

        Args:
            - p (int): proportional constant (default: 0x00010000)
            - i (int): integral constant (default: 0x00008000)
            - d (int): derivative constant (default: 0x00004000)
            - qpps (int): encoder speed when the motor is at 100% power (default: 44000)

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4444(
            self.address,
            CMD.SETM1PID,
            d * 65536,
            p * 65536,
            i * 65536,
            qpps,
        )

    def SetM2VelocityPID(
        self,
        p: int,
        i: int,
        d: int,
        qpps: int,
    ) -> bool:
        """29 - Set Velocity PID Constants M2

        Several motor and quadrature combinations can be used with RoboClaw.
        In some cases, the default PID values will need to be tuned for the driven system.
        This gives greater flexibility in what motor and encoder combinations can be used.
        The RoboClaw PID system consist of four constants starting with QPPS, P = Proportional, I= Integral and D= Derivative.
        QPPS (quadrature pulses per second) is the speed of the encoder when the motor is at 100% power. P, I, D are the default values used after a reset.

        Args:
            - p (int): proportional constant (default: 0x00010000)
            - i (int): integral constant (default: 0x00008000)
            - d (int): derivative constant (default: 0x00004000)
            - qpps (int): encoder speed when the motor is at 100% power (default: 44000)

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4444(
            self.address,
            CMD.SETM2PID,
            d * 65536,
            p * 65536,
            i * 65536,
            qpps,
        )

    def ReadISpeedM1(self) -> Sequence[int, int, int]:
        """30 - Read Raw Speed M1

        Read the pulses counted in that last 300th of a second.
        This is an unfiltered version of command 18.
        Command 30 can be used to make a independent PID routine.
        Returned value is in encoder counts per second.

        Status byte indicates the direction.
        0 forwards, 1 backwards.

        Returns:
            - int: Command acknowledgement
            - int: M1 raw speed (in encoder counts per second)
            - int: Status byte - Direction (0: forward / 1 : backward)
        """
        return self._read41(self.address, CMD.GETM1ISPEED)

    def ReadISpeedM2(self) -> Sequence[int, int, int]:
        """31 - Read Raw Speed M2

        Read the pulses counted in that last 300th of a second.
        This is an unfiltered version of command 19.
        Command 31 can be used to make a independent PID routine.
        Returned value is in encoder counts per second.

        Status byte indicates the direction.
        0 forwards, 1 backwards.

        Returns:
            - int: Command acknowledgement
            - int: M2 raw speed (in encoder counts per second)
            - int: Status byte - Direction (0: forward / 1 : backward)
        """
        return self._read41(self.address, CMD.GETM2ISPEED)

    def DutyM1(self, val: int) -> bool:
        """32 - Drive M1 With Signed Duty Cycle

        Drive M1 using a duty cycle value.
        The duty cycle is used to control the speed of the motor
        without a quadrature encoder.
        The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty).

        Args:
            val (int): Signed Duty Value. Range is -32767 to +32767 (eg. +-100% duty)

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS2(self.address, CMD.M1DUTY, val)

    def DutyM2(self, val: int) -> bool:
        """33 - Drive M2 With Signed Duty Cycle

        Drive M2 using a duty cycle value.
        The duty cycle is used to control the speed of the motor
        without a quadrature encoder.
        The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty).

        Args:
            val (int): Signed Duty Value.
            Range is -32767 to +32767 (eg. +-100% duty)

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS2(self.address, CMD.M2DUTY, val)

    def DutyM1M2(self, m1_val: int, m2_val: int) -> bool:
        """34 - Drive M1 / M2 With Signed Duty Cycle

        Drive both M1 and M2 using a duty cycle value.
        The duty cycle is used to control the speed of the motor
        without a quadrature encoder.
        The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty)

        Args:
            - m1_val (int): M1 Signed Duty Value.
            Range is -32767 to +32767 (eg. +-100% duty)
            - m2_val (int): M2 Signed Duty Value.
            Range is -32767 to +32767 (eg. +-100% duty)

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS2S2(self.address, CMD.MIXEDDUTY, m1_val, m2_val)

    def SpeedM1(self, speed: int) -> bool:
        """35 - Drive M1 With Signed Speed

        Drive M1 using a speed value.
        The sign indicates which direction the motor will turn.
        This command is used to drive the motor by quad pulses per second.
        Different quadrature encoders will have different rates at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent the motor will begin to accelerate as fast as possible until the defined rate is reached.

        Args:
            speed (int): M1 signed speed

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS4(self.address, CMD.M1SPEED, speed)

    def SpeedM2(self, speed: int) -> bool:
        """36 - Drive M2 With Signed Speed

        Drive M2 using a speed value.
        The sign indicates which direction the motor will turn. This command is used to drive the motor by quad pulses per second.
        Different quadrature encoders will have different rates at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent the motor will begin to accelerate as fast as possible until the defined rate is reached.

        Args:
            speed (int): M2 signed speed

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS4(self.address, CMD.M2SPEED, speed)

    def SpeedM1M2(self, m1_speed: int, m2_speed: int) -> bool:
        """37 - Drive M1 / M2 With Signed Speed

        Drive M1 and M2 in the same command using a signed speed value.
        The sign indicates which direction the motor will turn.
        This command is used to drive both motors by quad pulses per second.
        Different quadrature encoders will have different rates at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent the motor will begin to accelerate as fast as possible until the defined rate is reached.

        Args:
            - m1_speed (int): M1 signed speed, in QPPS.
            - m2_speed (int): M2 signed speed, in QPPS.

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS4S4(self.address, CMD.MIXEDSPEED, m1_speed, m2_speed)

    def SpeedAccelM1(self, accel: int, speed: int) -> bool:
        """38 - Drive M1 With Signed Speed And Acceleration

        Drive M1 with a signed speed and acceleration value.
        The sign indicates which direction the motor will run.
        The acceleration values are not signed.
        This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping.
        Different quadrature encoders will have different rates at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent, the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second.
        An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
        Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.

        Args:
            - accel (int): Unsigned acceleration, in QPPS per second.
            - speed (int): M1 signed speed, in QPPS.
            The sign indicates in which direction the motor will run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S4(self.address, CMD.M1SPEEDACCEL, accel, speed)

    def SpeedAccelM2(self, accel: int, speed: int) -> bool:
        """39 - Drive M2 With Signed Speed And Acceleration

        Drive M2 with a signed speed and acceleration value.
        The sign indicates which direction the motor will run.
        The acceleration values are not signed.
        This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping.
        Different quadrature encoders will have different rates at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent, the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second.
        An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
        Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.

        Args:
            - accel (int): Unsigned acceleration, in QPPS per second.
            - speed (int): M2 signed speed, in QPPS.
            The sign indicates in which direction the motor will run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S4(self.address, CMD.M2SPEEDACCEL, accel, speed)

    def SpeedAccelM1M2(self, accel: int, m1_speed: int, m2_speed: int) -> bool:
        """40 - Drive M1 / M2 With Signed Speed And Acceleration

        Drive M1 and M2 in the same command using one value for acceleration and two signed speed values for each motor.
        The sign indicates which direction the motor will run.
        The acceleration value is not signed.
        The motors are sync during acceleration.
        This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping.
        Different quadrature encoders will have different rates at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent, the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second.
        An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
        Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.

        Args:
            - accel (int): Unsigned acceleration, in QPPS per second.
            - m1_speed (int): M1 signed speed, in QPPS.
            The sign indicates in which direction the motor will run.
            - m2_speed (int): M2 signed speed, in QPPS.
            The sign indicates in which direction the motor will run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S4S4(
            self.address,
            CMD.MIXEDSPEEDACCEL,
            accel,
            m1_speed,
            m2_speed,
        )

    def BuffSpeedDistanceM1(
        self, speed: int, distance: int, buffer: Literal[0, 1] | None = 0
    ) -> bool:
        """41 - Buffered M1 Drive With Signed Speed And Distance

        Drive M1 with a signed speed and distance value.
        This command is used to control the top speed and total distance traveled by the motor.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - speed (int): M1 signed top speed, in QPPS.
            The sign indicates in which direction the motor will run.
            - distance (int): Unsigned distance value, in QPPS.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS441(
            self.address,
            CMD.BUF_M1SPEEDDIST,
            speed,
            distance,
            buffer,
        )

    def BuffSpeedDistanceM2(
        self, speed: int, distance: int, buffer: Literal[0, 1] | None = 0
    ) -> bool:
        """42 - Buffered M2 Drive With Signed Speed And Distance

        Drive M2 with a signed speed and distance value.
        This command is used to control the top speed and total distance traveled by the motor.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - speed (int): M2 signed top speed, in QPPS.
            The sign indicates in which direction the motor will run.
            - distance (int): Unsigned distance value, in QPPS.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS441(
            self.address,
            CMD.BUF_M2SPEEDDIST,
            speed,
            distance,
            buffer,
        )

    def BuffSpeedDistanceM1M2(
        self,
        m1_speed: int,
        m1_distance: int,
        m2_speed: int,
        m2_distance: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """43 - Buffered Drive M1 / M2 With Signed Speed And Distance

        Drive M1 and M2 with a speed and distance value.
        This command is used to control the top speed and total distance traveled by the motor.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - m1_speed (int): M1 signed top speed, in QPPS.
            The sign indicates in which direction the motor will run.
            - m1_distance (int): Unsigned M1 distance value, in QPPS.
            - m2_speed (int): M2 signed top speed, in QPPS.
            The sign indicates in which direction the motor will run.
            - m2_distance (int): Unsigned M2 distance value, in QPPS.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS44S441(
            self.address,
            CMD.BUF_MIXEDSPEEDDIST,
            m1_speed,
            m1_distance,
            m2_speed,
            m2_distance,
            buffer,
        )

    def BuffSpeedAccelDistanceM1(
        self,
        accel: int,
        speed: int,
        distance: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """44 - Buffered M1 Drive With Signed Speed, Accel And Distance

        Drive M1 with a speed, acceleration and distance value.
        This command is used to control the motor's top speed, total traveled distance and its incremental acceleration at which it will reach top speed.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.
        Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument.

        Args:
            - accel (int): Unsigned acceleration, in QPPS per second.
            - speed (int): M1 signed top speed, in QPPS.
            The sign indicates which direction the motor will run.
            - distance (int): Unsigned distance value, in Quad Pulses.
            - buffer ([0, 1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S441(
            self.address,
            CMD.BUF_M1SPEEDACCELDIST,
            accel,
            speed,
            distance,
            buffer,
        )

    def BuffSpeedAccelDistanceM2(
        self,
        accel: int,
        speed: int,
        distance: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """45 - Buffered M2 Drive With Signed Speed, Accel And Distance

        Drive M2 with a speed, acceleration and distance value.
        This command is used to control the motor's top speed, total traveled distance and its incremental acceleration at which it will reach top speed.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.
        Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument.

        Args:
            - accel (int): Unsigned acceleration, in QPPS per second.
            - speed (int): M2 signed top speed, in QPPS.
            The sign indicates which direction the motor will run.
            - distance (int): Unsigned distance value, in Quad Pulses.
            - buffer ([0, 1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S441(
            self.address,
            CMD.BUF_M2SPEEDACCELDIST,
            accel,
            speed,
            distance,
            buffer,
        )

    def BuffSpeedAccelDistanceM1M2(
        self,
        accel: int,
        m1_speed: int,
        m1_distance: int,
        m2_speed: int,
        m2_distance: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """46 - Buffered Drive M1 / M2 With Signed Speed, Accel And Distance

        Drive M1 and M2 with a speed, acceleration and distance value.
        This command is used to control both motors' top speed, total traveled distances and their incremental acceleration at which it will reach top speed.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.
        Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument.

        Args:
            - accel (int): Unsigned acceleration, in QPPS per second.
            - m1_speed (int): M1 signed top speed, in QPPS.
            The sign indicates which direction the motor will run.
            - m1_distance (int): Unsigned M1 distance value, in QPPS.
            - m2_speed (int): M2 signed top speed, in QPPS.
            The sign indicates which direction the motor will run.
            - m2_distance (int): Unsigned M2 distance value, in QPPS.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S44S441(
            self.address,
            CMD.BUF_MIXEDSPEEDACCELDIST,
            accel,
            m1_speed,
            m1_distance,
            m2_speed,
            m2_distance,
            buffer,
        )

    def ReadBufferLengths(self) -> Sequence[int, int, int]:
        """47 - Read Buffer Length

        Read both motor M1 and M2 buffer lengths.
        This command can be used to determine how many commands are waiting to execute.

        The returned values represent how many commands per buffer
        are waiting to be executed.
        The maximum buffer size per motor is 64 commands(0x3F).
        A return value of 0x80(128) indicates the buffer is empty.
        A return value of 0 indicates the last command sent is executing.
        A value of 0x80 indicates the last buffered command has finished.

        Returns:
            - int: Command Acknowledgement
            - int: M1 buffer length.
            Maximum buffer size per motor is 64 commands (0x3F).
            0x80 (128) indicates the buffer is empty.
            0 indicates the last sent command is executing.
            0x80 indicates the last buffered command has finished.
            - int: M2 buffer length.
            Maximum buffer size per motor is 64 commands (0x3F).
            0x80 (128) indicates the buffer is empty.
            0 indicates the last sent command is executing.
            0x80 indicates the last buffered command has finished.
        """
        val = self._read2(self.address, CMD.GETBUFFERS)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def ReadPWMs(self) -> Sequence[int, int, int]:
        """48 - Read Motor PWM values

        Read the current PWM output values for the motor channels.
        The values returned are +/-32767.
        The duty cycle percent is calculated by dividing the Value by 327.67.

        Returns:
            - int: Command Acknowledgement
            - int: Motor 1 PWM value. The duty cycle percent is calculated by dividing the Value by 327.67.
            - int: Motor 2 PWM value. The duty cycle percent is calculated by dividing the Value by 327.67.
        """
        val = self._read4(self.address, CMD.GETPWMS)
        if val[0]:
            pwm1 = val[1] >> 16
            pwm2 = val[1] & 0xFFFF
            if pwm1 & 0x8000:
                pwm1 -= 0x10000
            if pwm2 & 0x8000:
                pwm2 -= 0x10000
            return (1, pwm1, pwm2)
        return (0, 0, 0)

    def ReadCurrents(self) -> Sequence[int, int, int]:
        """49 - Read Motor Currents

        Read the current draw from each motor in 10 mA increments. The amps value is calculated by dividing the value by 100.

        Returns:
            - int: Command acknowledgement
            - int: M1 Current
            - int: M2 Current
        """
        val = self._read4(self.address, CMD.GETCURRENTS)
        if val[0]:
            cur1 = val[1] >> 16
            cur2 = val[1] & 0xFFFF
            if cur1 & 0x8000:
                cur1 -= 0x10000
            if cur2 & 0x8000:
                cur2 -= 0x10000
            return (1, cur1, cur2)
        return (0, 0, 0)

    def IndepSpeedAccelM1M2(
        self,
        m1_accel: int,
        m1_speed: int,
        m2_accel: int,
        m2_speed: int,
    ) -> bool:
        """50 - Drive M1 / M2 With Signed Speed And Individual Acceleration

        Drive M1 and M2 in the same command using two accelerations and two signed speed values, one for each motor.
        The signs indicates which direction the motors will run.
        The acceleration values are not signed.
        The motors are sync during acceleration.

        This command is used to drive the motor by quad pulses per second and using acceleration values for ramping.
        Different quadrature encoders will have different rates
        at which they generate the incoming pulses.
        The values used will differ from one encoder to another.
        Once a value is sent, the motor will begin to accelerate incrementally until the rate defined is reached.

        The acceleration is measured in speed increase per second.
        An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS
        would accelerate a motor from 0 to 12,000 QPPS in 1 second.
        Another example would be an acceleration value of 24,000 QPPS and a speed value
        of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.

        Args:
            - m1_accel (int): M1 unsigned acceleration, in QPPS per second.
            - m1_speed (int): M1 signed speed, in QPPS.
            The sign indicates in which direction the motor will run.
            - m2_accel (int): M2 unsigned acceleration, in QPPS per second.
            - m2_speed (int): M2 signed speed, in QPPS.
            The sign indicates in which direction the motor will run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S44S4(
            self.address,
            CMD.MIXEDSPEED2ACCEL,
            m1_accel,
            m1_speed,
            m2_accel,
            m2_speed,
        )

    def BuffIndepSpeedAccelDistanceM1M2(
        self,
        m1_accel: int,
        m1_speed: int,
        m1_distance: int,
        m2_accel: int,
        m2_speed: int,
        m2_distance: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """51 - Buffered Drive M1 / M2 With Signed Speed, Individual Accel And Distance

        Drive M1 and M2 with different speed, acceleration and distance values.
        This command is used to control both motors' top speed, total traveled distances and their incremental accelerations at which they will reach top speed.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.
        Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument.

        Args:
            - m1_accel (int): Unsigned M1 acceleration, in QPPS per second.
            - m1_speed (int): M1 signed top speed, in QPPS.
            The sign indicates which direction the motor will run.
            - m1_distance (int): Unsigned M1 distance value, in QPPS.
            - m2_accel (int): Unsigned M2 acceleration, in QPPS per second.
            - m2_speed (int): M2 signed top speed, in QPPS.
            The sign indicates which direction the motor will run.
            - m2_distance (int): Unsigned M2 distance value, in QPPS.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4S444S441(
            self.address,
            CMD.BUF_MIXEDSPEED2ACCELDIST,
            m1_accel,
            m1_speed,
            m1_distance,
            m2_accel,
            m2_speed,
            m2_distance,
            buffer,
        )

    def DutyAccelM1(self, accel: int, duty: int) -> bool:
        """52 - Drive M1 With Signed Duty And Acceleration

        Drive M1 with a signed duty and acceleration value.
        This command is used to drive the motor by PWM and using an acceleration value for ramping.
        Accel is the rate per second at which the duty changes
        from the current duty to the specified duty.

        Args:
            - accel (int): Unsigned M1 acceleration.
            Range is 0 to 655359 (maximum acceleration rate is -100% to 100% in 100ms)
            - duty (int): Signed M1 duty value.
            The sign indicates which direction the motor will run.
            Range is -32768 to +32767 (eg. +-100% duty)

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS24(
            self.address,
            CMD.M1DUTYACCEL,
            duty,
            accel,
        )

    def DutyAccelM2(self, accel: int, duty: int) -> bool:
        """53 - Drive M2 With Signed Duty And Acceleration

        Drive M2 with a signed duty and acceleration value.
        This command is used to drive the motor by PWM and using an acceleration value for ramping.
        Accel is the rate per second at which the duty changes
        from the current duty to the specified duty.

        Args:
            - accel (int): Unsigned M2 acceleration.
            Range is 0 to 655359 (maximum acceleration rate is -100% to 100% in 100ms)
            - duty (int): Signed M2 duty value.
            The sign indicates which direction the motor will run.
            Range is -32768 to +32767 (eg. +-100% duty)

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS24(
            self.address,
            CMD.M2DUTYACCEL,
            duty,
            accel,
        )

    def DutyAccelM1M2(
        self,
        m1_accel: int,
        m1_duty: int,
        m2_accel: int,
        m2_duty: int,
    ) -> bool:
        """54 - Drive M1 / M2 With Signed Duty And Acceleration

        Drive M1 and M2 in the same command using acceleration and signed duty values for each motor.
        This command is used to drive motors by PWM with an acceleration value for ramping.
        Accel is the rate per second at which the duty changes
        from the current duty to the specified duty.

        Args:
            - m1_accel (int): Unsigned M1 acceleration.
            Range is 0 to 655359 (maximum acceleration rate is -100% to 100% in 100ms)
            - m1_duty (int): Signed M1 duty value.
            The sign indicates which direction the motor will run.
            Range is -32768 to +32767 (eg. +-100% duty)
            - m2_accel (int): Unsigned M2 acceleration.
            Range is 0 to 655359 (maximum acceleration rate is -100% to 100% in 100ms)
            - m2_duty (int): Signed M2 duty value.
            The sign indicates which direction the motor will run.
            Range is -32768 to +32767 (eg. +-100% duty)

        Returns:
            bool: Command Acknowledgement
        """
        return self._writeS24S24(
            self.address,
            CMD.MIXEDDUTYACCEL,
            m1_duty,
            m1_accel,
            m2_duty,
            m2_accel,
        )

    def ReadM1VelocityPID(self) -> Sequence[int, int, int, int, int]:
        """55 - Read Motor 1 Velocity PID and QPPS Settings

        Read the M1 PID and QPPS Settings.

        Returns:
            - int: Command acknowledgement
            - int: Value of P in M1's Velocity PID
            - int: Value of I in M1's Velocity PID
            - int: Value of D in M1's Velocity PID
            - int: M1 QPPS setting
        """
        data = self._read_n(self.address, CMD.READM1PID, 4)
        if data[0]:
            data[1] /= 65536.0
            data[2] /= 65536.0
            data[3] /= 65536.0
            return data
        return (0, 0, 0, 0, 0)

    def ReadM2VelocityPID(self) -> Sequence[int, int, int, int, int]:
        """56 - Read Motor 2 Velocity PID and QPPS Settings

        Read the M2 PID and QPPS Settings.

        Returns:
            - int: Command acknowledgement
            - int: Value of P in M2's Velocity PID
            - int: Value of I in M2's Velocity PID
            - int: Value of D in M2's Velocity PID
            - int: M2 QPPS setting
        """
        data = self._read_n(self.address, CMD.READM2PID, 4)
        if data[0]:
            data[1] /= 65536.0
            data[2] /= 65536.0
            data[3] /= 65536.0
            return data
        return (0, 0, 0, 0, 0)

    def SetMainVoltages(self, min_volt: int, max_volt: int) -> bool:
        """57 - Set Main Battery Voltages

        Set the Main Battery Voltage cutoffs, Min and Max.
        Min and Max voltages are in 10th of a volt increments.
        Multiply the voltage to set by 10.

        Args:
            - min_volt (int): Minimum main voltage level.
            Multiply the voltage to set by 10.
            - max_volt (in_t): M_aximum main voltage level.
            Multiply the voltage to set by 10.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write22(self.address, CMD.SETMAINVOLTAGES, min_volt, max_volt)

    def SetLogicVoltages(self, min_volt: int, max_volt: int) -> bool:
        """58 - Set Logic Battery Voltages

        Set the Logic Battery Voltage cutoffs, Min and Max.
        Min and Max voltages are in 10th of a volt increments.
        Multiply the voltage to set by 10.

        Args:
            - min_volt (int): Minimum logic voltage level.
            Multiply the voltage to set by 10.
            - max_volt (int): Maximum logic voltage level.
            Multiply the voltage to set by 10.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write22(self.address, CMD.SETLOGICVOLTAGES, min_volt, max_volt)

    def ReadMinMaxMainVoltages(self) -> Sequence[int, int, int]:
        """59 - Read Main Battery Voltage Settings

        Read the Main Battery Voltage Settings.
        The voltage is calculated by dividing the value by 10

        Returns:
            - int: Command Acknowledgement
            - int: Minimum main voltage level. The voltage is calculated by dividing the value by 10.
            - int: Maximum main voltage level. The voltage is calculated by dividing the value by 10.
        """
        val = self._read4(self.address, CMD.GETMINMAXMAINVOLTAGES)
        if val[0]:
            min_volt = val[1] >> 16
            max_volt = val[1] & 0xFFFF
            return (1, min_volt, max_volt)
        return (0, 0, 0)

    def ReadMinMaxLogicVoltages(self) -> Sequence[int, int, int]:
        """60 - Read Logic Battery Voltage Settings

        Read the Logic Battery Voltage Settings.
        The voltage is calculated by dividing the value by 10

        Returns:
            - int: Command Acknowledgement
            - int: Minimum logic voltage level.
            The voltage is calculated by dividing the value by 10.
            - int: Maximum logic voltage level.
            The voltage is calculated by dividing the value by 10.
        """
        val = self._read4(self.address, CMD.GETMINMAXLOGICVOLTAGES)
        if val[0]:
            min_volt = val[1] >> 16
            max_volt = val[1] & 0xFFFF
            return (1, min_volt, max_volt)
        return (0, 0, 0)

    def SetM1PositionPID(
        self,
        kp: int,
        ki: int,
        kd: int,
        ki_max: int,
        deadzone: int,
        min_pos: int,
        max_pos: int,
    ) -> bool:
        """61 - Set Motor 1 Position PID Constants

        The RoboClaw Position PID system consist of seven constants,
        which are the 7 arguments of this method.
        The default values are all zero.
        Position constants are used only with the position commands (65, 66, 67)
        or when encoders are enabled in RC/Analog mode.

        Args:
            - kp (int): M1 position PID Proportional constant
            - ki (int): M1 position PID Integral constant
            - kd (int): M1 position PID Derivative constant
            - ki_max (int): Maximum integral windup
            - Deadzone (int): Deadzone, in encoder counts.
            - min_pos (int): Minimum position value
            - max_pos (int): Maximum position value

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4444444(
            self.address,
            CMD.SETM1POSPID,
            kd * 1024,
            kp * 1024,
            ki * 1024,
            ki_max,
            deadzone,
            min_pos,
            max_pos,
        )

    def SetM2PositionPID(
        self,
        kp: int,
        ki: int,
        kd: int,
        ki_max: int,
        deadzone: int,
        min_pos: int,
        max_pos: int,
    ) -> bool:
        """62 - Set Motor 2 Position PID Constants

        The RoboClaw Position PID system consist of seven constants, which are the 7 arguments of this method.
        The default values are all zero.
        Position constants are used only with the position commands (65, 66, 67)
        or when encoders are enabled in RC/Analog mode.

        Args:
            - kp (int): M2 position PID Proportional constant
            - ki (int): M2 position PID Integral constant
            - kd (int): M2 position PID Derivative constant
            - ki_max (int): Maximum integral windup
            - Deadzone (int): Deadzone, in encoder counts.
            - min_pos (int): Minimum position value
            - max_pos (int): Maximum position value

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4444444(
            self.address,
            CMD.SETM2POSPID,
            kd * 1024,
            kp * 1024,
            ki * 1024,
            ki_max,
            deadzone,
            min_pos,
            max_pos,
        )

    def ReadM1PositionPID(self) -> Sequence[int, int, int, int, int, int, int, int]:
        """63 - Read Motor 1 Position PID Constants

        Read the Position PID Settings for M1.

        Returns:
            - int: Command acknowledgement
            - int: M1 position PID Proportional constant
            - int: M1 position PID Integral constant
            - int: M1 position PID Derivative constant
            - int: Maximum integral windup
            - int: Deadzone, in encoder counts.
            - int: Minimum position value
            - int: Maximum position value
        """
        data = self._read_n(self.address, CMD.READM1POSPID, 7)
        if data[0]:
            data[1] /= 1024.0
            data[2] /= 1024.0
            data[3] /= 1024.0
            return data
        return (0, 0, 0, 0, 0, 0, 0, 0)

    def ReadM2PositionPID(self) -> Sequence[int, int, int, int, int, int, int, int]:
        """64 - Read Motor 2 Position PID Constants

        Read the Position PID Settings for M2.

        Returns:
            - int: Command acknowledgement
            - int: M2 position PID Proportional constant
            - int: M2 position PID Integral constant
            - int: M2 position PID Derivative constant
            - int: Maximum integral windup
            - int: Deadzone, in encoder counts.
            - int: Minimum position value
            - int: Maximum position value
        """
        data = self._read_n(self.address, CMD.READM2POSPID, 7)
        if data[0]:
            data[1] /= 1024.0
            data[2] /= 1024.0
            data[3] /= 1024.0
            return data
        return (0, 0, 0, 0, 0, 0, 0, 0)

    def BuffSpeedAccelDeccelPositionM1(
        self,
        accel: int,
        speed: int,
        deccel: int,
        position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """65 - Buffered Drive M1 with signed Speed, Accel, Deccel and Position

        Move M1 position from the current position to the specified new position and hold the new position.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - accel (int): M1 acceleration value
            - speed (int): M1 steady speed, in quadrature pulses, after acceleration and before decceleration.
            - deccel (int): M1 decceleration value
            - position (int): M1 goal position in quadrature pulses.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write44441(
            self.address,
            CMD.BUF_M1SPEEDACCELDECCELPOS,
            accel,
            speed,
            deccel,
            position,
            buffer,
        )

    def BuffSpeedAccelDeccelPositionM2(
        self,
        accel: int,
        speed: int,
        deccel: int,
        position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """66 - Buffered Drive M2 with signed Speed, Accel, Deccel and Position

        Move M2 position from the current position to the specified new position and hold the new position.
        All used values are in quad pulses per second.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - accel (int): M2 acceleration value
            - speed (int): M2 steady speed, in quadrature pulses, after acceleration and before decceleration.
            - deccel (int): M2 decceleration value
            - position (int): M2 goal position in quadrature pulses.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write44441(
            self.address,
            CMD.BUF_M2SPEEDACCELDECCELPOS,
            accel,
            speed,
            deccel,
            position,
            buffer,
        )

    def BuffSpeedAccelDeccelPositionM1M2(
        self,
        m1_accel: int,
        m1_speed: int,
        m1_deccel: int,
        m1_position: int,
        m2_accel: int,
        m2_speed: int,
        m2_deccel: int,
        m2_position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """67 - Buffered Drive M1 & M2 with signed Speed, Accel, Deccel and Position

        - speed (int): Speed, in quadrature pulses, of the motor after acceleration and before decceleration.

        Args:
            - m1_accel (int): M1 acceleration value
            - m1_speed (int): M1 steady speed, in quadrature pulses, after acceleration and before decceleration.
            - m1_deccel (int): M1 decceleration value
            - m1_position (int): M1 goal position in quadrature pulses.
            - m2_accel (int): M2 acceleration value
            - m2_speed (int): M2 steady speed, in quadrature pulses, after acceleration and before decceleration.
            - m2_deccel (int): M2 decceleration value
            - m2_position (int): M2 goal position in quadrature pulses.
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write444444441(
            self.address,
            CMD.BUF_MIXEDSPEEDACCELDECCELPOS,
            m1_accel,
            m1_speed,
            m1_deccel,
            m1_position,
            m2_accel,
            m2_speed,
            m2_deccel,
            m2_position,
            buffer,
        )

    def SetM1DefaultAccel(self, accel: int) -> bool:
        """68 - Set M1 Default Duty Acceleration

        Set the default acceleration for M1 when using duty cycle commands (Cmds 32, 33 and 34) or when using Standard Serial, RC and Analog PWM modes.

        Args:
            accel (int): Default M1 Duty Acceleration

        Returns:
            bool: Command acknowledgement
        """
        return self._write4(self.address, CMD.SETM1DEFAULTACCEL, accel)

    def SetM2DefaultAccel(self, accel: int) -> bool:
        """69 - Set M2 Default Duty Acceleration

        Set the default acceleration for M2 when using duty cycle commands (Cmds 32, 33 and 34) or when using Standard Serial, RC and Analog PWM modes.

        Args:
            accel (int): Default M2 Duty Acceleration

        Returns:
            bool: Command acknowledgement
        """
        return self._write4(self.address, CMD.SETM2DEFAULTACCEL, accel)

    def SetM1DefaultSpeed(self, speed: int) -> bool:
        """70 - Set Motor1 Default Speed

        Set M1 default speed for use with M1 position command and RC or analog modes
        when position control is enabled.
        This sets the percentage of the maximum speed set by QPSS as the default speed.
        The range is 0 to 32767.

        Args:
            speed (int): Default M1 Speed.
            This sets the percentage of the maximum speed set by QPSS as the default speed.
            The range is 0 to 32767.

        Returns:
            bool: Command acknowledgement
        """
        return self._write2(self.address, CMD.SETM1DEFAULTSPEED, speed)

    def SetM2DefaultSpeed(self, speed: int) -> bool:
        """71 - Set Motor2 Default Speed

        Set M2 default speed for use with M1 position command and RC or analog modes
        when position control is enabled.
        This sets the percentage of the maximu#m speed set by QPSS as the default speed.
        The range is 0 to 32767.

        Args:
            speed (int): Default M2 Speed.
            This sets the percentage of the maximum speed set by QPSS as the default speed.
            The range is 0 to 32767.

        Returns:
            bool: Command acknowledgement
        """
        return self._write2(self.address, CMD.SETM2DEFAULTSPEED, speed)

    def ReadDefaultSpeeds(self) -> Sequence[int, int, int]:
        """72 - Read Default Speed Settings

        Read current default speeds for M1 and M2.

        Returns:
            - int: Command Acknowledgement
            - int: Current default M1 speed.
            - int: Current default M2 speed.
        """
        val = self._read4(self.address, CMD.GETDEFAULTSPEEDS)
        if val[0]:
            m1_speed = val[1] >> 16
            m2_speed = val[1] & 0xFFFF
            return (1, m1_speed, m2_speed)
        return (0, 0, 0)

    def SetPinFunctions(self, S3mode: int, S4mode: int, S5mode: int) -> bool:
        """74 - Set S3, S4 and S5 Modes

        Set modes for S3, S4 and S5.

        Mode : S3mode / S4mode / S5mode
        0x00 : Default / Disabled / Disabled
        0x01 : E-Stop / E-Stop / E-Stop
        0x81 : E-Stop(Latching) / E-Stop(Latching) / E-Stop(Latching)
        0x14 : Voltage Clamp / Voltage Clamp / Voltage Clamp
        0x24 : RS485 Direction / - / -
        0x84 : Encoder toggle / - / -
        0x04 : - / Brake / Brake

        Args:
            - S3mode (int): S3 pin mode, in hexadecimal.
            - S4mode (int): S4 pin mode, in hexadecimal.
            - S5mode (int): S5 pin mode, in hexadecimal.

        Returns:
            bool: Command acknowledgement
        """
        return self._write111(
            self.address,
            CMD.SETPINFUNCTIONS,
            S3mode,
            S4mode,
            S5mode,
        )

    def ReadPinFunctions(self) -> Sequence[int, int, int, int]:
        """75 - Get S3, S4 and S5 Modes

        Read mode settings for S3,S4 and S5.

        Returns:
            - int: Command Acknowledgement
            - int: S3 pin mode.
            - int: S4 pin mode.
            - int: S5 pin mode.
        """
        tries = self._tries_timeout
        while True:
            self._sendcommand(self.address, CMD.GETPINFUNCTIONS)
            val1 = self._readbyte()
            if val1[0]:
                val2 = self._readbyte()
                if val1[0]:
                    val3 = self._readbyte()
                    if val1[0]:
                        crc = self._readchecksumword()
                        if crc[0]:
                            if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                                return (0, 0)
                            return (1, val1[1], val2[1], val3[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def SetDeadBand(self, min_dband: int, max_dband: int) -> bool:
        """76 - Set DeadBand for RC/Analog controls

        Deadband : Range of inputs that will return an output of 0

        Set RC/Analog mode control deadband percentage in 10ths of a percent.
        Default value is 25 (2.5%).
        Minimum value is 0 (no DeadBand), Maximum value is 250 (25%).

        Args:
            - min_dband (int): minimum deadband percentage in 10ths of a percent.
            Valid value range is 0-250 (0-25%)
            - max_dband (int): maximum deadband percentage in 10ths of a percent.
            Valid value range is 0-250 (0-25%)

        Returns:
            bool: Command acknowledgement
        """
        return self._write11(self.address, CMD.SETDEADBAND, min_dband, max_dband)

    def GetDeadBand(self) -> Sequence[int, int, int]:
        """77 - Read DeadBand for RC/Analog controls

        Read DeadBand settings in 10ths of a percent.
        Deadband : Range of inputs that will return an output of 0

        Returns:
            - int: Command acknowledgement
            - int: Reverse deadband in 10ths of a percent.
            - int: Forward deadband in 10ths of a percent.
        """
        val = self._read2(self.address, CMD.GETDEADBAND)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def GetEncoderCounters(self) -> Sequence[int, int, int]:
        """78 - Read Encoder Counters

        Read M1 and M2 encoder counters.

        Quadrature encoders have a range of 0 to 4,294,967,295.
        Absolute encoder values are converted from an analog voltage into a value from 0 to 2047 for the full 2V analog range.

        Returns:
            - int: Command acknowledgement
            - int: M1 encoder count.
            - int: M2 encoder count.
        """
        return self._read_n(self.address, CMD.GETENCCOUNTERS, 2)

    def GetISpeedCounters(self) -> Sequence[int, int, int]:
        """79 - Read ISpeeds Counters

        Read M1 and M2 instantaneous speeds.
        Returns the speed in encoder counts per second for the last 300th of a second for both encoder channels.

        Returns:
            - int: Command acknowledgement
            - int: M1 instantaneous speed.
            - int: M2 instantaneous speed.
        """
        return self._read_n(self.address, CMD.GETISPEEDCOUNTERS, 2)

    def RestoreDefaults(self) -> bool:
        """80 - Restore Defaults

        Reset Settings to factory defaults.

        WARNING: In TTL Serial, Baudrate will change if not already set to 38400.
        Communication will be lost

        Returns:
            bool: Command acknowledgement
        """
        return self._write0(self.address, CMD.RESTOREDEFAULTS)

    def ReadDefaultDutyAcceleration(self) -> Sequence[int, int, int]:
        """81 - Read Default Duty Acceleration Settings

        Read M1 and M2 Duty Cycle Acceleration Settings.

        Returns:
            - int: Command acknowledgement
            - int: M1 Duty Cycle Acceleration.
            - int: M2 Duty Cycle Acceleration.
        """
        return self._read_n(self.address, CMD.GETDEFAULTDUTYACCEL, 2)

    def ReadTemp(self) -> Sequence[int, int]:
        """82 - Read Temperature

        Read the board temperature.
        Returned value is in 10ths of degrees.

        Returns:
            - int: Command acknowledgement
            - int: Board temperature.
            Value is returned in 10ths of degrees.
        """
        return self._read2(self.address, CMD.GETTEMP)

    def ReadTemp2(self) -> Sequence[int, int]:
        """83 - Read Temperature 2

        Read the second board temperature (only on supported units).
        Returned value is in 10ths of degrees.

        Returns:
            - int: Command acknowledgement
            - int: Second board temperature.
            Value is returned in 10ths of degrees.
        """
        return self._read2(self.address, CMD.GETTEMP2)

    def ReadError(self) -> Sequence[int, int]:
        """90 - Read Status

        Read the current unit status.

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
            - int: Command acknowledgement
            - int: Status Bit Mask.
        """
        return self._read4(self.address, CMD.GETERROR)

    def ReadEncoderModes(self) -> Sequence[int, int, int]:
        """91 - Read Encoder Mode

        Read the encoder mode for both motors.

        Encoder Mode bits
            Bit 7 : Enable/Disable RC/Analog Encoder support
            Bit 6 : Reverse Encoder Relative Direction
            Bit 5 : Reverse Motor Relative Direction
            Bit 4-1 : N/A
            Bit 0 : Quadrature(0)/Absolute(1)

        Returns:
            - int: Command acknowledgement
            - int: Encoder 1 Mode byte.
            - int: Encoder 2 Mode byte.
        """
        val = self._read2(self.address, CMD.GETENCODERMODE)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def SetM1EncoderMode(self, mode: int) -> bool:
        """92 - Set Motor 1 Encoder Mode

        Set the Encoder Mode for motor 1.

        Encoder Mode bits
            Bit 7 : Enable/Disable RC/Analog Encoder support
            Bit 6 : Reverse Encoder Relative Direction
            Bit 5 : Reverse Motor Relative Direction
            Bit 4-1 : N/A
            Bit 0 : Quadrature(0)/Absolute(1)

        Args:
            mode (int): Encoder 1 Mode Byte.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write1(self.address, CMD.SETM1ENCODERMODE, mode)

    def SetM2EncoderMode(self, mode: int) -> bool:
        """93 - Set Motor 2 Encoder Mode

        Set the Encoder Mode for motor 2.

        Encoder Mode bits
            Bit 7 : Enable/Disable RC/Analog Encoder support
            Bit 6 : Reverse Encoder Relative Direction
            Bit 5 : Reverse Motor Relative Direction
            Bit 4-1 : N/A
            Bit 0 : Quadrature(0)/Absolute(1)

        Args:
            mode (int): Encoder 2 Mode Byte.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write1(self.address, CMD.SETM2ENCODERMODE, mode)

    def WriteNVM(self) -> bool:
        """94 - Write Settings to EEPROM

        Write all active settings to non-volatile memory.
        Values will be loaded after each power up.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write4(self.address, CMD.WRITENVM, 0xE22EAB7A)

    def ReadNVM(self) -> bool:
        """95 - Read Settings from EEPROM

        Restore settings from NVM

        WARNING: In TTL Serial, if baudrate changes or
        the control mode changes, communications will be lost.

        Returns:
            bool: Command acknowledgement
        """
        return self._write0(self.address, CMD.READNVM)

    def SetConfig(self, config: int) -> bool:
        """98 - Set Standard Config Settings

        Set config bits for standard settings.

        WARNING: In TTL Serial, if control mode is changed from packet serial mode
        when setting config, communications will be lost!
        WARNING: In TTL Serial, if baudrate of packet serial mode is changed,
        communications will be lost!

        Config Bit Mask : Function
            0x0000 : RC Mode
            0x0001 : Analog Mode
            0x0002 : Simple Serial Mode
            0x0003 : Packet Serial Mode
            0x0000 : Battery Mode Off
            0x0004 : Battery Mode Auto
            0x0008 : Battery Mode 2 Cell
            0x000C : Battery Mode 3 Cell
            0x0010 : Battery Mode 4 Cell
            0x0014 : Battery Mode 5 Cell
            0x0018 : Battery Mode 6 Cell
            0x001C : Battery Mode 7 Cell
            0x0020 : Mixing
            0x0040 : Exponential
            0x0080 : MCU
            0x0000 : BaudRate 2400
            0x0020 : BaudRate 9600
            0x0040 : BaudRate 19200
            0x0060 : BaudRate 38400
            0x0080 : BaudRate 57600
            0x00A0 : BaudRate 115200
            0x00C0 : BaudRate 230400
            0x00E0 : BaudRate 460800
            0x0100 : FlipSwitch
            0x0000 : Packet Address 0x80
            0x0100 : Packet Address 0x81
            0x0200 : Packet Address 0x82
            0x0300 : Packet Address 0x83
            0x0400 : Packet Address 0x84
            0x0500 : Packet Address 0x85
            0x0600 : Packet Address 0x86
            0x0700 : Packet Address 0x87
            0x0800 : Slave Mode
            0X1000 : Relay Mode
            0x2000 : Swap Encoders
            0x4000 : Swap Buttons
            0x8000 : Multi-Unit Mode

        Args:
            config (int): Config Bit Mask

        Returns:
            bool: Command acknowledgement
        """
        return self._write2(self.address, CMD.SETCONFIG, config)

    def GetConfig(self) -> Sequence[int, int]:
        """99 - Read Standard Config Settings

        Read config bits for standard settings See Command 98.

        Config Bit Mask : Function
            0x0000 : RC Mode
            0x0001 : Analog Mode
            0x0002 : Simple Serial Mode
            0x0003 : Packet Serial Mode
            0x0000 : Battery Mode Off
            0x0004 : Battery Mode Auto
            0x0008 : Battery Mode 2 Cell
            0x000C : Battery Mode 3 Cell
            0x0010 : Battery Mode 4 Cell
            0x0014 : Battery Mode 5 Cell
            0x0018 : Battery Mode 6 Cell
            0x001C : Battery Mode 7 Cell
            0x0020 : Mixing
            0x0040 : Exponential
            0x0080 : MCU
            0x0000 : BaudRate 2400
            0x0020 : BaudRate 9600
            0x0040 : BaudRate 19200
            0x0060 : BaudRate 38400
            0x0080 : BaudRate 57600
            0x00A0 : BaudRate 115200
            0x00C0 : BaudRate 230400
            0x00E0 : BaudRate 460800
            0x0100 : FlipSwitch
            0x0000 : Packet Address 0x80
            0x0100 : Packet Address 0x81
            0x0200 : Packet Address 0x82
            0x0300 : Packet Address 0x83
            0x0400 : Packet Address 0x84
            0x0500 : Packet Address 0x85
            0x0600 : Packet Address 0x86
            0x0700 : Packet Address 0x87
            0x0800 : Slave Mode
            0X1000 : Relay Mode
            0x2000 : Swap Encoders
            0x4000 : Swap Buttons
            0x8000 : Multi-Unit Mode

        Returns:
            - int: Command acknowledgement
            - int: Config byte.
        """
        return self._read2(self.address, CMD.GETCONFIG)

    def SetCTRLModes(self, ctrl1: int, ctrl2: int) -> bool:
        """100 - Set CTRL Modes

        Set CTRL modes of CTRL1 and CTRL2 output pins (available on select models).
        On select models of Roboclaw, two open drain, high current output drivers are available, CTRL1 and CTRL2.

        Mode : Function
            0 : Disable
            1 : User
            2 : Voltage Clamp
            3 : Brake

        ### Modes

            User Mode - The output level can be controlled by setting a value from 0 (0%) to 65535 (100%).
            A variable frequency PWM is generated at the specified percentage.

            Voltage Clamp Mode - The CTRL output will activate when an over voltage
            is detected and released when the overvoltage dissipates.
            Adding an external load dump resistor from the CTRL pin to B+ will allow the Roboclaw to disipate over voltage energy automatically (up to the 3A limit of the CTRL pin).

            Brake Mode - The CTRL pin can be used to activate an external brake (CTRL1 for Motor 1 brake and CTRL2 for Motor 2 brake).
            The signal will activate when the motor is stopped (eg 0 PWM).
            Note acceleration / default acceleration settings should be set appropriately
            to allow the motor to slow down before the brake is activated.

        Args:
            - ctrl1 (int): CTRL1 mode.
            - ctrl2 (int): CTRL2 mode.

        Returns:
            bool: Command acknowledgement
        """
        return self._write11(self.address, CMD.SETCTRLMODES, ctrl1, ctrl2)

    def ReadCTRLModes(self) -> Sequence[int, int, int]:
        """101 - Read CTRL Modes

        Read CTRL modes of CTRL1 and CTRL2 output pins (available on select models).

        Mode : Function
            0 : Disable
            1 : User
            2 : Voltage Clamp
            3 : Brake

        ### Modes

            User Mode - The output level can be controlled by setting a value from 0 (0%) to 65535 (100%).
            A variable frequency PWM is generated at the specified percentage.

            Voltage Clamp Mode - The CTRL output will activate when an over voltage
            is detected and released when the overvoltage dissipates.
            Adding an external load dump resistor from the CTRL pin to B+ will allow the Roboclaw to disipate over voltage energy automatically (up to the 3A limit of the CTRL pin).

            Brake Mode - The CTRL pin can be used to activate an external brake (CTRL1 for Motor 1 brake and CTRL2 for Motor 2 brake).
            The signal will activate when the motor is stopped (eg 0 PWM).
            Note acceleration / default acceleration settings should be set appropriately
            to allow the motor to slow down before the brake is activated.

        Returns:
            - int: Command acknowledgement
            - int: CTRL1 mode
            - int: CTRL2 mode
        """
        val = self._read2(self.address, CMD.READCTRLMODES)
        if val[0]:
            return (1, val[1] >> 8, val[1] & 0xFF)
        return (0, 0, 0)

    def SetCTRL1(self, value: int) -> bool:
        """102 - Set CTRL1

        Set CTRL1 output value (available on select models)

        ### Modes

            User Mode - The output level can be controlled by setting a value from 0 (0%) to 65535 (100%). A variable frequency PWM is generated at the specified percentage.

            Voltage Clamp Mode - The CTRL output will activate when an over voltage is detected and released when the overvoltage dissipates.
            Adding an external load dump resistor from the CTRL pin to B+ will allow the Roboclaw to disipate over voltage energy automatically (up to the 3A limit of the CTRL pin).

            Brake Mode - The CTRL pin can be used to activate an external brake (CTRL1 for Motor 1 brake and CTRL2 for Motor 2 brake).
            The signal will activate when the motor is stopped (eg 0 PWM).
            Note acceleration / default acceleration settings should be set appropriately
            to allow the motor to slow down before the brake is activated.

        Args:
            value (int): CTRL1 value

        Returns:
            bool: Command acknowledgement
        """
        return self._write2(self.address, CMD.SETCTRL1, value)

    def SetCTRL2(self, value: int) -> bool:
        """103 - Set CTRL2

        Set CTRL2 output value (available on select models)

        ### Modes

            User Mode - The output level can be controlled by setting a value from 0 (0%) to 65535 (100%). A variable frequency PWM is generated at the specified percentage.

            Voltage Clamp Mode - The CTRL output will activate when an over voltage is detected and released when the overvoltage dissipates.
            Adding an external load dump resistor from the CTRL pin to B+ will allow the Roboclaw to disipate over voltage energy automatically (up to the 3A limit of the CTRL pin).

            Brake Mode - The CTRL pin can be used to activate an external brake (CTRL1 for Motor 1 brake and CTRL2 for Motor 2 brake).
            The signal will activate when the motor is stopped (eg 0 PWM).
            Note acceleration / default acceleration settings should be set appropriately
            to allow the motor to slow down before the brake is activated.

        Args:
            value (int): CTRL2 value

        Returns:
            bool: Command acknowledgement
        """
        return self._write2(self.address, CMD.SETCTRL2, value)

    def ReadCTRLSettings(self) -> Sequence[int, int, int]:
        """104 - Read CTRL Settings

        Read CTRL1 and CTRL2 output values (available on select models)
        Reads currently set values for CTRL Settings.

        ### Modes

            User Mode - The output level can be controlled by setting a value from 0 (0%) to 65535 (100%). A variable frequency PWM is generated at the specified percentage.

            Voltage Clamp Mode - The CTRL output will activate when an over voltage is detected and released when the overvoltage dissipates.
            Adding an external load dump resistor from the CTRL pin to B+ will allow the Roboclaw to disipate over voltage energy automatically (up to the 3A limit of the CTRL pin).

            Brake Mode - The CTRL pin can be used to activate an external brake (CTRL1 for Motor 1 brake and CTRL2 for Motor 2 brake).
            The signal will activate when the motor is stopped (eg 0 PWM).
            Note acceleration / default acceleration settings should be set appropriately
            to allow the motor to slow down before the brake is activated.

        Returns:
            - int: Command acknowledgement
            - int: CTRL1 value
            - int: CTRL2 value
        """
        val = self._read4(self.address, CMD.READCTRLSETTINGS)
        if val[0]:
            ctrl1_val = val[1] >> 16
            ctrl2_val = val[1] & 0xFFFF
            return (1, ctrl1_val, ctrl2_val)
        return (0, 0, 0)

    def SetM1AutoHomingDutySpeedTimeout(self, percentage: int, timeout: int) -> bool:
        """105 - Set Auto Homing Duty/Speed and Timeout for M1

        Sets the percentage of duty or max speed and the timeout value for automatic homing of motor 1.

        If the motor is set up for velocity or position control, the percentage is defined relative to the maximum speed.
        It is defined as duty percentage otherwise.

        Args:
            - percentage (int): Percentage value.
            Percentage of maximum speed for motors set up in velocity or position control. Percent duty otherwise.
            Range for duty/speed is 0 to 32767.
            - timeout (int): Timeout value in 32 bits.
            Set in increments of 1/300 of a second.
            Ex : Timeout of 10 seconds would be set as 3000.

        Returns:
            bool: Command acknowledgement
        """
        return self._writeS24(self.address, CMD.SETM1AUTOHOMING, percentage, timeout)

    def SetM2AutoHomingDutySpeedTimeout(self, percentage: int, timeout: int) -> bool:
        """106 - Set Auto Homing Duty/Speed and Timeout for M2

        Sets the percentage of duty or max speed and the timeout value for automatic homing of motor 2.

        If the motor is set up for velocity or position control, the percentage is defined relative to the maximum speed.
        It is defined as duty percentage otherwise.

        Args:
            - percentage (int): Percentage value.
            Percentage of maximum speed for motors set up in velocity or position control. Percent duty otherwise.
            Range for duty/speed is 0 to 32767.
            - timeout (int): Timeout value in 32 bits.
            Set in increments of 1/300 of a second.
            Ex : Timeout of 10 seconds would be set as 3000.

        Returns:
            bool: Command acknowledgement
        """
        return self._writeS24(self.address, CMD.SETM2AUTOHOMING, percentage, timeout)

    def ReadAutoHomingDutySpeedTimeout(self) -> Sequence[int, int, int]:
        """107 - Read Auto Homing Duty/Speed and Timeout Settings

        Read the current auto homing duty/speed and timeout settings.

        If the motor is set up for velocity or position control, the percentage is defined relative to the maximum speed.
        It is defined as duty percentage otherwise.

        Returns:
            - int: Command acknowledgement
            - int: Percentage value. Range is 0 to 32767.
            - int: Timeout value in 32 bits, as increments of 1/300 of a second.
            Ex : Value of 3000 equals to a timeout of 10 seconds.
        """
        return self._read24(self.address, CMD.READCTRLSETTINGS)

    def GetMotorAverageSpeeds(self) -> Sequence[int, int, int]:
        """108 - Read Motor Average Speeds

        Read M1 and M2 average speeds.
        Return the speed in encoder counts per second for the last second for both encoder channels.

        Returns:
            - int: Command acknowledgement
            - int: M1 average speed, in encoder counts per second.
            - int: M2 average speed, in encoder counts per second.
        """
        return self._read_n(self.address, CMD.GETMOTORAVESPEEDS, 2)

    def SetSpeedErrorLimits(self, m1_limit: int, m2_limit: int) -> bool:
        """109 - Set Speed Error Limits

        Set motor speed error limits in encoder counts per second.

        Args:
            - m1_limit (int): M1 speed error limit in encoder counts per second.
            - m2_limit (int): M2 speed error limit in encoder counts per second.

        Returns:
            bool: Command acknowledgement
        """
        return self._write44(self.address, CMD.SETSPEEDERRLIMITS, m1_limit, m2_limit)

    def ReadSpeedErrorLimits(self) -> Sequence[int, int, int]:
        """110 - Read Speed Error Limits

        Read the current speed error limit values.

        Returns:
            - int: Command acknowledgement
            - int: M1 speed error limit in encoder counts per second.
            - int: M2 speed error limit in encoder counts per second.
        """
        return self._read_n(self.address, CMD.GETSPEEDERRLIMITS, 2)

    def GetSpeedErrors(self) -> Sequence[int, int, int]:
        """111 - Read Speed Errors

        Read current calculated speed error, in encoder counts per second.

        Returns:
            - int: Command acknowledgement
            - int: M1 speed error, in encoder counts per second.
            - int: M2 speed error, in encoder counts per second.
        """
        return self._read_n(self.address, CMD.GETSPEEDERRORS, 2)

    def SetPositionErrorLimits(self, m1_limit: int, m2_limit: int) -> bool:
        """112 - Set Position Error Limits

        Set motor position error limits in encoder counts.

        Args:
            - m1_limit (int): M1 position error limit, in encoder counts.
            - m2_limit (int): M2 position error limit, in encoder counts.

        Returns:
            bool: Command acknowledgement
        """
        return self._write44(self.address, CMD.SETPOSERRLIMITS, m1_limit, m2_limit)

    def ReadPositionErrorLimits(self) -> Sequence[int, int, int]:
        """113 - Read Position Error Limits

        Read the current motor position error limits.

        Returns:
            - int: Command acknowledgement
            - int: M1 position error limit in encoder counts.
            - int: M2 position error limit in encoder counts.
        """
        return self._read_n(self.address, CMD.GETPOSERRLIMITS, 2)

    def GetPositionErrors(self) -> Sequence[int, int, int]:
        """114 - Read Position Errors

        Read current calculated position error in encoder counts

        Returns:
            - int: Command acknowledgement
            - int: M1 position error, in encoder counts.
            - int: M2 position error, in encoder counts.
        """
        return self._read_n(self.address, CMD.GETPOSERRORS, 2)

    def SetBatteryVoltageOffsets(
        self,
        main_batt: int,
        logic_batt: int,
    ) -> bool:
        """115 - Set Battery Voltage Offsets

        Set the main and logic battery offsets to correct for differences in voltage readings.
        Range of values is +/- 1V in .1V increments with a range of -10 to 10.

        Args:
            - main_batt (int): Main battery offset.
            Range of +/- 1V in .1V increments with a range of -10 to 10.
            - logic_batt (int): Logic battery offset.
            Range of +/- 1V in .1V increments with a range of -10 to 10.

        Returns:
            bool: Command acknowledgement
        """
        return self._write11(
            self.address,
            CMD.SETBATTVOLTOFFSET,
            main_batt,
            logic_batt,
        )

    def ReadBatteryVoltageOffsets(self) -> Sequence[int, int, int]:
        """116 - Read Battery Voltage Offsets

        Read current voltage offset values.
        Range of values is +/- 1V in .1V increments with a range of -10 to 10.

        Returns:
            - int: Command acknowledgement
            - int: Main battery voltage offset.
            - int: Logic battery voltage offset.
        """
        val = self._read2(self.address, CMD.GETBATTVOLTOFFSET)
        if val[0]:
            main_offset = val[1] >> 8
            logic_offset = val[1] & 0xFF
            return (1, main_offset, logic_offset)
        return (0, 0, 0)

    def SetCurrentBlankingPercentage(self, m1_blanking: int, m2_blanking: int) -> bool:
        """117 - Set Current Blanking Percentage

        Sets the percentage of PWM duty for which current readings will be blanked.
        This setting is used to prevent noise from low PWM duty from causing incorrect current readings.

        Args:
            - m1_blanking (int): M1 Current blanking percentage. The range is 0 to 6554 (0 to 20%).
            - m2_blanking (int): M2 Current blanking percentage. The range is 0 to 6554 (0 to 20%).

        Returns:
            bool: Command acknowledgement
        """
        return self._write22(
            self.address, CMD.SETCURRENTBLANKINGPERCENTAGE, m1_blanking, m2_blanking
        )

    def GetCurrentBlankingPercentage(self) -> Sequence[int, int, int]:
        """118 - Read Current Blanking Percentage

        Read the current blanking percentages.

        Returns:
            - int: Command acknowledgement
            - int: M1 Current blanking percentage. The range is 0 to 6554 (0 to 20%).
            - int: M2 Current blanking percentage. The range is 0 to 6554 (0 to 20%).
        """
        val = self._read4(self.address, CMD.GETCURRENTBLANKINGPERCENTAGE)
        if val[0]:
            m1_blanking = val[1] >> 16
            m2_blanking = val[1] & 0xFFFF
            return (1, m1_blanking, m2_blanking)
        return (0, 0, 0)

    def BuffPositionM1(self, position: int, buffer: Literal[0, 1] | None = 0) -> bool:
        """119 - Buffered Drive M1 with Position

        Move M1 from the current position to the specified new position and hold the new position.
        Uses default accel, deccel and speed values.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - position (int): M1 goal position in quadrature pulses
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write41(self.address, CMD.BUF_POSM1, position, buffer)

    def BuffPositionM2(self, position: int, buffer: Literal[0, 1] | None = 0) -> bool:
        """120 - Buffered Drive M2 with Position

        Move M2 from the current position to the specified new position and hold the new position.
        Uses default accel, deccel and speed values.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - position (int): M2 goal position in quadrature pulses
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write41(self.address, CMD.BUF_POSM2, position, buffer)

    def BuffPositionM1M2(
        self,
        m1_position: int,
        m2_position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """121 - Buffered Drive M1/M2 with Position

        Move M1 and M2 from the current positions to the specified new positions and hold the new positions.
        Uses default accel, deccel and speeds values.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - m1_position (int): M1 goal position in quadrature pulses
            - m2_position (int): M2 goal position in quadrature pulses
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write441(
            self.address,
            CMD.BUF_POSM1M2,
            m1_position,
            m2_position,
            buffer,
        )

    def BuffSpeedPositionM1(
        self,
        speed: int,
        position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """122 - Buffered Drive M1 with Speed and Position

        Move M1 from the current position to the specified new position and hold the new position, using the specified speed.
        Uses default accel and deccel values.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - speed (int): Speed in QPPS at which M1 will reach goal position
            - position (int): M1 goal position in quadrature pulses
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write441(
            self.address,
            CMD.BUF_SPEEDPOSM1,
            speed,
            position,
            buffer,
        )

    def BuffSpeedPositionM2(
        self,
        speed: int,
        position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """123 - Buffered Drive M2 with Speed and Position

        Move M2 from the current position to the specified new position and hold the new position, using the specified speed.
        Uses default accel and deccel values.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - speed (int): Speed in QPPS at which M2 will reach goal position
            - position (int): M2 goal position in quadrature pulses
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write441(
            self.address,
            CMD.BUF_SPEEDPOSM2,
            speed,
            position,
            buffer,
        )

    def BuffSpeedPositionM1M2(
        self,
        m1_speed: int,
        m1_position: int,
        m2_speed: int,
        m2_position: int,
        buffer: Literal[0, 1] | None = 0,
    ) -> bool:
        """124 - Buffered Drive M1/M2 with Speed and Position

        Move M1 and M2 from the current position to the specified new position and hold the new position, using the specified speed.
        Uses default accel and deccel values.

        This command is buffered.
        Each motor channel M1 and M2 have separate buffers.
        This command will execute immediately if no other command for that channel is
        executing, otherwise the command will be buffered in the order it was sent.

        Args:
            - m1_speed (int): Speed in QPPS at which M1 will reach goal position
            - m1_position (int): M1 goal position in quadrature pulses
            - m2_speed (int): Speed in QPPS at which M2 will reach goal position
            - m2_position (int): M2 goal position in quadrature pulses
            - buffer ([0,1]): Buffer flag.
            If 0, the command will be buffered and executed in the buffer order.
            If 1, the current command is stopped, the buffer is erased and the new command is run.

        Returns:
            bool: Command Acknowledgement
        """
        return self._write44441(
            self.address,
            CMD.BUF_SPEEDPOSM2,
            m1_speed,
            m1_position,
            m2_speed,
            m2_position,
            buffer,
        )

    def SetM1MaxCurrent(self, max_current: int) -> bool:
        """133 - Set M1 Max Current Limit

        Set Motor 1 Maximum Current Limit.
        Current value is in 10 mA units.
        To calculate, multiply current limit by 100.

        Args:
            max_current (int): M1 maximum current limit, in 10 mA units.
            To set value, multiply current limit by 100.

        Returns:
            bool: Command acknowledgement
        """
        return self._write44(self.address, CMD.SETM1MAXCURRENT, max_current, 0)

    def SetM2MaxCurrent(self, max_current: int) -> bool:
        """134 - Set M2 Max Current Limit

        Set Motor 2 Maximum Current Limit.
        Current value is in 10 mA units.
        To calculate, multiply current limit by 100.

        Args:
            max_current (int): M2 maximum current limit, in 10 mA units.
            To set value, multiply current limit by 100.

        Returns:
            bool: Command acknowledgement
        """
        return self._write44(self.address, CMD.SETM2MAXCURRENT, max_current, 0)

    def ReadM1MaxCurrent(self) -> Sequence[int, int]:
        """135 - Read M1 Max Current Limit

        Read Motor 1 Maximum Current Limit.
        Current value is in 10 mA units.
        To calculate, divide value by 100.
        MinCurrent is always 0.

        Returns:
            - int: Command acknowledgement
            - int: M1 maximum current limit, in 10 mA units. Divide value by 100.
        """
        data = self._read_n(self.address, CMD.GETM1MAXCURRENT, 2)
        if data[0]:
            return (1, data[1])
        return (0, 0)

    def ReadM2MaxCurrent(self) -> Sequence[int, int]:
        """136 - Read M2 Max Current Limit

        Read Motor 2 Maximum Current Limit.
        Current value is in 10 mA units.
        To calculate, divide value by 100.
        MinCurrent is always 0.

        Returns:
            - int: Command acknowledgement
            - int: M2 maximum current limit, in 10 mA units. Divide value by 100.
        """
        data = self._read_n(self.address, CMD.GETM2MAXCURRENT, 2)
        if data[0]:
            return (1, data[1])
        return (0, 0)

    def SetPWMMode(self, pwm_mode: int) -> bool:
        """148 - Set PWM Mode

        Set PWM Drive mode.
        0: Locked Antiphase
        1: Sign Magnitude

        Args:
            pwm_mode (int): PWM Drive mode.
            Locked Antiphase (0) or Sign Magnitude (1).

        Returns:
            bool: Command acknowledgement
        """
        return self._write1(self.address, CMD.SETPWMMODE, pwm_mode)

    def GetPWMMode(self) -> Sequence[int, int]:
        """149 - Read PWM Mode

        Read PWM Drive mode

        Returns:
            - int: Command acknowledgement
            - int: PWM Drive mode.
            Locked Antiphase (0) or Sign Magnitude (1).
        """
        return self._read1(self.address, CMD.GETPWMMODE)

    def ReadEEPROM(self, ee_address: int) -> Sequence[int, int]:
        """252 - Read User EEPROM Memory Location

        Read a 16 bit value from user EEPROM address.
        Memory address range is 0 to 255.

        TYPO IN USER MANUAL : Referenced as 253 in the user manual

        Args:
            ee_address (int): EEPROM Memory Address.
            Memory address range is 0 to 255.

        Returns:
            - int: Command acknowledgement
            - int: 16 bit value from user EEPROM memory address
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(self.address, CMD.READEEPROM)
            self.crc_update(ee_address)
            self._port.write(chr(ee_address))
            val1 = self._readword()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                        return (0, 0)
                    return (1, val1[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def WriteEEPROM(self, ee_address: int, ee_word: int) -> bool:
        """253 - Write User EEPROM Memory Location

        Write a 16 bit value to user EEPROM.
        Memory address range is 0 to 255.

        TYPO IN USER MANUAL : Referenced as 252 in the user manual

        Args:
            - ee_address (int): EEPROM Memory Address.
            Memory address range is 0 to 255.
            - ee_word (int): 16 bit value to write in user EEPROM

        Returns:
            bool: Command acknowledgement
        """
        retval = self._write111(
            self.address, CMD.WRITEEEPROM, ee_address, ee_word >> 8, ee_word & 0xFF
        )
        if retval == True:
            tries = self._tries_timeout
            while True:
                self._port.flushInput()
                val1 = self._readbyte()
                if val1[0]:
                    if val1[1] == 0xAA:
                        return True
                tries -= 1
                if tries == 0:
                    break
        return False
    

    # Private Functions
    def crc_clear(self) -> None:
        """Clear CRC attribute"""
        self._crc = 0

    def crc_update(self, data: int) -> None:
        """Update CRC with data

        This method is based on the C code found in page 59 of the user manual.

        Args:
            data (int): Data to add to the CRC.
        """
        self._crc = self._crc ^ (data << 8)
        for bit in range(8):
            if (self._crc & 0x8000) == 0x8000:
                self._crc = (self._crc << 1) ^ 0x1021
            else:
                self._crc = self._crc << 1

    def _sendcommand(self, address: int, command: int) -> None:
        """Send an address and a command number on the port

        Args:
            - address (int): RoboClaw hex address
            - command (int): Command number
        """
        self.crc_clear()
        self.crc_update(address)
        self._port.write(address.to_bytes(1, "big"))
        self.crc_update(command)
        self._port.write(command.to_bytes(1, "big"))

    def _readchecksumword(self) -> Sequence[int, int]:
        """Read the CRC word value from the port

        This method is used to get CRC to check that read methods
        got the right data from the port.

        Returns:
            - int: Command acknowledgement
            - int: CRC value
        """
        data = self._port.read(2)
        if len(data) == 2:
            crc = (data[0] << 8) | data[1]
            return (1, crc)
        return (0, 0)

    def _readbyte(self) -> Sequence[int, int]:
        """Read a 1-byte value from the port

        Returns:
            - int: Command acknowledgement
            - int: Byte value read from the port
        """
        data = self._port.read(1)
        if len(data):
            val = ord(data)
            self.crc_update(val)
            return (1, val)
        return (0, 0)

    def _readword(self) -> Sequence[int, int]:
        """Read a 2-byte word value from the port

        Returns:
            - int: Command acknowledgement
            - int: 2-byte word read from the port
        """
        val1 = self._readbyte()
        if val1[0]:
            val2 = self._readbyte()
            if val2[0]:
                return (1, val1[1] << 8 | val2[1])
        return (0, 0)

    def _readlong(self) -> Sequence[int, int]:
        """Read a 4-byte long value from the port

        Returns:
            - int: Command acknowledgement
            - int: 4-byte long read from the port
        """
        val1 = self._readbyte()
        if val1[0]:
            val2 = self._readbyte()
            if val2[0]:
                val3 = self._readbyte()
                if val3[0]:
                    val4 = self._readbyte()
                    if val4[0]:
                        return (
                            1,
                            val1[1] << 24 | val2[1] << 16 | val3[1] << 8 | val4[1],
                        )
        return (0, 0)

    def _readslong(self) -> Sequence[int, int]:
        """Read a 4-byte signed long value from the port

        Returns:
            - int: Command acknowledgement
            - int: Signed 4-byte long read from the port
        """
        val = self._readlong()
        if val[0]:
            if val[1] & 0x80000000:
                return (val[0], val[1] - 0x100000000)
            return (val[0], val[1])
        return (0, 0)

    def _writebyte(self, val: int) -> None:
        """Write a 1-byte value on the port

        Args:
            val (int): Byte to write on the port
        """
        self.crc_update(val & 0xFF)
        self._port.write(val.to_bytes(1, "big"))

    def _writesbyte(self, val: int) -> None:
        """Write a signed 1-byte value on the port

        Args:
            val (int): Byte to write on the port
        """
        self._writebyte(val)

    def _writeword(self, val: int) -> None:
        """Write a 2-byte word value on the port

        Args:
            val (int): 2-byte word to write on the port
        """
        self._writebyte((val >> 8) & 0xFF)
        self._writebyte(val & 0xFF)

    def _writesword(self, val: int) -> None:
        """Write a signed 2-byte word value on the port

        Args:
            val (int): 2-byte word to write on the port
        """
        self._writeword(val)

    def _writelong(self, val: int) -> None:
        """Write a 4-byte long value on the port

        Args:
            val (int): 4-byte long to write on the port
        """
        self._writebyte((val >> 24) & 0xFF)
        self._writebyte((val >> 16) & 0xFF)
        self._writebyte((val >> 8) & 0xFF)
        self._writebyte(val & 0xFF)

    def _writeslong(self, val: int) -> None:
        """Write a signed 4-byte long value on the port

        Args:
            val (int): 4-byte long to write on the port
        """
        self._writelong(val)

    def _read1(self, address: int, cmd: int) -> Sequence[int, int]:
        """Read a 1-byte value from the port

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number

        Returns:
            - int: Command acknowledgement
            - int: Byte value
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(address, cmd)
            val1 = self._readbyte()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                        return (0, 0)
                    return (1, val1[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def _read2(self, address: int, cmd: int) -> Sequence[int, int]:
        """Read a 2-byte word value from the port

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number

        Returns:
            - int: Command acknowledgement
            - int: Word value
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(address, cmd)
            val1 = self._readword()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                        return (0, 0)
                    return (1, val1[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def _read4(self, address: int, cmd: int) -> Sequence[int, int]:
        """Read a 4-byte long value from the port

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number

        Returns:
            - int: Command acknowledgement
            - int: Long value
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(address, cmd)
            val1 = self._readlong()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                        return (0, 0)
                    return (1, val1[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def _read41(self, address: int, cmd: int) -> Sequence[int, int, int]:
        """Send a command that reads, in order:
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number

        Returns:
            - int: Command acknowledgement
            - int: 4-byte long that was read with the command
            - int: 1-byte value that was read with the command
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(address, cmd)
            val1 = self._readslong()
            if val1[0]:
                val2 = self._readbyte()
                if val2[0]:
                    crc = self._readchecksumword()
                    if crc[0]:
                        if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                            return (0, 0)
                        return (1, val1[1], val2[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def _read24(self, address: int, cmd: int) -> Sequence[int, int, int]:
        """Send a command that reads, in order:
        - a 2-bytes word
        - a 4-bytes long

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number

        Returns:
            - int: Command acknowledgement
            - int: 2-byte word that was read with the command
            - int: 4-byte long that was read with the command
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            self._sendcommand(address, cmd)
            val1 = self._readword()
            if val1[0]:
                val2 = self._readslong()
                if val2[0]:
                    crc = self._readchecksumword()
                    if crc[0]:
                        if self._crc & 0xFFFF != crc[1] & 0xFFFF:
                            return (0, 0)
                        return (1, val1[1], val2[1])
            tries -= 1
            if tries == 0:
                break
        return (0, 0)

    def _read_n(self, address: int, cmd: int, n_args: int) -> Sequence[int]:
        """Send a command that reads `N` 4-bytes long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - n_args (int): Number of 4-bytes longs to read

        Returns:
            - int: Command acknowledgement
            - `N` * int: `N` 4-byte longs that were read with the command
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            tries -= 1
            if tries == 0:
                break
            failed = False
            self._sendcommand(address, cmd)
            data = [1]
            for i in range(n_args):
                val = self._readlong()
                if val[0] == 0:
                    failed = True
                    break
                data.append(val[1])
            if failed:
                continue
            crc = self._readchecksumword()
            if crc[0]:
                if self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    return data
        return (0, 0, 0, 0, 0)

    def _read_Sn(self, address: int, cmd: int, n_args: int) -> Sequence[int]:
        """Send a command that reads `N` signed 4-bytes long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - n_args (int): Number of signed 4-bytes longs to read

        Returns:
            - int: Command acknowledgement
            - `N` * int: `N` signed 4-byte longs that were read with the command
        """
        tries = self._tries_timeout
        while True:
            self._port.flushInput()
            tries -= 1
            if tries == 0:
                break
            failed = False
            self._sendcommand(address, cmd)
            data = [1]
            for i in range(n_args):
                val = self._readslong()
                if val[0] == 0:
                    failed = True
                    break
                data.append(val[1])
            if failed:
                continue
            crc = self._readchecksumword()
            if crc[0]:
                if self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    return data
        return (0, 0, 0, 0, 0)

    def _writechecksum(self) -> bool:
        """Send the checksum

        Returns:
            bool: Command Acknowledgement
        """
        self._writeword(self._crc & 0xFFFF)
        val = self._readbyte()
        if len(val) > 0 and val[0]:
            return True
        return False

    def _write0(self, address: int, cmd: int) -> bool:
        """Send a command without bytes

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write1(self, address: int, cmd: int, val: int) -> bool:
        """Send a command that writes a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val (int): Value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writebyte(val)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write11(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes two 1-byte values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 1-byte value to write in the command
            - val2 (int): Second 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writebyte(val1)
            self._writebyte(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write111(self, address: int, cmd: int, val1: int, val2: int, val3: int) -> bool:
        """Send a command that writes three 1-byte values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 1-byte value to write in the command
            - val2 (int): Second 1-byte value to write in the command
            - val3 (int): Third 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writebyte(val1)
            self._writebyte(val2)
            self._writebyte(val3)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write2(self, address: int, cmd: int, val: int) -> bool:
        """Send a command that writes a 2-byte word value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val (int): 2-byte word to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writeword(val)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS2(self, address: int, cmd: int, val: int) -> bool:
        """Send a command that writes a 2-byte signed word value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val (int): 2-byte word to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writesword(val)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write22(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes two 2-bytes word values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 2-byte word to write in the command
            - val2 (int): Second 2-byte word to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writeword(val1)
            self._writeword(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS22(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes a signed 2-bytes word and a 2-bytes word

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): Signed 2-byte word to write in the command
            - val2 (int): 2-byte word to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writesword(val1)
            self._writeword(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS2S2(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes two signed 2-bytes word values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First signed 2-byte word to write in the command
            - val2 (int): Second signed 2-byte word to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writesword(val1)
            self._writesword(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS24(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes a signed 2-bytes word and a 4-bytes long

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): Signed 2-byte word to write in the command
            - val2 (int): 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writesword(val1)
            self._writelong(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS24S24(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a signed 2-bytes word
        - a 4-bytes long
        - a signed 2-bytes word
        - a 4-bytes long

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): Signed 2-byte word to write in the command
            - val2 (int): 4-byte long to write in the command
            - val3 (int): Signed 2-byte word to write in the command
            - val4 (int): 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writesword(val1)
            self._writelong(val2)
            self._writesword(val3)
            self._writelong(val4)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4(self, address: int, cmd: int, val: int) -> bool:
        """Send a command that writes a 4-byte long value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val (int): 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS4(self, address: int, cmd: int, val: int) -> bool:
        """Send a command that writes a signed 4-byte long value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val (int): Signed 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writeslong(val)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write44(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes two 4-byte long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Second 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writelong(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4S4(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - a signed 4-bytes long

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): 4-byte long to write in the command
            - val2 (int): Signed 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writeslong(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS4S4(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes two signed 4-bytes long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First signed 4-byte long to write in the command
            - val2 (int): Second signed 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writeslong(val1)
            self._writeslong(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write41(self, address: int, cmd: int, val1: int, val2: int) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): 4-byte long to write in the command
            - val2 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writebyte(val2)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write441(self, address: int, cmd: int, val1: int, val2: int, val3: int) -> bool:
        """Send a command that writes, in order:
        - two 4-bytes long values
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Second 4-byte long to write in the command
            - val3 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writebyte(val3)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS441(self, address: int, cmd: int, val1: int, val2: int, val3: int) -> bool:
        """Send a command that writes, in order:
        - a signed 4-bytes long
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): Signed 4-byte word to write in the command
            - val2 (int): 4-byte long to write in the command
            - val3 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writeslong(val1)
            self._writelong(val2)
            self._writebyte(val3)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4S4S4(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - two signed 4-bytes long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): 4-byte long to write in the command
            - val2 (int): First signed 4-byte long to write in the command
            - val3 (int): Second signed 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writeslong(val3)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4S441(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - a signed 4-bytes long
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Signed 4-byte long to write in the command
            - val3 (int): Second 4-byte long to write in the command
            - val4 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writebyte(val4)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4444(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
    ) -> bool:
        """Send a command that writes four 4-byte long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Second 4-byte long to write in the command
            - val3 (int): Third 4-byte long to write in the command
            - val4 (int): Fourth 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4S44S4(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - a signed 4-bytes long
        - a 4-bytes long
        - a signed 4-bytes long

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): First signed 4-byte long to write in the command
            - val3 (int): Second 4-byte long to write in the command
            - val4 (int): Second signed 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writeslong(val4)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write44441(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
        val5: int,
    ) -> bool:
        """Send a command that writes, in order:
        - four 4-byte long values
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Second 4-byte long to write in the command
            - val3 (int): Third 4-byte long to write in the command
            - val4 (int): Fourth 4-byte long to write in the command
            - val5 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writebyte(val5)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _writeS44S441(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
        val5: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a signed 4-bytes long
        - a 4-bytes long
        - a signed 4-bytes long
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First signed 4-byte long to write in the command
            - val2 (int): First 4-byte long to write in the command
            - val3 (int): Second signed 4-byte long to write in the command
            - val4 (int): Second 4-byte long to write in the command
            - val5 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writeslong(val1)
            self._writelong(val2)
            self._writeslong(val3)
            self._writelong(val4)
            self._writebyte(val5)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4S44S441(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
        val5: int,
        val6: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - a signed 4-bytes long
        - a 4-bytes long
        - a signed 4-bytes long
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): First signed 4-byte long to write in the command
            - val3 (int): Second 4-byte long to write in the command
            - val4 (int): Second signed 4-byte long to write in the command
            - val5 (int): Third 4-byte long to write in the command
            - val6 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writeslong(val4)
            self._writelong(val5)
            self._writebyte(val6)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4S444S441(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
        val5: int,
        val6: int,
        val7: int,
    ) -> bool:
        """Send a command that writes, in order:
        - a 4-bytes long
        - a signed 4-bytes long
        - two 4-bytes long
        - a signed 4-bytes long
        - a 4-bytes long
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): First signed 4-byte long to write in the command
            - val3 (int): Second 4-byte long to write in the command
            - val4 (int): Third 4-byte long to write in the command
            - val5 (int): Second signed 4-byte long to write in the command
            - val6 (int): Fourth 4-byte long to write in the command
            - val7 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writeslong(val5)
            self._writelong(val6)
            self._writebyte(val7)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write4444444(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
        val5: int,
        val6: int,
        val7: int,
    ) -> bool:
        """Send a command that writes seven 4-byte long values

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Second 4-byte long to write in the command
            - val3 (int): Third 4-byte long to write in the command
            - val4 (int): Fourth 4-byte long to write in the command
            - val5 (int): Fifth 4-byte long to write in the command
            - val6 (int): Sixth 4-byte long to write in the command
            - val7 (int): Seventh 4-byte long to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writelong(val5)
            self._writelong(val6)
            self._writelong(val7)
            if self._writechecksum():
                return True
            tries -= 1
        return False

    def _write444444441(
        self,
        address: int,
        cmd: int,
        val1: int,
        val2: int,
        val3: int,
        val4: int,
        val5: int,
        val6: int,
        val7: int,
        val8: int,
        val9: int,
    ) -> bool:
        """Send a command that writes, in order:
        - eight 4-byte long values
        - a 1-byte value

        Args:
            - address (int): RoboClaw hex address
            - cmd (int): Command number
            - val1 (int): First 4-byte long to write in the command
            - val2 (int): Second 4-byte long to write in the command
            - val3 (int): Third 4-byte long to write in the command
            - val4 (int): Fourth 4-byte long to write in the command
            - val5 (int): Fifth 4-byte long to write in the command
            - val6 (int): Sixth 4-byte long to write in the command
            - val7 (int): Seventh 4-byte long to write in the command
            - val8 (int): Eigth 4-byte long to write in the command
            - val9 (int): 1-byte value to write in the command

        Returns:
            bool: Command Acknowledgement
        """
        tries = self._tries_timeout
        while tries:
            self._sendcommand(address, cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writelong(val5)
            self._writelong(val6)
            self._writelong(val7)
            self._writelong(val8)
            self._writebyte(val9)
            if self._writechecksum():
                return True
            tries -= 1
        return False
