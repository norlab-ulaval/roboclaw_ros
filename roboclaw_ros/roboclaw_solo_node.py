#!/usr/bin/env python
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from tcr_roboclaw import Roboclaw
from . import utils as u

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class RoboclawNode(Node):
    """RoboClaw Node"""

    def __init__(self):
        super().__init__("roboclaw_node")

        self.read_parameters()

        self.driver = self.init_device(self.DEV, self.ADDRESS, self.BAUD)
        self.reset_device()
        self.configure_device()

        self.create_subscribers()
        self.create_timers()

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("RoboClaw")
        self.updater.add(
            diagnostic_updater.FunctionDiagnosticTask(
                "Vitals",
                self.diagnostics,
            )
        )

        self.last_cmd_timestamp = self.get_clock().now().nanoseconds

    def read_parameters(self):
        """Read parameters from the parameter server"""

        self.get_logger().info("Reading parameters...")

        # Device params
        self.DEV = self.init_parameter("dev", "/dev/ttyACM0")
        self.BAUD = self.init_parameter("baud", 115200)
        self.ADDRESS = self.init_parameter("address", 128)
        self.serial_timeout = self.init_parameter("serial_timeout", 0.0)    
        if self.ADDRESS > 0x87 or self.ADDRESS < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # Odometry params
        self.TICKS_PER_METER = self.init_parameter("ticks_per_meter", 4342.2)
        self.TICKS_PER_ROTATION = self.init_parameter("ticks_per_rotation", 2780)

        # Movement params
        self.control_type = self.init_parameter("control_type", "pwm")
        self.max_speed = self.init_parameter("max_speed", 2.0)
        self.accel = int(self.init_parameter("acceleration", 1.0) * self.TICKS_PER_METER)
        self.stop_if_idle = self.init_parameter("stop_if_idle", True)
        self.idle_timeout = self.init_parameter("idle_timeout", 1.0)

        # Roboclaw params
        self.custom_pid = self.init_parameter("custom_pid", True)
        self.P = self.init_parameter("p_constant", 3.0)
        self.I = self.init_parameter("i_constant", 0.42)
        self.D = self.init_parameter("d_constant", 0.0)
        self.qpps = self.init_parameter("qpps", 6000)

        # Publishing params
        self.ELEC_RATE = self.init_parameter("elec_rate", 1)
        self.PUBLISH_ENCODERS = self.init_parameter("publish_encoders", True)
        self.PUBLISH_ELEC = self.init_parameter("publish_elec", True)

        # IO params
        self.S3Mode = self.init_parameter("S3_mode", 0x00)
        self.S4Mode = self.init_parameter("S4_mode", 0x00)
        self.S5Mode = self.init_parameter("S5_mode", 0x00)

    def init_parameter(self, name, default):
        """Initialize a parameter and log it"""

        value = self.declare_parameter(name, default).value
        self.get_logger().info(f"  {name}: {value}")
        return value

    def update_parameters(self):
        """Update dynamic parameters once in a while"""

        # self.control_type = self.init_parameter("control_type", "pwm")
        # self.max_speed = self.get_parameter("max_speed").value
        self.stop_if_idle = self.get_parameter("stop_if_idle").value
        self.idle_timeout = self.get_parameter("idle_timeout").value

        # self.custom_pid = self.get_parameter("custom_pid").value
        # self.P = self.get_parameter("p_constant").value
        # self.I = self.get_parameter("i_constant").value
        # self.D = self.get_parameter("d_constant").value
        # self.qpps = self.get_parameter("qpps").value
        self.configure_device()

    def init_device(self, dev_name: str, address: int, baud_rate: int):
        """Initialize the Roboclaw device and connect to it"""

        try:
            self.get_logger().info(
                f"Connecting to device {dev_name} with address {str(address)}"
            )
            driver = Roboclaw(dev_name, baud_rate, address)
            driver.open()
            self.get_logger().info("Connected!")
        except Exception as e:
            self.get_logger().fatal("Could not connect to device.")
            self.get_logger().debug(e)
            self.shutdown("Could not connect to Roboclaw")

        try:
            _, version = driver.ReadVersion()
            self.get_logger().info(f"Roboclaw version: {str(version)}")
        except Exception as e:
            self.get_logger().warn("Problem getting Roboclaw version")
            self.get_logger().debug(e)

        return driver

    def reset_device(self):
        """Reset the roboclaw speed and encoder counters"""

        self.stop_motors()
        try:
            self.driver.ResetEncoders()
        except OSError as e:
            self.get_logger().warn(f"ResetEncoders OSError: {str(e.errno)}")
            self.get_logger().debug(e)

    def configure_device(self):
        """Configure the Roboclaw device with the ROS parameters"""

        # Set the serial timeout
        self.driver.SetSerialTimeout(int(self.serial_timeout * 10))

        # Set PID parameters
        # if self.custom_pid:
        #     self.driver.SetM1VelocityPID(self.P, self.I, self.D, self.qpps)

        # Set S3, S4, S5 modes
        self.driver.SetPinFunctions(self.S3Mode, self.S4Mode, self.S5Mode)

    def create_subscribers(self):
        """Create subscribers for the node"""

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/velocity",
            self.velocity_callback,
            1,
        )

    def create_timers(self):
        """Create timers for the node"""

        # if self.ODOM_RATE:
        #     self.odom_timer = self.create_timer(
        #         1.0 / self.ODOM_RATE,
        #         self.encoder_wrapper.update_and_publish,
        #     )
        # if self.ELEC_RATE:
        #     self.elec_timer = self.create_timer(
        #         1.0 / self.ELEC_RATE,
        #         self.electrical_wrapper.update_and_publish,
        #     )

        self.idle_timer = self.create_timer(1 / 30, self.idle_callback)
        self.dynamic_params_timer = self.create_timer(1.0, self.update_parameters)

    def idle_callback(self):
        """Stop the robot if no commands are received for 'idle_timeout' seconds"""

        now = self.get_clock().now().nanoseconds
        if (
            self.stop_if_idle
            and now - self.last_cmd_timestamp > self.idle_timeout * 1e9
        ):
            self.get_logger().info(
                f"Did not get command for {self.idle_timeout} second, stopping"
            )
            self.stop_motors()

    def velocity_callback(self, vel_msg):
        """Callback for velocity topic"""

        ## Assuming PWM mode for now
        # Velocity message value is between -100 and 100% 
        # The duty value is signed and the range is -32767 to +32767
        duty = int(vel_msg.data / 100 * 32767)

        self.get_logger().debug(
            f"Sending command -> {str(duty)} %"
        )

        # Send the command
        try:
            self.driver.DutyM1(duty)
            self.last_cmd_timestamp = self.get_clock().now().nanoseconds
        except OSError as e:
            self.get_logger().warn("DutyM1 OSError: " + str(e.errno))
            self.get_logger().debug(e)

    def diagnostics(self, stat):
        """Read the error status of the Roboclaw and report it as diagnostics"""

        try:
            status = self.driver.ReadError()[1]
            statuses = u.decipher_rclaw_status(status)
            for state, message in statuses:
                stat.summary(state, message)
            self.get_logger().debug("Got unit status: " + str(hex(status)))
        except Exception as e:
            self.get_logger().warn("Diagnostics error: " + str(e))
            self.get_logger().debug(e)
            return
        return stat
    
    def stop_motors(self):
        """Stop the motors"""

        try:
            self.driver.DutyM1(0)
            self.last_cmd_timestamp = self.get_clock().now().nanoseconds
        except OSError as e:
            self.get_logger().error("Could not stop motors")
            self.get_logger().debug(e)

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self, str_msg):
        """Shutdown the node"""

        self.get_logger().info("Shutting down: " + str_msg)
        self.stop_motors()
        self.stop_motors()  # Call twice to make sure


def main(args=None):
    rclpy.init(args=args)

    roboclaw_node = RoboclawNode()
    rclpy.spin(roboclaw_node)
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
