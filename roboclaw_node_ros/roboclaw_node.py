#!/usr/bin/env python
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from tcr_roboclaw import Roboclaw
from .electrical_wrapper import ElectricalWrapper
from .encoder_wrapper import EncoderWrapper
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
        self.create_wrappers()
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
        if self.ADDRESS > 0x87 or self.ADDRESS < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # Movement params
        self.max_speed_linear = self.init_parameter("max_speed_linear", 2.0)
        self.max_speed_angular = self.init_parameter("max_speed_angular", 1.0)
        self.stop_if_idle = self.init_parameter("stop_if_idle", True)
        self.idle_timeout = self.init_parameter("idle_timeout", 1.0)

        # Odometry params
        self.TICKS_PER_METER = self.init_parameter("ticks_per_meter", 4342.2)
        self.TICKS_PER_ROTATION = self.init_parameter("ticks_per_rotation", 2780)
        self.BASE_WIDTH = self.init_parameter("base_width", 0.315)

        # Roboclaw params
        self.custom_pid = self.init_parameter("custom_pid", True)
        self.P = self.init_parameter("p_constant", 3.0)
        self.I = self.init_parameter("i_constant", 0.42)
        self.D = self.init_parameter("d_constant", 0.0)
        self.qpps = self.init_parameter("qpps", 6000)

        # Publishing params
        self.ODOM_RATE = self.init_parameter("odom_rate", 10)
        self.ELEC_RATE = self.init_parameter("elec_rate", 1)
        self.PUBLISH_ODOM = self.init_parameter("publish_odom", True)
        self.PUBLISH_ENCODERS = self.init_parameter("publish_encoders", True)
        self.PUBLISH_ELEC = self.init_parameter("publish_elec", True)
        self.PUBLISH_TF = self.init_parameter("publish_tf", False)

        # TODO: add a parameter for the acceleration (tried but it didn't work)

    def init_parameter(self, name, default):
        """Initialize a parameter and log it"""

        value = self.declare_parameter(name, default).value
        self.get_logger().info(f"  {name}: {value}")
        return value

    def update_parameters(self):
        """Update dynamic parameters once in a while"""

        self.max_speed_linear = self.get_parameter("max_speed_linear").value
        self.max_speed_angular = self.get_parameter("max_speed_angular").value
        self.stop_if_idle = self.get_parameter("stop_if_idle").value
        self.idle_timeout = self.get_parameter("idle_timeout").value

        self.custom_pid = self.get_parameter("custom_pid").value
        self.P = self.get_parameter("p_constant").value
        self.I = self.get_parameter("i_constant").value
        self.D = self.get_parameter("d_constant").value
        self.qpps = self.get_parameter("qpps").value
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

        try:
            self.driver.SpeedM1M2(0, 0)
            self.driver.ResetEncoders()
        except OSError as e:
            self.get_logger().warn(f"Device reset OSError: {str(e.errno)}")
            self.get_logger().debug(e)

    def configure_device(self):
        """Configure the Roboclaw device with the ROS parameters"""

        # Set PID parameters
        if self.custom_pid:
            self.driver.SetM1VelocityPID(self.P, self.I, self.D, self.qpps)
            self.driver.SetM2VelocityPID(self.P, self.I, self.D, self.qpps)

        # self.driver.SetM1DefaultAccel(655000)
        # self.driver.SetM2DefaultAccel(655000)

    def create_subscribers(self):
        """Create subscribers for the node"""

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            1,
        )

    def create_wrappers(self):
        """Create the encoder and electrical wrappers"""

        self.encoder_wrapper = EncoderWrapper(
            self,
            self.driver,
            self.TICKS_PER_METER,
            self.TICKS_PER_ROTATION,
            self.BASE_WIDTH,
            self.PUBLISH_ODOM,
            self.PUBLISH_ENCODERS,
            self.PUBLISH_TF,
        )
        self.electrical_wrapper = ElectricalWrapper(
            self, self.driver, self.PUBLISH_ELEC
        )

    def create_timers(self):
        """Create timers for the node"""

        if self.ODOM_RATE:
            self.odom_timer = self.create_timer(
                1.0 / self.ODOM_RATE,
                self.encoder_wrapper.update_and_publish,
            )
        if self.ELEC_RATE:
            self.elec_timer = self.create_timer(
                1.0 / self.ELEC_RATE,
                self.electrical_wrapper.update_and_publish,
            )

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
            try:
                self.driver.SpeedM1M2(0, 0)
                self.last_cmd_timestamp = now
            except OSError as e:
                self.get_logger().error("Could not stop")
                self.get_logger().debug(e)

    def cmd_vel_callback(self, twist):
        """Callback for /cmd_vel topic, move the robot according to the Twist message"""

        self.last_cmd_timestamp = self.get_clock().now().nanoseconds

        # Limit the speed
        linear_x = min(
            max(twist.linear.x, -self.max_speed_linear),
            self.max_speed_linear,
        )
        angular_z = min(
            max(twist.angular.z, -self.max_speed_angular),
            self.max_speed_angular,
        )

        # Convert to motor speeds
        right_speed = int(
            (linear_x + angular_z * self.BASE_WIDTH) * self.TICKS_PER_METER
        )  # ticks/s
        left_speed = int(
            (linear_x - angular_z * self.BASE_WIDTH) * self.TICKS_PER_METER
        )

        self.get_logger().debug(
            f"Sending command -> right: {str(right_speed)}, left: {str(left_speed)} (ticks/sec)"
        )

        # Send the command
        try:
            self.driver.SpeedM1M2(right_speed, left_speed)
        except OSError as e:
            self.get_logger().warn("SpeedM1M2 OSError: " + str(e.errno))
            self.get_logger().debug(e)

    def diagnostics(self, stat):
        """Read the error status of the Roboclaw and report it as diagnostics"""

        try:
            status = self.driver.ReadError()[1]
            statuses = u.decipher_rclaw_status(status)
            for state, message in statuses:
                stat.summary(state, message)
            self.get_logger().info("Got unit status: " + str(hex(status)))
        except Exception as e:
            self.get_logger().warn("Diagnostics error: " + str(e))
            self.get_logger().debug(e)
            return
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self, str_msg):
        """Shutdown the node"""

        self.get_logger().info("Shutting down: " + str_msg)
        try:
            self.driver.SpeedM1M2(0, 0)
        except OSError:
            self.get_logger().error("Shutdown did not work trying again")
            try:
                self.driver.SpeedM1M2(0, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().debug(e)


def main(args=None):
    rclpy.init(args=args)

    roboclaw_node = RoboclawNode()
    rclpy.spin(roboclaw_node)
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
