#!/usr/bin/env python
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .roboclaw_driver import RoboclawDriver
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

        self.driver = self.init_device(self.dev, self.address, self.baud)
        self.reset_device()
        self.configure_device()

        self.create_subscribers()
        self.create_wrappers()
        self.create_timers()

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("RoboClaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.diagnostics))

        self.last_cmd_timestamp = self.get_clock().now().nanoseconds


    def read_parameters(self):
        """Read parameters from the parameter server"""

        self.get_logger().info("Reading parameters...")

        # Device params
        self.dev = self.init_parameter("dev", "/dev/ttyACM0")
        self.baud = self.init_parameter("baud", 115200)
        self.address = self.init_parameter("address", 128)
        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # Movement params
        self.max_speed_linear = self.init_parameter("max_speed_linear", 2.0)
        self.max_speed_angular = self.init_parameter("max_speed_angular", 1.0)
        self.stop_if_idle = self.init_parameter("stop_if_idle", True)
        self.idle_timeout = self.init_parameter("idle_timeout", 1.0)

        # Odometry params
        self.ticks_per_meter = self.init_parameter("ticks_per_meter", 4342.2)
        self.ticks_per_rotation = self.init_parameter("ticks_per_rotation", 2780)
        self.base_width = self.init_parameter("base_width", 0.315)

        # Roboclaw params
        self.P = self.init_parameter("p_constant", 3.0)
        self.I = self.init_parameter("i_constant", 0.42)
        self.D = self.init_parameter("d_constant", 0.0)
        self.qpps = self.init_parameter("qpps", 6000)

        # Publishing params
        self.odom_rate = self.init_parameter("odom_rate", 10)
        self.elec_rate = self.init_parameter("elec_rate", 1)
        self.publish_odom = self.init_parameter("pub_odom", True)
        self.publish_encoders = self.init_parameter("pub_encoders", True)
        self.publish_elec = self.init_parameter("pub_elec", True)
        self.publish_tf = self.init_parameter("pub_tf", False)

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

        self.P = self.get_parameter("p_constant").value
        self.I = self.get_parameter("i_constant").value
        self.D = self.get_parameter("d_constant").value
        self.qpps = self.get_parameter("qpps").value
        self.configure_device()     


    def init_device(self, dev_name, address, baud_rate):
        """Initialize the Roboclaw device and connect to it"""

        try:
            self.get_logger().info("Connecting to Roboclaw at " + dev_name + " with address " + str(address))
            driver = RoboclawDriver(dev_name, baud_rate, address)
            self.get_logger().info("Connected!")
        except Exception as e:
            self.get_logger().fatal("Could not connect to Roboclaw at " + dev_name + " with address " + str(address))
            self.get_logger().debug(e)
            self.shutdown("Could not connect to Roboclaw")

        try:
            _, version = driver.ReadVersion()
            self.get_logger().info("Roboclaw version: " + str(version))
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
            self.get_logger().warn("Device reset OSError: " + str(e.errno))
            self.get_logger().debug(e)

    
    def configure_device(self):
        """Configure the Roboclaw device with the ROS parameters"""

        # Set PID parameters
        self.driver.SetM1VelocityPID(self.P, self.I, self.D, self.qpps)
        self.driver.SetM2VelocityPID(self.P, self.I, self.D, self.qpps)


    def create_subscribers(self):
        """Create subscribers for the node"""

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 1)


    def create_wrappers(self):
        """Create the encoder and electrical wrappers"""

        self.encoder_wrapper = EncoderWrapper(
            self,
            self.driver,
            self.base_width,
            self.ticks_per_meter,
            self.ticks_per_rotation,
            self.publish_odom,
            self.publish_encoders,
            self.publish_tf
        )
        self.electrical_wrapper = ElectricalWrapper(
            self,
            self.driver,
            self.publish_elec
        )


    def create_timers(self):
        """Create timers for the node"""

        if self.odom_rate > 0:
            self.odom_timer = self.create_timer(1.0 / self.odom_rate, self.encoder_wrapper.update_and_publish)
        if self.elec_rate > 0:
            self.elec_timer = self.create_timer(1.0 / self.elec_rate, self.electrical_wrapper.update_and_publish)
            
        self.idle_timer = self.create_timer(1 / 30, self.idle_callback)
        self.dynamic_params_timer = self.create_timer(1.0, self.update_parameters)


    def idle_callback(self):
        """Stop the robot if no commands are received for 'idle_timeout' seconds"""

        now = self.get_clock().now().nanoseconds
        if (self.stop_if_idle and now - self.last_cmd_timestamp > self.idle_timeout * 1e9):
            self.get_logger().info(f"Did not get command for {self.idle_timeout} second, stopping")
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
        linear_x = min(max(twist.linear.x, -self.max_speed_linear), self.max_speed_linear)
        angular_z = min(max(twist.angular.z, -self.max_speed_angular), self.max_speed_angular)

        # Convert to motor speeds
        right_speed = int((linear_x + angular_z * self.base_width) * self.ticks_per_meter)  # ticks/s
        left_speed = int((linear_x - angular_z * self.base_width) * self.ticks_per_meter)

        self.get_logger().debug(f"Sending command -> right: {str(right_speed)}, left: {str(left_speed)} (ticks/sec)")

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
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
            return
        state, message = u.ROBOCLAW_ERRORS[status]
        stat.summary(state, message)
        return stat


    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self, str_msg):
        """Shutdown the node"""
        
        self.get_logger().info("Shutting down :" + str_msg)
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
