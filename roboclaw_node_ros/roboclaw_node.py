#!/usr/bin/env python
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64

from .roboclaw_driver import RoboclawDriver
from .electrical_wrapper import ElectricalWrapper
from .encoder_wrapper import EncoderWrapper
from . import utils as u

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?


class Movement:
    """Movement class - responsible of running the RoboClaw and"""

    def __init__(
        self,
        driver,
        max_speed,
        base_width,
        ticks_per_meter,
        ticks_per_rotation,
        parent_clock,
        parent_logger,
    ):
        self.twist = None
        self.driver = driver
        self.max_speed = max_speed
        self.base_width = base_width
        self.ticks_per_meter = ticks_per_meter
        self.ticks_per_rotation = ticks_per_rotation
        self.clock = parent_clock
        self.logger = parent_logger
        self.last_set_speed_time = self.clock.now().nanoseconds
        self.vr_ticks = 0
        self.vl_ticks = 0
        self.stopped = True

    def run(self):
        if self.twist is None:
            return

        if self.twist.linear.x != 0 or self.twist.angular.z != 0:
            self.stopped = False

        linear_x = self.twist.linear.x
        if linear_x > self.max_speed:
            linear_x = self.max_speed
        if linear_x < -self.max_speed:
            linear_x = -self.max_speed

        vr = linear_x + self.twist.angular.z * self.base_width  # m/s
        vl = linear_x - self.twist.angular.z * self.base_width
        self.twist = None

        vr_ticks = int(vr * self.ticks_per_meter)  # ticks/s
        vl_ticks = int(vl * self.ticks_per_meter)

        self.logger.debug("vr_ticks: " + str(vr_ticks) + "vl_ticks: " + str(vl_ticks))

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0

            # if vr_ticks is 0 and vl_ticks is 0:
            #     roboclaw.ForwardM1(self.address, 0)
            #     roboclaw.ForwardM2(self.address, 0)
            # else:
            #     roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)

            if vr_ticks == 0 and vl_ticks == 0:
                self.driver.SpeedM1M2(0, 0)
                self.vr_ticks = 0
                self.vl_ticks = 0
            else:
                gain = 0.5
                self.vr_ticks = gain * vr_ticks + (1 - gain) * self.vr_ticks
                self.vl_ticks = gain * vl_ticks + (1 - gain) * self.vl_ticks
                self.driver.SpeedM1M2(int(self.vr_ticks), int(self.vl_ticks))
        except OSError as e:
            self.logger.warn("SpeedM1M2 OSError: " + str(e.errno))
            self.logger.debug(e)


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

        self.ERRORS = u.ROBOCLAW_ERRORS

        freq = 30
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.run)

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("RoboClaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))

        self.movement = Movement(
            self.driver,
            self.max_speed,
            self.base_width,
            self.ticks_per_meter,
            self.ticks_per_rotation,
            self.get_clock(),
            self.get_logger(),
        )
        self.last_set_speed_time = self.get_clock().now().nanoseconds

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 1
        )
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))


    def read_parameters(self):
        self.dev = self.declare_parameter("dev", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("baud", 115200).value
        self.address = self.declare_parameter("address", 128).value
        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # Movement params
        self.max_speed = self.declare_parameter("max_speed", 2.0).value
        self.ticks_per_meter = self.declare_parameter("ticks_per_meter", 4342.2).value
        self.ticks_per_rotation = self.declare_parameter("ticks_per_rotation", 2780).value
        self.base_width = self.declare_parameter("base_width", 0.315).value
        self.stop_movement = self.declare_parameter("stop_movement", True).value

        # Roboclaw params
        self.P = self.declare_parameter("p_constant", 3.0).value
        self.I = self.declare_parameter("i_constant", 0.42).value
        self.D = self.declare_parameter("d_constant", 0.0).value
        self.qpps = self.declare_parameter("qpps", 6000).value

        # Publishing params
        self.odom_rate = self.declare_parameter("odom_rate", 10).value
        self.elec_rate = self.declare_parameter("elec_rate", 1).value
        self.publish_odom = self.declare_parameter("pub_odom", True).value
        self.publish_encoders = self.declare_parameter("pub_encoders", True).value
        self.publish_elec = self.declare_parameter("pub_elec", True).value
        self.publish_tf = self.declare_parameter("pub_tf", False).value


    def init_device(self, dev_name, address, baud_rate):
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
        try:
            self.driver.SpeedM1M2(0, 0)
            self.driver.ResetEncoders()
        except OSError as e:
            self.get_logger().warn("Device reset OSError: " + str(e.errno))
            self.get_logger().debug(e)

    
    def configure_device(self):
        # Set PID parameters
        self.get_logger().info(f"PID parameters, P: {self.P}, I: {self.I}, D: {self.D}")
        self.driver.SetM1VelocityPID(self.P, self.I, self.D, self.qpps)
        self.driver.SetM2VelocityPID(self.P, self.I, self.D, self.qpps)


    def create_subscribers(self):
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 1)


    def create_wrappers(self):
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
        # self.electrical_wrapper = ElectricalWrapper(self)


    def create_timers(self):
        if self.odom_rate > 0:
            self.odom_timer = self.create_timer(1.0 / self.odom_rate, self.encoder_wrapper.update_and_publish)
        # if self.elec_rate > 0:
        #     self.elec_timer = self.create_timer(1.0 / self.elec_rate, self.electrical_wrapper.publish_elec)


    def run(self):
        # stop movement if robot doesn't receive commands for 1 sec
        if (
            self.stop_movement
            and not self.movement.stopped
            and self.get_clock().now().nanoseconds - self.movement.last_set_speed_time
            > 10e9
        ):
            self.get_logger().info("Did not get command for 1 second, stopping")
            try:
                self.driver.SpeedM1M2(0, 0)
            except OSError as e:
                self.get_logger().error("Could not stop")
                self.get_logger().debug(e)
            self.movement.stopped = True

        # # Update electrical data
        # if self.publish_elec:
        #     try:
        #         elec_data = self.poll_elec()
        #     except ValueError:
        #         pass
        #     except OSError as e:
        #         self.get_logger().warn("Electrical OSError: " + str(e.errno))
        #         self.get_logger().debug(e)

        # publish_time = self.get_clock().now()

        # if self.electrical_wrapper:
        #     self.electrical_wrapper.publish_elec(publish_time, elec_data)

        self.get_logger().info("Update done moving if cmd")
        self.movement.run()


    def poll_elec(self) -> dict:
        """Read motors electrical data"""
        elec_data = {}

        status, *currents = self.driver.ReadCurrents()
        self.log_elec(status, "currents")
        elec_data["current"] = {
            sd: curr / 100 for sd, curr in zip(("left", "right"), currents)
        }

        status, voltage = self.driver.ReadMainBatteryVoltage()
        self.log_elec(status, "voltages")
        elec_data["voltage"] = voltage / 10

        status, logicbatt = self.driver.ReadLogicBatteryVoltage()
        self.log_elec(status, "logic voltages")
        elec_data["logicbatt"] = logicbatt / 10

        status, *pwms = self.driver.ReadPWMs()
        self.log_elec(status, "PWMs")
        elec_data["pwm"] = {
            sd: pwm / 327.67 for sd, pwm in zip(("left", "right"), pwms)
        }

        status, temperature = self.driver.ReadTemp()
        self.log_elec(status, "temperature")
        elec_data["temperature"] = temperature / 10

        return elec_data


    def log_elec(self, status: int, name: str):
        if status:
            self.get_logger().info(f"Got {name.lower()} data")
        else:
            self.get_logger().info(f"Missing {name.lower()}")


    def cmd_vel_callback(self, twist):
        self.movement.last_set_speed_time = self.get_clock().now().nanoseconds
        self.movement.twist = twist


    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = self.driver.ReadError()[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add(
                "Main Batt V:",
                str(float(self.driver.ReadMainBatteryVoltage()[1] / 10)),
            )
            stat.add(
                "Logic Batt V:",
                str(float(self.driver.ReadLogicBatteryVoltage()[1] / 10)),
            )
            stat.add("Temp1 C:", str(float(self.driver.ReadTemp()[1] / 10)))
            stat.add("Temp2 C:", str(float(self.driver.ReadTemp2()[1] / 10)))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
        return stat


    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self, str_msg):
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
