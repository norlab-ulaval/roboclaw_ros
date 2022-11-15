#!/usr/bin/env python
import math
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
from . import roboclaw_driver as roboclaw
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_py
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?


class EncoderOdom:
    def __init__(
        self,
        ticks_per_meter,
        ticks_per_rotation,
        base_width,
        parent_clock,
        parent_odom_pub,
        parent_left_encoder_pub,
        parent_right_encoder_pub,
        parent_logger,
        parent_node,
    ):
        self.TICKS_PER_METER = ticks_per_meter
        self.TICKS_PER_ROTATION = ticks_per_rotation
        self.BASE_WIDTH = base_width
        self.clock = parent_clock
        self.odom_pub = parent_odom_pub
        self.left_encoder_pub = parent_left_encoder_pub
        self.right_encoder_pub = parent_right_encoder_pub
        self.logger = parent_logger
        self.parent_node = parent_node
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = self.clock.now().nanoseconds
        self.vel_theta = 0
        self.left_ang_vel = 0
        self.right_ang_vel = 0

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = self.clock.now().nanoseconds
        d_time = current_time - self.last_enc_time
        self.last_enc_time = current_time

        self.left_ang_vel = 2 * math.pi * left_ticks / (self.TICKS_PER_ROTATION * d_time*1e-9)
        self.right_ang_vel = 2 * math.pi * right_ticks / (self.TICKS_PER_ROTATION * d_time*1e-9)

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 1000:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / (d_time * 1e-9)
            vel_theta = d_theta / (d_time * 1e-9)

        self.vel_theta = vel_theta
        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            self.logger.error(
                "Ignoring left encoder jump: cur "
                + str(enc_left)
                + ", last "
                + str(self.last_enc_left)
            )
        elif abs(enc_right - self.last_enc_right) > 20000:
            self.logger.error(
                "Ignoring right encoder jump: cur "
                + str(enc_right)
                + ", last "
                + str(self.last_enc_right)
            )
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = quaternion_from_euler(0, 0, cur_theta)
        current_time = self.clock.now()

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "odom"
        t.transform.translation.x = cur_x
        t.transform.translation.y = cur_y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, -cur_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #br = tf2_ros.TransformBroadcaster(self.parent_node)
        #br.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        left_enc = Float64()
        left_enc.data = self.left_ang_vel
        right_enc = Float64()
        right_enc.data = self.right_ang_vel

        self.odom_pub.publish(odom)

        self.left_encoder_pub.publish(left_enc)
        self.right_encoder_pub.publish(right_enc)


class Movement:
    def __init__(
        self,
        address,
        max_speed,
        base_width,
        ticks_per_meter,
        ticks_per_rotation,
        parent_clock,
        parent_logger,
    ):
        self.twist = None
        self.address = address
        self.MAX_SPEED = max_speed
        self.BASE_WIDTH = base_width
        self.TICKS_PER_METER = ticks_per_meter
        self.TICKS_PER_ROTATION = ticks_per_rotation
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
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + self.twist.angular.z * self.BASE_WIDTH  # m/s
        vl = linear_x - self.twist.angular.z * self.BASE_WIDTH
        self.twist = None

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        self.logger.debug("vr_ticks: " + str(vr_ticks) + "vl_ticks: " + str(vl_ticks))

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0

            # if vr_ticks is 0 and vl_ticks is 0:
            #     roboclaw.ForwardM1(self.address, 0)
            #     roboclaw.ForwardM2(self.address, 0)
            # else:
            #     roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)

            if vr_ticks == 0 and vl_ticks == 0:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
                self.vr_ticks = 0
                self.vl_ticks = 0
            else:
                gain = 0.5
                self.vr_ticks = gain * vr_ticks + (1 - gain) * self.vr_ticks
                self.vl_ticks = gain * vl_ticks + (1 - gain) * self.vl_ticks
                roboclaw.SpeedM1M2(self.address, int(self.vr_ticks), int(self.vl_ticks))
        except OSError as e:
            self.logger.warn("SpeedM1M2 OSError: " + str(e.errno))
            self.logger.debug(e)


class RoboclawNode(Node):
    def __init__(self):
        super().__init__("roboclaw_node")
        self.ERRORS = {
            0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
            0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
            0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
            0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
            0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
            0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
            0x0020: (
                diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                "Main batt voltage high",
            ),
            0x0040: (
                diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                "Logic batt voltage high",
            ),
            0x0080: (
                diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                "Logic batt voltage low",
            ),
            0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
            0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
            0x0400: (
                diagnostic_msgs.msg.DiagnosticStatus.WARN,
                "Main batt voltage high",
            ),
            0x0800: (
                diagnostic_msgs.msg.DiagnosticStatus.WARN,
                "Main batt voltage low",
            ),
            0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
            0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
            0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
            0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home"),
        }

        freq = 30
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.run)

        self.get_logger().info("Connecting to roboclaw")

        self.declare_parameter("dev", "/dev/ttyACM0")
        dev_name = self.get_parameter("dev").get_parameter_value().string_value
        self.get_logger().info(dev_name)

        self.declare_parameter("baud", 115200)
        baud_rate = self.get_parameter("baud").get_parameter_value().integer_value

        self.declare_parameter("address", 128)
        self.address = self.get_parameter("address").get_parameter_value().integer_value
        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            self.get_logger().fatal("Could not connect to Roboclaw")
            self.get_logger().debug(e)
            self.shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(
            diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals)
        )

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            self.get_logger().warn("Problem getting roboclaw version")
            self.get_logger().warn(e)
            pass

        if not version[0]:
            self.get_logger().warn("Could not get version from roboclaw")
        else:
            self.get_logger().debug(repr(version[1]))

        roboclaw.SpeedM1M2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)

        self.declare_parameter("max_speed", 2.0)
        self.MAX_SPEED = self.get_parameter("max_speed").get_parameter_value().double_value
        self.declare_parameter("ticks_per_meter", 4342.2)
        self.TICKS_PER_METER = self.get_parameter("ticks_per_meter").get_parameter_value().double_value
        self.declare_parameter("ticks_per_rotation", 2780)
        self.TICKS_PER_ROTATION = self.get_parameter("ticks_per_rotation").get_parameter_value().integer_value
        self.declare_parameter("base_width", 0.315)
        self.BASE_WIDTH = self.get_parameter("base_width").get_parameter_value().double_value
        self.declare_parameter("pub_odom", True)
        self.PUB_ODOM = self.get_parameter("pub_odom").get_parameter_value().bool_value
        self.declare_parameter("stop_movement", True)
        self.STOP_MOVEMENT = self.get_parameter("stop_movement").get_parameter_value().bool_value

        self.encodm = None
        if self.PUB_ODOM:
            self.odom_pub = self.create_publisher(Odometry, "/odom_roboclaw", 1)
            self.left_encoder_pub = self.create_publisher(Float64, "/left_encoder_angular_velocity", 1)
            self.right_encoder_pub = self.create_publisher(Float64, "/right_encoder_angular_velocity", 1)
            self.encodm = EncoderOdom(
                self.TICKS_PER_METER,
                self.TICKS_PER_ROTATION,
                self.BASE_WIDTH,
                self.get_clock(),
                self.odom_pub,
                self.left_encoder_pub,
                self.right_encoder_pub,
                self.get_logger(),
                self,
            )
        self.movement = Movement(
            self.address,
            self.MAX_SPEED,
            self.BASE_WIDTH,
            self.TICKS_PER_METER,
            self.TICKS_PER_ROTATION,
            self.get_clock(),
            self.get_logger(),
        )
        self.last_set_speed_time = self.get_clock().now().nanoseconds

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 1
        )
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))


    def run(self):
        # stop movement if robot doesn't recieve commands for 1 sec
        if (
            self.STOP_MOVEMENT
            and not self.movement.stopped
            and self.get_clock().now().nanoseconds - self.movement.last_set_speed_time
            > 10e9
        ):
            self.get_logger().info("Did not get command for 1 second, stopping")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.get_logger().error("Could not stop")
                self.get_logger().debug(e)
            self.movement.stopped = True

        # TODO need find solution to the OSError11 looks like sync problem with serial
        status1, enc1, crc1 = None, None, None
        status2, enc2, crc2 = None, None, None

        try:
            status1, enc1, crc1 = roboclaw.ReadEncM1(self.address)
        except ValueError:
            pass
        except OSError as e:
            self.get_logger().warn("ReadEncM1 OSError: " + str(e.errno))
            self.get_logger().debug(e)

        try:
            status2, enc2, crc2 = roboclaw.ReadEncM2(self.address)
        except ValueError:
            pass
        except OSError as e:
            self.get_logger().warn("ReadEncM2 OSError: " + str(e.errno))
            self.get_logger().debug(e)
        if ("enc1" in vars()) and ("enc2" in vars() and enc1 and enc2):
            self.get_logger().debug(" Encoders " + str(enc1) + " " + str(enc2))
            if self.encodm:
                self.encodm.update_publish(enc1, enc2)
            self.updater.update()
        self.get_logger().info("Update done moving if cmd")
        self.movement.run()

    def cmd_vel_callback(self, twist):
        self.movement.last_set_speed_time = self.get_clock().now().nanoseconds
        self.movement.twist = twist

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add(
                "Main Batt V:",
                str(float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10)),
            )
            stat.add(
                "Logic Batt V:",
                str(float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10)),
            )
            stat.add("Temp1 C:", str(float(roboclaw.ReadTemp(self.address)[1] / 10)))
            stat.add("Temp2 C:", str(float(roboclaw.ReadTemp2(self.address)[1] / 10)))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self, str_msg):
        self.get_logger().info("Shutting down :" + str_msg)
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            self.get_logger().error("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().debug(e)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, tb):
        self.shutdown(str(exc_value))
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    roboclaw_node = RoboclawNode()
    rclpy.spin(roboclaw_node)
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
