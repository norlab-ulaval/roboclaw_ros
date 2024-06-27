from math import cos, pi, sin, fmod

from tcr_roboclaw import Roboclaw
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from norlab_custom_interfaces.msg import EncoderState
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

COUNTER_MAX = 2**32


class EncoderWrapper:

    def __init__(
        self,
        node: Node,
        driver: Roboclaw,
        ticks_per_meter: float,
        ticks_per_rotation: float,
        base_width: float,
        pub_odom: bool,
        pub_encoders: bool,
        pub_tf: bool,
    ):
        """Encoder Wrapper

        Args:
            node (rclpy.node.Node): ROS 2 node
            driver (tcr_roboclaw.Roboclaw): Roboclaw driver
            ticks_per_meter (float): Encoder ticks per meter
            ticks_per_rotation (float): Encoder ticks per rotation
            base_width (float): Distance between the two wheels
            pub_odom (bool): Publish odometry data
            pub_encoders (bool): Publish encoder data
            pub_tf (bool): Publish the transform from odom to base_link
        """
        self.TICKS_PER_METER = ticks_per_meter
        self.TICKS_PER_ROTATION = ticks_per_rotation
        self.BASE_WIDTH = base_width
        self.driver = driver

        # Node parameters
        self.node = node
        self.clock = self.node.get_clock()
        self.logger = self.node.get_logger()
        self.PUB_ODOM = pub_odom
        self.PUB_ENCODERS = pub_encoders
        self.PUB_TF = pub_tf

        # Encoder data
        self.timestamp = self.clock.now()
        self.left_ticks = [0, 0]
        self.right_ticks = [0, 0]
        self.left_velocity = 0
        self.right_velocity = 0
        self.poll_encoders()

        # Odometry data
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0.0
        self.vel_x = 0.0
        self.vel_theta = 0.0

        # Publishers
        # TODO: Topics should be parameters
        self.left_encoder_pub = self.node.create_publisher(
            EncoderState,
            "/motors/left/encoder",
            10,
        )
        self.right_encoder_pub = self.node.create_publisher(
            EncoderState,
            "/motors/right/encoder",
            10,
        )
        self.odom_pub = self.node.create_publisher(Odometry, "/motors/odometry", 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)

    @staticmethod
    def normalize_angle(angle):
        return fmod(angle + pi, 2.0 * pi) - pi

    def update_and_publish(self):
        """Update the odometry and publish the data"""

        if self.poll_encoders():
            self.update_odometry()
            if self.PUB_ODOM:
                self.publish_odometry()
            if self.PUB_ENCODERS:
                self.publish_encoder_data()
            if self.PUB_TF:
                self.broadcast_tf()
        else:
            self.logger.warn("Failed to poll encoders.")

    def poll_encoders(self):
        """Poll the encoder data from the hardware"""

        try:
            self.timestamp = self.clock.now()
            status1, ticks1, ticks2 = self.driver.GetEncoderCounters()
            status2, speed1, speed2 = self.driver.GetMotorAverageSpeeds()
        except Exception as e:
            self.logger.warn("Read encoders error: " + str(e.errno))
            self.logger.debug(e)

        # Left motor encoder : M2 / Right motor encoder : M1
        if status1 == 1:
            self.right_ticks = [self.right_ticks[1], ticks1]
            self.left_ticks = [self.left_ticks[1], ticks2]
        if status2 == 1:
            self.right_velocity = speed1
            self.left_velocity = speed2

        return status1 and status2

    def update_odometry(self):
        """Update the odometry estimation"""

        # Compute the ticks difference (handle overflow)
        delta_ticks_left = self.left_ticks[1] - self.left_ticks[0]
        delta_ticks_right = self.right_ticks[1] - self.right_ticks[0]

        # Handle overflow and underflow
        if delta_ticks_left < -COUNTER_MAX / 2:
            delta_ticks_left += COUNTER_MAX
        elif delta_ticks_left > COUNTER_MAX / 2:
            delta_ticks_left -= COUNTER_MAX
        if delta_ticks_right < -COUNTER_MAX / 2:
            delta_ticks_right += COUNTER_MAX
        elif delta_ticks_right > COUNTER_MAX / 2:
            delta_ticks_right -= COUNTER_MAX

        # Compute the traveled distance
        dist_left = delta_ticks_left / self.TICKS_PER_METER
        dist_right = delta_ticks_right / self.TICKS_PER_METER
        dist_center = (dist_right + dist_left) / 2.0

        # Compute the displacement
        d_theta = (dist_right - dist_left) / self.BASE_WIDTH
        d_x = dist_center * cos(self.pose_theta + d_theta / 2.0)
        d_y = dist_center * sin(self.pose_theta + d_theta / 2.0)

        # Update the pose
        self.pose_x += d_x
        self.pose_y += d_y
        self.pose_theta = self.normalize_angle(self.pose_theta + d_theta)

        # Compute the linear and angular velocities
        vel_left = self.left_velocity / self.TICKS_PER_METER
        vel_right = self.right_velocity / self.TICKS_PER_METER
        self.vel_x = (vel_left + vel_right) / 2.0
        self.vel_theta = (vel_right - vel_left) / self.BASE_WIDTH

    def publish_odometry(self):
        """Publish odometry data to the ROS network"""

        odom = Odometry()
        odom.header.stamp = self.timestamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, self.pose_theta)
        odom.pose.pose.orientation = Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3],
        )

        odom.twist.twist.linear.x = self.vel_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vel_theta

        self.odom_pub.publish(odom)

    def publish_encoder_data(self):
        """Publish the encoder data to the ROS network"""

        encoder_state = EncoderState()
        encoder_state.header.stamp = self.timestamp.to_msg()
        encoder_state.header.frame_id = "base_link"
        encoder_state.ticks_per_rotation = float(self.TICKS_PER_ROTATION)
        encoder_state.ticks_per_meter = float(self.TICKS_PER_METER)

        encoder_state.position = self.left_ticks[1]
        encoder_state.velocity = float(self.left_velocity)
        self.left_encoder_pub.publish(encoder_state)

        encoder_state.position = self.right_ticks[1]
        encoder_state.velocity = float(self.right_velocity)
        self.right_encoder_pub.publish(encoder_state)

    def broadcast_tf(self):
        """Broadcast the transform from odom to base_link"""

        t = TransformStamped()
        t.header.stamp = self.timestamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, self.pose_theta)
        t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.tf_broadcaster.sendTransform(t)
