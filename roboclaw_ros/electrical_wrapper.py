from norlab_custom_interfaces.msg import MotorState
from tcr_roboclaw import Roboclaw
from rclpy.node import Node
from copy import deepcopy


class ElectricalWrapper:

    def __init__(self, node: Node, driver: Roboclaw, pub_elec: bool, swap_motors: bool):
        """Electrical Wrapper

        Args:
            node (rclpy.node.Node): ROS 2 node
            driver (tcr_roboclaw.Roboclaw): RoboClaw driver
            pub_elec (bool): Publish electrical data
            swap_motors (bool): Swap the left and right motors
        """

        # Node parameters
        self.driver = driver
        self.node = node
        self.clock = self.node.get_clock()
        self.logger = self.node.get_logger()
        self.PUB_ELEC = pub_elec
        self.swap_motors = swap_motors

        # Electrical data
        self.timestamp = self.clock.now()
        self.voltage = 0.0
        self.currents = [0.0, 0.0]
        self.temperature = 0.0
        self.duty_cycles = [0.0, 0.0]
        self.poll_electrical_data()

        # Publishers
        self.left_elec_pub = self.node.create_publisher(
            MotorState,
            "/motors/left/electrical",
            1,
        )
        self.right_elec_pub = self.node.create_publisher(
            MotorState,
            "/motors/right/electrical",
            1,
        )

    def update_and_publish(self):
        """Update and publish electrical data"""

        if self.poll_electrical_data():
            if self.PUB_ELEC:
                self.publish_elec()
        else:
            self.logger.warn("Failed to poll electrical data.")

    def poll_electrical_data(self):
        """Poll electrical data from the Roboclaw"""

        status1, status2, status3, status4 = 0, 0, 0, 0

        try:
            self.timestamp = self.clock.now()
            status1, *currents = self.driver.ReadCurrents()
            status2, voltage = self.driver.ReadMainBatteryVoltage()
            status3, *pwms = self.driver.ReadPWMs()
            status4, temp = self.driver.ReadTemp()
        except Exception as e:
            self.logger.warn(f"Read electrical data error: {str(e)}")

        if status1 == 1:
            self.currents = [curr / 100 for curr in currents]
        if status2 == 1:
            self.voltage = voltage / 10
        if status3 == 1:
            self.duty_cycles = [pwm / 327.67 for pwm in pwms]
        if status4 == 1:
            self.temperature = temp / 10

        return status1 and status2 and status3 and status4

    def publish_elec(self):
        """Publish electrical data in two MotorState messages"""

        motor_state = MotorState()
        motor_state.header.stamp = self.timestamp.to_msg()
        motor_state.header.frame_id = "base_link"
        motor_state.voltage = self.voltage
        motor_state.temperature = self.temperature

        motor_state1, motor_state2 = deepcopy(motor_state), deepcopy(motor_state)

        motor_state1.current = self.currents[0]
        motor_state1.duty = self.duty_cycles[0]
        motor_state2.current = self.currents[1]
        motor_state2.duty = self.duty_cycles[1]

        if self.swap_motors:
            self.right_elec_pub.publish(motor_state2)
            self.left_elec_pub.publish(motor_state1)
        else:
            self.right_elec_pub.publish(motor_state1)
            self.left_elec_pub.publish(motor_state2)