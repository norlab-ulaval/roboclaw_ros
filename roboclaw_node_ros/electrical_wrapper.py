from rclpy.time import Time
from sensor_msgs.msg import BatteryState


class ElectricalWrapper:

    def __init__(
        self,
        node,
        driver,
        pub_elec
    ):
        """Electrical Wrapper

        Args:
            node (rclpy.node.Node): ROS2 node
            driver (roboclaw_driver.Roboclaw): Roboclaw driver
            pub_elec (bool): Publish electrical data
        """

        # Node parameters
        self.driver = driver
        self.node = node
        self.clock = self.node.get_clock()
        self.logger = self.node.get_logger()
        self.PUB_ELEC = pub_elec

        # Electrical data
        self.timestamp = self.clock.now()
        self.voltage = 0.0
        self.currents = [0.0, 0.0]
        self.temperature = 0.0
        self.duty_cycles = [0.0, 0.0]
        self.poll_electrical_data()

        # Publishers
        self.left_elec_pub = self.node.create_publisher(BatteryState, "/motors/left/electrical", 1)
        self.right_elec_pub = self.node.create_publisher(BatteryState, "/motors/right/electrical", 1)


    def update_and_publish(self):
        """Update and publish electrical data"""

        if self.poll_electrical_data():
            if self.PUB_ELEC: self.publish_elec()
        else:
            self.logger.warn("Failed to poll electrical data.")


    def poll_electrical_data(self):
        """Poll electrical data from the Roboclaw"""

        try:
            self.timestamp = self.clock.now()
            status1, *currents = self.driver.ReadCurrents()
            status2, voltage = self.driver.ReadMainBatteryVoltage()
            status3, *pwms = self.driver.ReadPWMs()
            status4, temp = self.driver.ReadTemp()
        except Exception as e:
            self.logger.warn("Read electrical data error: " + str(e.errno))
            self.logger.debug(e)

        if status1 == 1:
            self.currents = [curr / 100 for curr in currents]
        if status2 == 1:
            self.voltage = voltage / 10
        if status3 == 1:
            self.duty_cycles = [pwm / 327.67 for pwm in pwms]
        if status4 == 1:
            self.temperature = temp / 10

        return status1 == 1 and status2 == 1 and status3 == 1 and status4 == 1


    def publish_elec(self):
        """Publish electrical data in two battery states messages

        Main Battery Voltage => voltage
        Current => current
        PWM => percentage
        Temperature => temperature
        """

        bs = BatteryState()
        bs.header.stamp = self.timestamp.to_msg()
        bs.header.frame_id = "base_link"
        bs.voltage = self.voltage
        bs.temperature = self.temperature 

        bs.current = self.currents[0]
        bs.percentage = self.duty_cycles[0]
        self.right_elec_pub.publish(bs)

        bs.current = self.currents[1]
        bs.percentage = self.duty_cycles[1]
        self.left_elec_pub.publish(bs)
