"""
A subscribtion node that subscribes to the topic 'distance'
and uses it's messages to control a motor
"""
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32


class MotorControl(Node):
    """
    This class sunscribes to a message where values
    between 0 and 100 are expected.
    It assumes once the robot is less than 10 meters
    away from target, we have reached our destination
    """

    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
                Int32,
                'distance',
                self.custom_callback,
                10)
        self.lower_treshold = 10

        self.subscription # To prevent unused variable warning


    def custom_callback(self, distance):
        if distance.data < self.lower_treshold:
            self.get_logger().info(f'[{distance.data}]\t\tStop')
        else:
            self.get_logger().info(f'[{distance.data}]\tMove forward')




def main(args=None):
    # initialize
    rclpy.init(args=args)

    # Instantiate
    motor_control_node = MotorControl()

    # Spin
    rclpy.spin(motor_control_node)

    # Destroy
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
   main()
