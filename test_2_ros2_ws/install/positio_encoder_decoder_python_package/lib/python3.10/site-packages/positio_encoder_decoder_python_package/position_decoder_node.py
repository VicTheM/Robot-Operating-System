"""
The subscriber
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PositionDecoder(Node):
    """
    This class receives co-ordinates of a planar position
    as a string of this form "x-value y-value", and seperate
    them into floats (in the future it would use it to control
    the turtle in turtlesim
    """

    def __init__(self):
        super().__init__('position_decoder_node')
        self.subscription = self.create_subscription(
                String,
                'planar_position',
                self.custom_callback,
                10)
        self.subscription       # To prevent unused variable warning

    def custom_callback(self, msg):
        x: float = float(msg.data.split()[0])
        y: float = float(msg.data.split()[1])
        self.get_logger().info(f'x: {x} | y: {y}')



def main(args=None):
    # initialize
    rclpy.init(args=args)

    # Instantiate
    position_decoder = PositionDecoder()

    # Spin
    rclpy.spin(position_decoder)

    # Destroy
    position_decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
