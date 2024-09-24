"""
The subscriber
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


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

        # Publisher to control the tortise
        self.publisher_ = self.create_publisher(
                Twist,
                'turtle1/cmd_vel',
                10)
        timer_period = 1        # in secs
        self.timer = self.create_timer(timer_period, self.control)
        self.subscription       # To prevent unused variable warning

    def custom_callback(self, msg):
        x: float = float(msg.data.split()[0])
        y: float = float(msg.data.split()[1])
        self.get_logger().info(f'x: {x - 4.98} | y: {y}')
        
        self.control(x - 4.98, y)

    def control(self, x=0.0, y=0.0, z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        twist = Twist()

        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        
        twist.angular.x = angular_x
        twist.angular.y = angular_y
        twist.angular.z = angular_z

        self.publisher_.publish(twist)



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
