"""
This file houses the position encoder node. instead of publishing 
the two co-ordnate position as floats, I will use string so I can 
use the std_msgs interface. therefore the nodes will handle the
conversion of string to float by themselves

    This will be the format of the string

    "<x-cordinate y-cordinate>"

    as seen, they are space delimeted
"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import math

# And we create the node here
class PositionEncoder(Node):
    """
    This class uses a sinusiodal function to generate
    position co-ordinates that can be mapped and plotted
    as a sine graph.it is assumed the points it publishes
    were read from a position sensor that has been placed
    on a robot that moves in a sinusodial path
    """
    
    def __init__(self):
        super().__init__('position_encoder_node')
        self.publisher_ = self.create_publisher(String, 'planar_position', 10)
        timer_period = 1            # In seconds
        self.num_of_points = 0
        self.x = 0                  # In degrees
        self.timer = self.create_timer(timer_period, self.get_position)

    def get_position(self):
        """
        This function generates a planar position co-ordinate that simulates
        a sinusodial movement
        """

        y = math.sin(math.radians(self.x))
        point = String()
        point.data = f"{self.x} {y:.2f}"
        self.x += 5
        self.num_of_points += 1

        # publish the point
        self.publisher_.publish(point)
        # spit it on the terminal
        self.get_logger().info(f"\tx:\t{self.x},\t y:\t{y:.2f}")




# THE MAIN FUNCTION (ENTRY POINT)
def main(args=None):
    # initialize
    rclpy.init(args=args)

    # instantiate
    position_encoder_node = PositionEncoder()

    # spin into an infinite loop
    rclpy.spin(position_encoder_node)

    # shutdown gracefully
    position_encoder_node.destroy_node()

    # close
    rclpy.shutdown()


if __name__ == '__main__':
    main()
