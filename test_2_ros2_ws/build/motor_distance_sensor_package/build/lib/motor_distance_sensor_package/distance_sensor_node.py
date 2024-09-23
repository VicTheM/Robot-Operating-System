"""
This file houses a ROS2 node that publishes fake distance
sensor data as a message over a topic
"""
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32        # The message is a 32-bit interger
import random


class DistanceSensor(Node):
    """
    This class has one publisher, it publishes distance in meters
    to the Int32 message through the topic 'distance'

    It is assumed the Robot is x distance away from a destination
    where x is an integer between 0 and 100
    """
    
    def __init__(self):
        super().__init__('distance_sensor_node')
        self.publisher_ = self.create_publisher(Int32, 'distance', 10)
        timer_period = 1                                    # In seconds
        self.timer = self.create_timer(timer_period, self.get_distance)


    def get_distance(self):
        """
        Generate pseudo-random integers between 0 and 100
        and publishes to a message over a topic
        """

        distance = Int32()          # Instantiate the message bag (type)
        distance.data = random.randint(0, 100)

        # publish the distance
        self.publisher_.publish(distance)

        # spit it on the terminal
        self.get_logger().info(f"Distance: [{distance.data}] meters")


# THE MAIN FUNCTION (ENTRY POINT)
def main(args=None):
    # initialize
    rclpy.init(args=args)

    # instantiate
    distance_sensor_node = DistanceSensor()

    # spin into an infinite loop
    rclpy.spin(distance_sensor_node)

    # shutdown gracefully
    distance_sensor_node.destroy_node()

    # close
    rclpy.shutdown()


if __name__ == '__main__':
    main()
