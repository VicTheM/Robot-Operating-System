import rclpy
from rclpy.node import Node

class TalkForever(Node):
    """A node is a class that inherits the Node class from rclpy"""

    def __init__(self):
        super().__init__("Node_Name")
        timer_period: float = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.print_count: int = 0

    def timer_callback(self):
        """The timer keeps calling this method"""
        self.get_logger().info(f'I am talking for the {self.print_count} (st/nd/th) time')
        self.print_count = self.print_count + 1

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        talk_forever_nodee = TalkForever()

        rclpy.spin(talk_forever_nodee)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
