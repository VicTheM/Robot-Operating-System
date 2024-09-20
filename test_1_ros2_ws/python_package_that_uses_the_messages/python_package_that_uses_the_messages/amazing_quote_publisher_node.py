import rclpy
from rclpy.node import Node
from package_with_interfaces.msg import AmazingQuote


class AmazingQuotePublisherNode(Node):
    """A ROS2 Node that publishes amazing quotes."""

    def __init__(self):
        super().__init__('amazing_quote_publisher_node')

        self.amazing_quote_publisher = self.create_publisher(
            msg_type=AmazingQuote,
            topic='/quotesToTheWorld',
            qos_profile=1)

        timer_period: float = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.incremental_id: int = 0

    def timer_callback(self):
        """Method that is periodically called by the timer."""

        amazing_quote = AmazingQuote() 			#instantiate a message class that will carry the message
        amazing_quote.id = self.incremental_id
        amazing_quote.quote = 'Hanging around a concept long enough will make you understand it'
        amazing_quote.philosopher_name = 'Victory'

        self.amazing_quote_publisher.publish(amazing_quote)

        self.incremental_id = self.incremental_id + 1


def main(args=None):
    """
    The main function.
    
	Helps us run the node
    """
    try:
        rclpy.init(args=args)

        amazing_quote_publisher_node = AmazingQuotePublisherNode()

        rclpy.spin(amazing_quote_publisher_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
