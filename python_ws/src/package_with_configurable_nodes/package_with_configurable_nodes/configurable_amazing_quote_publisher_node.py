"""This file houses a node that uses RPS2 parameters"""
import rclpy
from rclpy.node import Node
from package_with_interfaces.msg import AmazingQuote


class AmazingQuoteConfigurablePublisherNode(Node):
    """A configurable ROS2 Node that publishes a configurable amazing quote."""

    def __init__(self):
        super().__init__('publisher_node')

        # Periodically-obtained parameters
		# We declare parameters (key-value pairs that can be chnaged at runtime)
		# in a declear_parameter(key, value) function
        self.declare_parameter('quote', 'Parameter default value')
        self.declare_parameter('philosopher_name', 'Philosopher default value')

        # One-off parameters. no difference in declarition anyways
		# We only get them once in the node lifetime
        self.declare_parameter('topic_name', 'amazing_quote') # There MUST be no sapace in both key or values, why?
        topic_name: str = self.get_parameter('topic_name').get_parameter_value().string_value
        self.declare_parameter('period', 1)
        timer_period: float = self.get_parameter('period').get_parameter_value().double_value

        self.configurable_amazing_quote_publisher = self.create_publisher(
            msg_type=AmazingQuote,
            topic=topic_name,
            qos_profile=1)

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.incremental_id: int = 0

    def timer_callback(self):
        """Method that is periodically called by the timer."""

		# Each time this function is called by the timer, it retrieves the value stored in the parameter
		# and uses it. Therefore, is a parameter changes externally, when the noce comes to retrieve it, 
		# it will retrieve the updated versiom
        quote: str = self.get_parameter('quote').get_parameter_value().string_value
        philosopher_name: str = self.get_parameter('philosopher_name').get_parameter_value().string_value

        amazing_quote = AmazingQuote()
        amazing_quote.id = self.incremental_id
        amazing_quote.quote = quote
        amazing_quote.philosopher_name = philosopher_name

        self.configurable_amazing_quote_publisher.publish(amazing_quote)

        self.incremental_id = self.incremental_id + 1


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        configurable_amazing_quote_publisher_node = AmazingQuoteConfigurablePublisherNode()

        rclpy.spin(configurable_amazing_quote_publisher_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
