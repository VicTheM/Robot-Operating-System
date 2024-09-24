"""This file houses a node that makes a request to a service server"""

import random
from textwrap import dedent

import rclpy
from rclpy.task import Future
from rclpy.node import Node

from package_with_interfaces.srv import WhatIsThePoint


class WhatIsThePointServiceClientNode(Node):
    """A ROS2 Node with a Service Client for WhatIsThePoint."""

    def __init__(self):
        super().__init__('what_is_the_point_service_client')

        self.service_client = self.create_client(
            srv_type=WhatIsThePoint,
            srv_name='/what_is_the_point')

        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.service_client.srv_name} not available, waiting...')

        self.future: Future = None

        timer_period: float = 0.5
        self.timer = self.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback)

    def timer_callback(self):
        """Method that is periodically called by the timer."""

        request = WhatIsThePoint.Request()
        if random.uniform(0, 1) < 0.5:
            request.quote.quote = "I wonder about the Ultimate Question of Life, the Universe, and Everything."
            request.quote.philosopher_name = "Creators of Deep Thought"
            request.quote.id = 1979
        else:
            request.quote.quote = """[...] your living... it is always potatoes. I dream of potatoes."""
            request.quote.philosopher_name = "a young Maltese potato farmer"
            request.quote.id = 2013

        if self.future is not None and not self.future.done():
            self.future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().info("Service Future canceled. The Node took too long to process the service call."
                                   "Is the Service Server still alive?")
        self.future = self.service_client.call_async(request)
        self.future.add_done_callback(self.process_response)

    def process_response(self, future: Future):
        """Callback for the future, that will be called when it is done"""
        response = future.result()
        if response is not None:
            self.get_logger().info(dedent(f"""
                We have thus received the point of our quote.

                            {(response.point.x, response.point.y, response.point.z)}
            """))
        else:
            self.get_logger().info(dedent("""
                    The response was None. :(    
            """))


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        what_is_the_point_service_client_node = WhatIsThePointServiceClientNode()

        rclpy.spin(what_is_the_point_service_client_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
