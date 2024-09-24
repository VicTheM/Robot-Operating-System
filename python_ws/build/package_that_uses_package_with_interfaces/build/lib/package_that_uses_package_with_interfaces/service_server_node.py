"""This file houses a node that offers a service upon request"""
import random
from textwrap import dedent

import rclpy
from rclpy.node import Node
from package_with_interfaces.srv import WhatIsThePoint


class WhatIsThePointServiceServerNode(Node):
    """A ROS2 Node with a Service Server for WhatIsThePoint Service."""

    def __init__(self):
        super().__init__('service_server')

		# Create the service and assign it to a class attribute
		# srv_name is more like topic in msgs
		# callback gets executed when service is requested
        self.service_server = self.create_service(
            srv_type=WhatIsThePoint,
            srv_name='/what_is_the_point',
            callback=self.what_is_the_point_service_callback)

        self.service_server_call_count: int = 0

    def what_is_the_point_service_callback(self,
                                           request: WhatIsThePoint.Request,
                                           response: WhatIsThePoint.Response
                                           ) -> WhatIsThePoint.Response:

        # Generate the x,y,z of the point
        if "life" in request.quote.quote.lower():
            x: float = random.uniform(0, 42)
            y: float = random.uniform(0, 42 - x)
            z: float = 42 - (x + y)
        else:
            x: float = random.uniform(0, 100)
            y: float = random.uniform(0, 100)
            z: float = random.uniform(0, 100)
            if x + y + z == 42:  # So you’re telling me there’s a chance? Yes!
                x = x + 1  # Not anymore :(

        # Assign to the response
        response.point.x = x
        response.point.y = y
        response.point.z = z

        # Increase the call count
        self.service_server_call_count = self.service_server_call_count + 1

        self.get_logger().info(dedent(f"""
            This is the call number {self.service_server_call_count} to this Service Server.
            The analysis of the AmazingQuote below is complete.
            
                    {request.quote.quote}
            
            -- {request.quote.philosopher_name}
            
            The point has been sent back to the client.
        """))

        return response		# same as send_response() ?


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        what_is_the_point_service_server_node = WhatIsThePointServiceServerNode()

        rclpy.spin(what_is_the_point_service_server_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
