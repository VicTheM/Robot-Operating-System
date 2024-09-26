"""This file houses the server that responds to an action request"""
import time

import rclpy
from rclpy.action import ActionServer					    # Used to create an instance of an action
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci 	# Our own action definition


class FiboActionServer(Node):
    """Just like any other node in ROS2"""

    def __init__(self):
        super().__init__('fibo_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,                       # The action
            'fibonacci',                     # Action internal name
            self.execute_callback)

	# This function is called when a request is made to the action servero    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # goal_handle is the request
        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()
        feedback.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback.partial_sequence.append(
                    feedback.partial_sequence[i] + feedback.partial_sequence[i-1]
                    )
            self.get_logger().info('Feedback: {0}'.format(feedback.partial_sequence))
            goal_handle.publish_feedback(feedback)
            time.sleep(5)

        goal_handle.succeed()               # it was successfule, could also be aborted
        result.sequence = feedback.partial_sequence

        return result


def main(args=None):
    try:
    	rclpy.init(args=args)

    	fibo_action_server = FiboActionServer()

    	rclpy.spin(fibo_action_server)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)



if __name__ == '__main__':
    main()
