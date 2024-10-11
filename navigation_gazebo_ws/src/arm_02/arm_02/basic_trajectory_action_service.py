# (c) robotics.snowcron.com
# Use: MIT license

import os
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
import tinyik

class TrajectoryActionClient(Node):

    def __init__(self, joint_names):

        super().__init__('points_publisher_node_action_client')
        self.action_client = ActionClient (self, FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.joint_names = joint_names

    def send_goal(self):
    
        arrAngles = [0.0, 1.0, 0.0, -1.0, 0.0]
        points = []
        nDuration = 2.0
        for nJoint, _ in enumerate(self.joint_names):
        
            arrPositions = [0.0] * len(self.joint_names)
            for t, angle in enumerate(arrAngles):
                arrPositions[nJoint] = angle

                point_msg = JointTrajectoryPoint()
                point_msg.positions = arrPositions
                point_msg.time_from_start = Duration(seconds=nDuration).to_msg()
                nDuration += 2.0

                points.append(point_msg)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, 
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def send_effector_pos(self, arrAngles):
        points = []
        nDuration = 2.0
        point_msg = JointTrajectoryPoint()
        point_msg.positions = arrAngles
        point_msg.time_from_start = Duration(seconds=nDuration).to_msg()

        points.append(point_msg)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)        
    
    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected ')
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future= goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback (self, future):
        
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


def main(args=None):

    # tinyik libarry:

    # --- Init

    arm = tinyik.Actuator([
        'z', [0., 0., 0.114],
        'z', [0., 0.08, 0.08],
        'y', [0., 0., 0.5],
        'y', [0., -0.08, 0.5],
        'z', [0., 0.08, 0], 
        'y', [0., 0., 0.08], 
        'z', [0., 0., 0.], 
    ])

    # dictJointLimits = {
    #     'joint1': [-6.2832, 6.2832],
    #     'joint2': [-6.2832, 6.2832],
    #     'joint3': [-6.2832, 6.2832],
    #     'joint4': [-6.2832, 6.2832],
    #     'joint5': [-6.2832, 6.2832],
    #     'joint6': [-6.2832, 6.2832],
    #     'gripper_joint_left_1': [-0.04, 0.06]
    # }

    # objKinematics = inverse_kinematics.Kinematics(arrJointCoords, len(joint_names), dictJointLimits)    

    #--- Simple FK and IK:

    arm.angles = [0., 0., np.pi / 2, 0., 0., 0., 0.]
    print(arm.angles)
    print(arm.ee)    
    #print(np.round(np.rad2deg(arm.angles)))

    # --- Simulator
    joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6', 'gripper_joint_left_1']

    rclpy.init()
    action_client = TrajectoryActionClient(joint_names)

    #arm.ee = [0.5, 0, 0]
    arrTargetAngles = [0., 0., np.pi / 2, 0., 0., 0., 0.035]
    print(arrTargetAngles)
    future = action_client.send_effector_pos(arrTargetAngles)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()




