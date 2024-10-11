# (c) robotics.snowcron.com
# Use: MIT license

import os
import time
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

import numpy as np
import tinyik

from action_msgs.srv import CancelGoal

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from rclpy.executors import SingleThreadedExecutor

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SetEntityState

class RoboticArmController(Node):
    def __init__(self, arm_joint_names, gripper_joint_names):

        super().__init__('robotic_arm_controller')
        self.action_client = ActionClient (self, FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory')

        # RoboticArmController controls 2 groups of joints: one for arm and
        # one for gripper. These groups can not move simultaneously: gripper
        # opens or closes only when arm doesn't move.

        self.arm_joint_names = arm_joint_names
        self.gripper_joint_names = gripper_joint_names

        # "opened", "closed", "failed"
        self.nStatus = "opened"
        self.arrPoses = []
        self.nCurrentPosIdx = 0

        # # Create a service to receive cancel_goal requests
        # self.cancel_goal_service = self.create_service(
        #     CancelGoalService, '/trajectory_action_client/cancel_goal', self.cancel_goal_service_callback
        # )

    def send_effector_pos(self, arrAngles):
        #arrGripperVelocities = [10., 10., 10., 10., 10., 10., 10., 10.]

        print("Target: ", arrAngles)  

        points = []
        nDuration = 2.0
        point_msg = JointTrajectoryPoint()
        
        # arm or gripper
        strType = arrAngles['name']

        point_msg.positions = arrAngles['pose']
        

        point_msg.time_from_start = Duration(seconds=nDuration).to_msg()

        points.append(point_msg)
        
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        
        if(strType == 'arm'):
            goal_msg.trajectory.joint_names = self.arm_joint_names
        else:
            goal_msg.trajectory.joint_names = self.gripper_joint_names

        goal_msg.trajectory.points = points

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)     
    
    def goal_response_callback(self, future):
        
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected ')
            self.nStatus = "failed"
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future= self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback (self, future):
        result = future.result().result
        self.get_logger().info('>>> Result: '+str(result))

        if(self.nStatus == "opening"):
            self.nStatus = "opened"

        # Start processing next pose
        self.start(None)

        
    def feedback_callback(self, feedback_msg):
        self.feedback_msg = feedback_msg
        #print(feedback)

    def start(self, rclpy):
        if(rclpy is not None):
            self.rclpy = rclpy

        if(self.nStatus != "failed"):
            if(self.nCurrentPosIdx < len(self.arrPoses)):
                arrCurrentPose = self.arrPoses[self.nCurrentPosIdx]

                if(arrCurrentPose['name'] == 'gripper'):
                    if(self.nStatus == "closed"):
                        # If this is an "open" command
                        if(self.leftGripperPos > arrCurrentPose['pose'][0]):
                            self.nStatus = "opening"
                            self.send_effector_pos(arrCurrentPose)
                        else:
                            pass    # Do not close if already closed
                    else:   # opened
                        self.send_effector_pos(arrCurrentPose)
                else:   # move arm
                    self.send_effector_pos(arrCurrentPose)

                self.nCurrentPosIdx += 1
            else:
                self.rclpy.shutdown()
                print(">>> Done")
                pass

# ==========================================

class GraspingController(Node):

    def __init__(self, robotic_arm_controller):
        super().__init__('grasping_controller')

        self.robotic_arm_controller = robotic_arm_controller
        
        # Create a QoS profile with the desired reliability and durability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Subscribe using the created QoS profile
        self.contact_state_subscription = self.create_subscription(
            ContactsState,
            '/contact_sensor/bumper_gripper_link_left',
            self.contact_callback,
            qos_profile
        )     

        # # Create a client for the vacuum gripper's switch service
        # self.vacuum_switch_client = self.create_client(SetBool, 'custom_switch')
        # while not self.vacuum_switch_client.wait_for_service(timeout_sec=1.0):
        #     print('Service not available, waiting...')

        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting...')

        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            print('Service /set_entity_state is not available, waiting...')

        self.timer = self.create_timer(0.02, self.set_object_position_callback) # 50Hz
        self.gripped_object_pose = Pose(
            position=Point(x=-0.5, y=0.0, z=2.0),  # Change this to the desired position
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Change this to the desired orientation
        )        

        # self.cancel_client = self.create_client(CancelGoal, '/follow_joint_trajectory/cancel')

    def set_object_position_callback(self):
        if(self.robotic_arm_controller.nStatus == "closed"):
            request = SetEntityState.Request()
            request.state.name = "box_01"
            request.state.pose = self.gripped_object_pose
            request.state.reference_frame = 'armA::gripper_link_right_1'

            future = self.set_entity_state_client.call_async(request)
            future.add_done_callback(self.set_entity_state_callback)

    def set_entity_state_callback(self, future):
        try:
            if future.result() is not None:
                response = future.result()
                if response.success:
                    pass #self.get_logger().info('Object position set successfully')
                else:
                    print('>>> Failed to set object position: ', response.status_message)
        except Exception as e:
            print('>>> Service call failed to set entity state', str(e))                

    def get_box_coordinates(self):
        request = GetEntityState.Request()
        request.name = "box_01"
        request.reference_frame = 'armA::gripper_link_right_1'

        future = self.get_entity_state_client.call_async(request)
        #self.executor.spin_until_future_complete(self, future)
        future.add_done_callback(self.get_entity_state_callback)


    def get_entity_state_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.gripped_object_pose = response.state.pose
                print(f"Box position: x={self.gripped_object_pose.position.x}, \
                    y={self.gripped_object_pose.position.y}, \
                    z={self.gripped_object_pose.position.z}")

                print(f"Box orientation: x={self.gripped_object_pose.orientation.x}, \
                    y={self.gripped_object_pose.orientation.y}, \
                    z={self.gripped_object_pose.orientation.z}, \
                    w={self.gripped_object_pose.orientation.w}")
            else:
                print('>>> Failed to get entity state: ', response.status_message)
        except Exception as e:
            print('>>> Service call failed to get entity state', str(e))

    def contact_callback(self, msg):
        if(self.robotic_arm_controller.nStatus != "opened"):
            return

        for contact in msg.states:
            link1 = contact.collision1_name
            link2 = contact.collision2_name
            left_finger = 'armA::gripper_link_left_1::gripper_link_left_1_collision'
            right_finger = 'armA::gripper_link_right_1::gripper_link_right_1_collision'
            base_link = 'gripper_link_base'
            
            # Check if contact involves gripper fingers and any other object
            if((left_finger in link1 and base_link not in link2 and right_finger not in link2) \
                or (left_finger in link2 and base_link not in link1 and right_finger not in link2 )):
               
                # box_01::link::collision armA::gripper_link_left_1::gripper_link_left_1_collision
                # print(">>> ", link1, link2)
                self.handle_contact(link1, link2)
                
                break

    def handle_contact(self, finger_link, object_link):
        # Implement logic for handling the contact
        print(">>> ", finger_link, " is in contact with ", object_link, " <<<", flush=True)

        self.get_box_coordinates()
        #self.robotic_arm_controller.send_goal_future.cancel()

        # Call service to cancel goal
        # request = CancelGoal.Request()
        # request.goal_info.goal_id = self.robotic_arm_controller.goal_handle.goal_id
        # self.cancel_client.call_async(request)      
         
        # feedback_msg = self.robotic_arm_controller.goal_handle.get_status()
        # current_positions = feedback_msg.feedback.desired.positions

        # self.robotic_arm_controller.leftGripperPos = current_positions[6]
        # self.robotic_arm_controller.rightGripperPos = current_positions[7]

        self.robotic_arm_controller.nStatus = "closed"

        current_positions = self.robotic_arm_controller.feedback_msg.feedback.desired.positions
        self.robotic_arm_controller.leftGripperPos = current_positions[6]
        self.robotic_arm_controller.rightGripperPos = current_positions[7]
        self.robotic_arm_controller.strMessage = "Sending blocking one"
        self.robotic_arm_controller.send_effector_pos({'name': 'gripper', 'pose': current_positions[6:] })
        
        # self.robotic_arm_controller.goal_handle.cancel_goal() #get_status()
        # self.robotic_arm_controller.goal_handle.wait_for_cancel()
        # print(">>> Goal canceled", flush=True)

        # feedback_msg = self.robotic_arm_controller.goal_handle.get_status()
        # current_positions = feedback_msg.feedback.desired.positions

        # self.robotic_arm_controller.leftGripperPos = current_positions[6]
        # self.robotic_arm_controller.rightGripperPos = current_positions[7]

        # current_positions = feedback_msg.feedback.desired.positions         
        # self.robotic_arm_controller.send_effector_pos(current_positions)
        
       

        # print(f"Box position: x={box_pose.position.x}, y={box_pose.position.y}, z={box_pose.position.z}")
        # print(f"Box orientation: x={box_pose.orientation.x}, \
        #     y={box_pose.orientation.y}, z={box_pose.orientation.z}, w={box_pose.orientation.w}")
    

        #self.robotic_arm_controller.arrPoses.clear()

        #self.get_logger().info(f"Color changed to {rgba}")
        # self.get_logger().info(f"{finger_link} is in contact with {other_link}")


# ==========================================

def main(args=None):

    arm_joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
    gripper_joint_names = ['gripper_joint_left_1', 'gripper_joint_right_1']

    try:  
        rclpy.init(args=args)

        robotic_arm_controller = RoboticArmController(arm_joint_names, gripper_joint_names)
        grasping_controller = GraspingController(robotic_arm_controller)

        grasping_controller.robotic_arm_controller.arrPoses = [
            # Open gripper
            { 'name': 'gripper',    'pose': [-0.04, 0.04] },
            # Bend
            { 'name': 'arm',        'pose': [0.31, -0.31, -1.89, 0., -1.19, 1.8] },
            # Close gripper
            { 'name': 'gripper',    'pose': [0.1, -0.1] },
            # Unbend
            { 'name': 'arm',        'pose': [0., 0., 0., 0., 0., 0.] },
            # Open gripper
            { 'name': 'gripper',    'pose': [-0.04, 0.04] },
        ]

        grasping_controller.robotic_arm_controller.start(rclpy)

        executor = SingleThreadedExecutor()

        grasping_controller.rclpy = rclpy
        grasping_controller.executor = executor

        executor.add_node(grasping_controller)
        executor.add_node(grasping_controller.robotic_arm_controller)

        try:
            executor.spin()
        finally:
            executor.shutdown()
 
    finally:
        # Shutdown
        #rclpy.shutdown()     
        pass  

if __name__ == '__main__':
    main()




