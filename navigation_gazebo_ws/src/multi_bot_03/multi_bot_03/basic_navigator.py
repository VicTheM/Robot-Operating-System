#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified: robotics.snowcron.com


import concurrent.futures
import math
import time
from enum import Enum

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
# Comment the following line for Foxy, uncomment for Galactic:
from nav2_msgs.action import (ComputePathThroughPoses, ComputePathToPose,
                              FollowWaypoints, NavigateThroughPoses,
                              NavigateToPose)
from nav2_msgs.srv import (ClearEntireCostmap, GetCostmap, LoadMap,
                           ManageLifecycleNodes)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

#from gazebo_msgs.srv import SetModelState, GetModelState
#from gazebo_msgs.msg import ModelState
#from geometry_msgs.msg import Pose, Twist



class NavigationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3 


class BasicNavigator(Node):
    def __init__(self, strRobotName = ""):
        if(strRobotName != ""):
            super().__init__('basic_navigator_' + strRobotName)
            self.strRobotNameSlash = "/" + strRobotName
            self.strRobotNameUnderscore = "_" + strRobotName
            self.model_name = strRobotName
        else:
            super().__init__('basic_navigator')
            self.strRobotNameSlash = ""
            self.strRobotNameUnderscore = ""
            self.model_name = "multi_bot_03"

        self.strRobotName = strRobotName

        # ---
            
        time = self.get_clock().now().to_msg()

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = time
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.position.z = 0.0
        self.initial_pose.pose.orientation.x = 0.0
        self.initial_pose.pose.orientation.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0   

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.bNodesActive = False

        # Comment the following line for Foxy, uncomment for Galactic:
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Foxy:
        # self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')

        # Galactic:
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Comment the following line for Foxy, uncomment for Galactic:
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses,
            'compute_path_through_poses')
        
        # ---

        self.initial_pose_received = False
        
        self.amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
            '/amcl_pose' + self.strRobotNameSlash, self._amclPoseCallback, self.amcl_pose_qos)
       
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
            '/initialpose' + self.strRobotNameSlash, 10)
        
        # ---
        
        self.change_maps_srv = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, '/local_costmap/get_costmap')

        # TBD: set robot init. pos in Gazebo
        # set_model_state_client and get_model_state_client are client objects 
        # created using the create_client() method of the rclpy library. These 
        # client objects are used to communicate with the services provided by the Gazebo simulator.
        # set_model_state_client is used to set the state of a model in Gazebo, 
        # such as its position, orientation, and velocity. It takes in a SetModelState 
        # service request, which includes the name of the model and its desired state.
        # get_model_state_client is used to retrieve the state of a model in Gazebo, 
        # such as its position, orientation, and velocity. It takes in a 
        # GetModelState service request, which includes the name of the model to retrieve.
        # Both of these clients allow your code to interact with the Gazebo 
        # simulator by sending service requests and receiving responses. 
        # By using these clients, you can control the simulation and obtain 
        # information about the objects in the environment.
        # self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        # self.get_model_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()
    
    def goThroughPoses(self, poses):
        # Sends a `NavThroughPoses` action request
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose):
        # Sends a `NavToPose` action request
        print("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):

        # Sends a `FollowWaypoints` action request
        print("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Following ' + str(len(poses)) + ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        print('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            print("task was cancelled or completed")
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print('Goal failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            # print("Timed out, still processing, not complete yet")
            return False

        print('isNavComplete: Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        print('>>> amcl starting...', end=' ')
        self._waitForNodeToActivate('amcl' + self.strRobotNameUnderscore)
        print('done.')

        print(">>>_waitForInitialPose...", end=" ")
        self._waitForInitialPose()
        print('done.')

        lifecycle_nodes = [
            'amcl', 
            #'ekf_filter_node',
            'controller_server',
            'planner_server',
            'recoveries_server',
            'bt_navigator',
            'waypoint_follower'
        ]

        for node_name in lifecycle_nodes:
            print(">>>_waitForNodeToActivate (", node_name, ")...", end=" ")
            self._waitForNodeToActivate(node_name)
            print('done.')

        self.info('Nav2 is ready for use!')

        self.bNodesActive = True
        return

    def getPath(self, start, goal):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path

    '''
    def getPathThroughPoses(self, start, goals):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathThroughPoses' action server not available, waiting...")

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = goals
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path
    '''

    def changeMap(self, map_filepath):
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change map request failed!')
        else:
            self.info('Change map request was successful!')
        return

    def clearAllCostmaps(self):
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        srvs = self.get_service_names_and_types()
        for srv in srvs:
            if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                srv_name = srv[0]
                self.info('Starting up ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        srvs = self.get_service_names_and_types()
        for srv in srvs:
            if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                srv_name = srv[0]
                self.info('Shutting down ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
            time.sleep(2)
        return

    def get_localization_pose_sub(self):
        while not self.initial_pose_received:
            # wait for the first callback
            time.sleep(1)
            pass
        return self.localization_pose_sub


    def destroy_subscription(self):
        if self.localization_pose_sub:
            self.localization_pose_sub.destroy()

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        # TBD: check this logic
        if (not self.initial_pose_received) and self.bNodesActive == True:

            # unsubscribe after receiving the message once
            self.destroy_subscription()


    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        #print('Received action feedback message: ', self.feedback)
        return
    
    '''
    def getYaw(self, orientation):
        """
        Extracts the yaw angle (in radians) from the given orientation quaternion.
        """
        # Extract quaternion components
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Compute Euler angles (roll, pitch, yaw) from quaternion
        t0 = +2.0 * (w * z + x * y)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t0, t1)

        return yaw    
    '''
    
    '''
    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return np.array([qx, qy, qz, qw])    
    '''

    '''
    def set_initial_pose(self, x, y, z, yaw):
        # Wait for the set_model_state service to become available
        
        # self.wait_for_service('/gazebo/set_model_state') is used to wait for the '/gazebo/set_model_state' service to become available before trying to call it. This is because the set_initial_pose function needs to call the '/gazebo/set_model_state' service to update the pose of the robot in the simulation.
        # If the service is not available when wait_for_service is called, the function will block until the service becomes available or until a timeout is reached. Once the service becomes available, the function will return and the set_initial_pose function can proceed to call the '/gazebo/set_model_state' service.
        # By waiting for the service to become available before calling it, we can ensure that the service call will succeed and the robot's pose will be updated in the simulation.        
       
        #self.wait_for_service('/gazebo/set_model_state')
        while not self.set_model_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /gazebo/set_model_state not available, waiting again...')

        while not self.get_model_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /gazebo/get_model_state not available, waiting again...')

        # Create a proxy to the set_model_state service
        set_model_state = self.create_client(SetModelState, '/gazebo/set_model_state')

        # Create a ModelState message with the new pose and twist
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = self.quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        twist = Twist()
        model_state = ModelState()
        
        #Yes, if you start 10 identical robots based on the same URDF file, they will all have the same model_name in Gazebo.
        #When you call the set_model_state service to set the initial pose of the robot, the model_name field in the ModelState message specifies which robot you want to update. If you set the model_name to the same value for all 10 robots, then the last set_model_state call will overwrite the initial pose of all 10 robots.
        #To avoid this, you can use different model_name values for each robot. One way to do this is to append a unique identifier to the end of the model_name for each robot, such as a number or a random string. This will ensure that each robot is uniquely identified in Gazebo, and the set_model_state service will set the initial pose of the correct robot.
        
        model_state.model_name = self.model_name
        model_state.pose = pose
        model_state.twist = twist

        # Call the set_model_state service
        
        #In this part, we are using the SetModelState service to set the initial pose of the robot in the Gazebo simulator.
        #request = SetModelState.Request() creates a request object to send to the SetModelState service.
        #request.model_state = model_state sets the model_state field of the request object to the ModelState message we created earlier, which contains the new pose and twist.
        #future = set_model_state.call_async(request) calls the SetModelState service asynchronously with the request object we just created, and returns a future object that will be set with the result of the service call when it completes.
        #rclpy.spin_until_future_complete(self, future) spins the node until the future object is completed, which means that the service call has completed and the result is available. This ensures that the service call is processed and the result is retrieved before the function exits.
        #if future.result() is None: checks if the result of the service call is None, which indicates that the call failed.
        #self.get_logger().error('Service call failed') logs an error message if the service call failed.
        #self.get_logger().info('Initial pose set successfully') logs an info message if the service call succeeded and the initial pose was set successfully.
        
        request = SetModelState.Request()
        request.model_state = model_state
        future = set_model_state.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Service call failed')
        else:
            self.get_logger().info('Initial pose set successfully')
    '''

    def _setInitialPose(self):
        # set the initial pose in Gazebo
#        self.set_initial_pose(self.initial_pose.pose.position.x,
#                        self.initial_pose.pose.position.y,
#                        self.initial_pose.pose.position.z,
#                        self.getYaw(self.initial_pose.pose.orientation))
                
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)

        # Temp!!!
        # self.initial_pose_received = True
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
