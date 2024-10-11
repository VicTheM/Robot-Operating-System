# (c) robotics.snowcron.com
# Use: MIT license

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_srvs.srv import Trigger

#from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Twist         # Velocity command

import time
from datetime import datetime

from rclpy.time import Time

import math
from lifecycle_msgs.srv import GetState

from rclpy.action import ActionClient
from gazebo_msgs.srv import SpawnEntity

from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import ModelStates

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum

from rclpy.qos import QoSPresetProfiles

import matplotlib.pyplot as plt
import copy

# Path visualization
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import DeleteModel
from rclpy.duration import Duration

import subprocess

import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# class NavigationResult(Enum):
#     UNKNOWN = 0
#     SUCCEEDED = 1
#     CANCELED = 2
#     FAILED = 3 

class PathFollower(Node):

    def __init__(self, robot_name):
        print("*** PathFollower.init()")

        # ---

        self.arrColorIds = []

        # TBD: this should be moved to common code area
        if(robot_name != ""):
            super().__init__(robot_name + '_PathFollower')
            self.strSlashRobotName = "/" + robot_name
            self.strRobotNameSlash = robot_name + "/"
        else:
            super().__init__('PathFollower')
            self.strSlashRobotName = ""
            self.strRobotNameSlash = ""

        self.strRobotName = robot_name

        # ---

        if(self.strRobotName == ""):
            self.path_color_0 = (0, 255, 0, 128)          # bgra, green
            self.path_color_1 = (255, 0, 0, 128)          # bgra, blue
            self.real_path_color = (0, 0, 255, 255)       # bgra, red
        elif(self.strRobotName == "robot1"):
            self.path_color_0 = (255, 255, 0, 128)          # bgra, cyan
            self.path_color_1 = (255, 0, 255, 128)          # bgra, magenta
            self.real_path_color = (0, 255, 255, 255)     # bgra, yellow
        else:
            self.path_color_0 = (0, 255, 0, 128)          # bgra, green
            self.path_color_1 = (255, 0, 0, 128)          # bgra, blue
            self.real_path_color = (0, 0, 255, 255)     # bgra, red

        # ---

        self.dDistBetweenRobotMarkers = 0.2
        self.grid_size = 16   # 16x16 m

        self.img_height = 480
        self.img_width = 640
        self.n_channels = 4
        self.imgCanvas = np.zeros((self.img_height, self.img_width, self.n_channels), dtype=np.uint8)
        self.bridge = CvBridge()
        self.frame_num = 0

        self.canvas_publisher = self.create_publisher(Image, 
            self.strSlashRobotName + '/images/robot_path_markers', 10)

        # ---

        # Uncomment to see RViz markers
        # self.marker_pub = self.create_publisher(Marker, '/gazebo/default/marker', 10)
        # self.marker_array_publisher = self.create_publisher(MarkerArray, 
        #     self.strSlashRobotName + '/markers_path_array', 10)
        self.markers = None

        # ---

        self.nCurrentSegment = 0
        self.nTotalSegments = -1
        self.lookaheadDistance = 1.0
        self.dLookaheadAccuracy = 0.1
        self.bNearFinish = False
        self.target_pose = Pose()
        self.dDistanceToFinish = float("inf")

        # --- Used to present data as charts

        self.arrChartTime         = []   # Integer numbers in seconds
        self.arrChartAMCL         = []
        self.arrChartOdom         = []
        self.arrChartFilteredOdom = []
        self.arrChartGPS          = []
        self.arrChartFilteredGPS  = []
        self.arrChartGT           = []

        self.nPointIdx = 0

        # ---

        self.bPathSet = False
        self.nTotalSegments = -1

        # Temp!
        self.current_pose = Pose()
        self.current_pose.position.x = -2.0
        self.current_pose.position.y = -2.0
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = 0.0
        self.current_pose.orientation.w = 1.0

        self.amcl_pose = copy.deepcopy(self.current_pose)
        self.odom_pose = copy.deepcopy(self.current_pose)
        self.filtered_odom_pose = copy.deepcopy(self.current_pose)
        self.ground_truth_pose = copy.deepcopy(self.current_pose)
        self.prev_pose = copy.deepcopy(self.current_pose)
        self.gps_pose = copy.deepcopy(self.current_pose)
        self.gps_filtered_pose = copy.deepcopy(self.current_pose)
        
        # Subscribe to "path_follower_command" TEXT message
        self.command_subscription = self.create_subscription(
            String,
            self.strSlashRobotName + "/path_follower_command",
            self.command_callback,
            10  # QoS profile depth
        )
        #self.command_subscription  # Prevent unused variable warning

        self.path_follower_command_publisher = self.create_publisher(
            String,
            self.strSlashRobotName + "/path_follower_command",
            10  # QoS profile depth
        )        

        # ---
        
        # Subscribe to "path_follower_path" nav_msgs/Path message
        self.path_subscription = self.create_subscription(
            Path,
            self.strSlashRobotName + "/path_follower_set_path",
            self.set_path_callback,
            10  # QoS profile depth
        )
        self.path_subscription  # Prevent unused variable warning

        # Create a publisher for "path_follower_response" TEXT message
        self.response_publisher = self.create_publisher(
            String,
            self.strSlashRobotName + "/path_follower_response",
            10  # QoS profile depth
        )

        self.path_follower_path_publisher = self.create_publisher(
            Path,
            self.strSlashRobotName + "/path_follower_set_path",
            10  # QoS profile depth
        )        

        # ---

        # Publish the desired linear and angular velocity of the robot (in the
        # robot chassis coordinate frame) to the /cmd_vel_unstamped topic. 
        # The diff_drive controller that our robot uses will read this topic 
        # and move the robot accordingly.
        self.publisherCmdVel = self.create_publisher(
            Twist, #TwistStamped, 
            #'/diff_cont/cmd_vel_unstamped' + self.strSlashRobotName,  # For ROS2 diff. drive controller
            self.strSlashRobotName + "/cmd_vel",                       # For Gazebo diff. drive controller
            10)            

        self.angular_speed = 1.0 #0.5
        self.linear_speed = 1.0 #0.5

        # ---

        # Note that calling node (in our case it is ChargerNavigator)
        # has to make sure self.waitForNodesToStart() was called and
        # finished with success status

        self.navStatus = "idle"

        # ---

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.navigation_callback)
        #time = self.get_clock().now().to_msg()
        # Used to publish status
        self.prev_time = datetime.now()

        #self.end_move_time = None
        #self.end_angle = None

        # ---

        # self.ground_truth_coords_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        # while not self.ground_truth_coords_client.wait_for_service(timeout_sec=1.0):
        #     print("Waiting for '/gazebo/get_entity_state' service...")

        ground_truth_pos_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.ground_truth_pose_callback,
            10  # QoS profile depth
        )

        # ---

        # Subscribe to messages of type nav_msgs/Odometry that provides position and orientation of the robot
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.strSlashRobotName + '/odom',
            self.odom_callback,
            10)        

        # ---

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Adjust the depth according to your needs
            durability=QoSDurabilityPolicy.VOLATILE  # You can use TRANSIENT_LOCAL for durability
        )

        self.filtered_odom_subscriber = self.create_subscription(
            Odometry,
            self.strSlashRobotName + '/odometry/filtered',
            self.filtered_odom_callback,
            qos_profile = qos_profile)

        # --- GPS

        # Set GPS initial pos
        self.origin_latitude = 44.98
        self.origin_longitude = -93.27
        self.origin_altitude = 254.99

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            self.strSlashRobotName + '/gps/fix',
            self.gps_callback,
            qos_profile = qos_profile  # Set the queue size
        )

        self.gps_filtered_subscriber = self.create_subscription(
            NavSatFix,
            self.strSlashRobotName + '/gps/filtered',
            self.gps_filtered_callback,
            qos_profile = qos_profile  # Set the queue size
        )

        # --- AMCL related; TBD: move starting AMCL and getting init. position to a separate class/node?

        self.bNodesActive = False
        self.initial_pose_received = False

        self.pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
            self.strSlashRobotName + '/amcl_pose', self._amclPoseCallback, self.pose_qos)

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
            self.strSlashRobotName + '/initialpose', 10) 
            
        self.waitForNodesToStart()

        # ---

        # self.datum_client = self.create_client(SetDatum, '/datum')
        # while not self.datum_client.wait_for_service(timeout_sec=1.0):
        #     print('Service not available, waiting...')

        # self.call_set_datum_service()



        print("*** PathFollower.init(): done")

    # ------

    def command(self, strCommand):
        command_msg = String()
        command_msg.data = strCommand

        self.path_follower_command_publisher.publish(command_msg)

    def waitForNodesToStart(self):
        print("*** PathFollower.waitForNodesToStart()...")
        print("\t*** amcl starting...")
        self._waitForNodeToActivate(self.strRobotNameSlash + 'amcl')
        print("\t*** amcl started successfully")

        # print("\t*** Datum starting...")
        # self._waitForNodeToActivate(self.strRobotNameSlash + 'datum')
        # print("\t*** datum started successfully")        

        self._waitForInitialPose()
        
        print("*** PathFollower.waitForNodesToStart() done")

        self.bNodesActive = True

    # ------

    # def call_set_datum_service(self):

    #     request = SetDatum.Request()
    #     request.geo_pose.position.latitude = self.origin_latitude
    #     request.geo_pose.position.longitude = self.origin_longitude
    #     request.geo_pose.position.altitude = self.origin_altitude
    #     request.geo_pose.orientation.x = 0.0
    #     request.geo_pose.orientation.y = 0.0
    #     request.geo_pose.orientation.z = 0.0
    #     request.geo_pose.orientation.w = 1.0

    #     future = self.datum_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() is not None:
    #         print('Datum set successfully')
    #     else:
    #         print('Failed to set datum: ', future.exception())

    # ------

    def _waitForNodeToActivate(self, node_name):
        print("\t\t*** Waiting for ", node_name, " to become active...")
        node_service = node_name + "/get_state"
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print("\t\t*** ", node_service, " service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while (state != "active"):
            print("\t\t*** Getting ", node_name, " state...")
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print("\t\t*** Result of get_state: %s" % state)
            #time.sleep(1)
            time.sleep(1)
        return

    # ------

    def _waitForInitialPose(self):
        print("\t***_waitForInitialPose...")
        while not self.initial_pose_received:
            #print("\t\t*** Setting initial pose")
            #self._setInitialPose()
            print("\t\t***Waiting for amcl_pose to be received")
            rclpy.spin_once(self, timeout_sec=1.0)
            #time.sleep(1)
            time.sleep(1)
        print("\tInitial pose received")
        return

    # ------

    def _setInitialPose(self, pose):
                
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = self.strRobotNameSlash + 'map'
        
        # Yes, in the context of ROS 2, when you need to work with time, especially when interacting with the ROS Time system, it's generally recommended to use the Time class from rclpy.time rather than the Python standard library time module or datetime module.
        # Here's why you should use rclpy.time.Time:
        # ROS Time Integration: ROS 2 provides its own time management system to handle both real time and simulated time. The rclpy.time.Time class is part of this system and is designed to work seamlessly with ROS Time.
        # Simulated Time Support: When your ROS 2 environment is configured to use simulated time, rclpy.time.Time will automatically provide you with the simulated time, allowing your code to transition between real time and simulated time without modification.
        # ROS API Compatibility: ROS messages and other ROS components (e.g., publishers, subscribers, and actions) use rclpy.time.Time, so using it ensures compatibility with the ROS 2 ecosystem.
        # Consistency: Using rclpy.time.Time consistently throughout your ROS 2 codebase helps maintain consistency and makes it clear that you are working with ROS Time.
        # While you can still use the Python standard library time module or datetime module for general time-related tasks, when interacting with ROS 2 and its time-related components, it's advisable to use rclpy.time.Time to ensure proper integration and support for both real and simulated time.

        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        initial_pose.pose.position.x = pose.position.x
        initial_pose.pose.position.y = pose.position.y
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = pose.orientation.x
        initial_pose.pose.orientation.y = pose.orientation.y
        initial_pose.pose.orientation.z = pose.orientation.z
        initial_pose.pose.orientation.w = pose.orientation.w

        msg = PoseWithCovarianceStamped()
        msg.pose.pose = initial_pose.pose
        msg.header.frame_id = initial_pose.header.frame_id
        msg.header.stamp = initial_pose.header.stamp
        print("\t\t\t*** Publishing Initial Pose")
        self.initial_pose_pub.publish(msg)

        return

    # ------

    '''
    In the context of the amcl node, the frequency of pose updates can be controlled by 
    the following parameters:
    update_min_d: This parameter specifies the minimum translation (in meters) 
        that the robot must move before a new pose estimate is published. 
        If the robot's translation (position change) is less than this value, 
        no new pose estimate is published. 
        
    update_min_a: This parameter specifies the minimum rotation (in radians) that the robot 
        must turn before a new pose estimate is published. 
    '''

    # TBD: wait for amsl initial pose to be published
    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        self.amcl_pose = msg.pose.pose
        
        # I was still not able to figure out how to make robot_localization_node to work with 
        # namespaces. What that means: I have a class member self.current_pose.
        # Now, in a single robot project, it is possible to set this variable in
        # the callback for filtered odometry, which, in theory, provides maximum accuracy. 
        # Filtered odometry is published by robot_localization_node (ekf).
        # And it doesn't work with namespaces, so in multi-robot project I have to 
        # set the self.current_pose in AMCL callback. This has to be fixed.    
        if(self.strRobotName != ""):
            self.current_pose = msg.pose.pose

        if(self.initial_pose_received == False):
            self._setInitialPose(msg.pose.pose)
        
        self.printPoseInfo()

    # ---

    def printPoseInfo(self):

        if(self.strRobotName != "" or self.initial_pose_received == False):
            return

        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        if delta_ms >= 1000:
            amcl_pose = self.amcl_pose

            _, _, amcl_yaw = self.euler_from_quaternion(
                amcl_pose.orientation.x,
                amcl_pose.orientation.y,
                amcl_pose.orientation.z,
                amcl_pose.orientation.w)

            # ---

            odom_pose = self.odom_pose

            _, _, odom_yaw = self.euler_from_quaternion(
                odom_pose.orientation.x,
                odom_pose.orientation.y,
                odom_pose.orientation.z,
                odom_pose.orientation.w)

            # ---  

            filtered_odom_pose = self.filtered_odom_pose

            _, _, filtered_odom_yaw = self.euler_from_quaternion(
                filtered_odom_pose.orientation.x,
                filtered_odom_pose.orientation.y,
                filtered_odom_pose.orientation.z,
                filtered_odom_pose.orientation.w)

            # ---            

            ground_truth_pose = self.ground_truth_pose
            # ground_truth_pose = self.getGroundTruthCoordinates()

            _, _, ground_truth_yaw = self.euler_from_quaternion(
                ground_truth_pose.orientation.x,
                ground_truth_pose.orientation.y,
                ground_truth_pose.orientation.z,
                ground_truth_pose.orientation.w)

            # ---
            gps_pose = self.gps_pose    
            gps_filtered_pose = self.gps_filtered_pose

            print("*** Status: %s; \n\
                      X        Y        Z      Yaw \n\
AMCL:           %7.2f  %7.2f  %7.2f  %7.0f\n\
Odom:           %7.2f  %7.2f  %7.2f  %7.0f\n\
Filtered Odom:  %7.2f  %7.2f  %7.2f  %7.0f\n\
GPS:            %7.2f  %7.2f  %7.2f     ---\n\
Filtered GPS:   %7.2f  %7.2f  %7.2f     ---\n\
GT:             %7.2f  %7.2f  %7.2f  %7.0f\n===\n\n" %  
            
                (self.navStatus, 
                    amcl_pose.position.x, amcl_pose.position.y, 
                    amcl_pose.position.z, math.degrees(amcl_yaw), 

                    odom_pose.position.x, odom_pose.position.y, 
                    odom_pose.position.z, math.degrees(odom_yaw), 

                    filtered_odom_pose.position.x, filtered_odom_pose.position.y, 
                    filtered_odom_pose.position.z, math.degrees(filtered_odom_yaw),

                    gps_pose.position.x, gps_pose.position.y, 
                    gps_pose.position.z, 

                    gps_filtered_pose.position.x, gps_filtered_pose.position.y, 
                    gps_filtered_pose.position.z,                     

                    ground_truth_pose.position.x, ground_truth_pose.position.y,
                    ground_truth_pose.position.z, math.degrees(ground_truth_yaw)
                )
            )

            if(self.strRobotName == "" and self.nPointIdx > 0):
                self.arrChartTime.append(len(self.arrChartTime))
                self.arrChartAMCL.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - amcl_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - amcl_pose.position.y)**2)) 
                self.arrChartOdom.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - odom_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - odom_pose.position.y)**2))
                self.arrChartFilteredOdom.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - filtered_odom_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - filtered_odom_pose.position.y)**2))
                self.arrChartGPS.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - gps_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - gps_pose.position.y)**2))
                self.arrChartFilteredGPS.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - gps_filtered_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - gps_filtered_pose.position.y)**2))

            self.nPointIdx += 1

    # ---

    def ground_truth_pose_callback(self, msg):
        strName = self.strRobotName
        if(self.strRobotName == ""):
            strName = "nav25d_04"
        robot_index = msg.name.index(strName)
        self.ground_truth_pose = msg.pose[robot_index]
        #print(f"Robot Ground-Truth Pose: x={robot_pose.position.x}, y={robot_pose.position.y}, z={robot_pose.position.z}")


    # def getGroundTruthCoordinates(self):
    #     print("Calling getGroundTruthCoordinates")
    #     request = GetEntityState.Request() 
    #     request.name = self.strRobotName
    #     request.reference_frame = "world"
        
    #     # state = self.ground_truth_coords_client.call(request)
    #     # return state.pose

    #     future = self.ground_truth_coords_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)

    #     if future.result() is not None:
    #         print("Calling getGroundTruthCoordinates: success")
    #         return future.result().pose
    #     else:
    #         print("*** Service call failed: getGroundTruthCoordinates")
    #         return None

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        if(self.initial_pose_received == False):
            self.current_pose = msg.pose.pose
            self._setInitialPose(msg.pose.pose)

        self.printPoseInfo()

    # ---

    def lat_lon_to_xy(self, latitude, longitude):

        # Earth radius in meters (approximate)
        earth_radius = 6371000.0

        # Convert degrees to radians
        lat1 = math.radians(latitude)
        lon1 = math.radians(longitude)
        lat2 = math.radians(self.origin_latitude)
        lon2 = math.radians(self.origin_longitude)

        # Calculate the differences in latitude and longitude
        delta_lat = lat1 - lat2
        delta_lon = lon1 - lon2

        # Calculate X and Y coordinates in meters
        x = -earth_radius * delta_lon * math.cos((lat1 + lat2) / 2)
        y = -earth_radius * delta_lat

        return x, y

    # ---

    def gps_callback(self, msg):

        x, y = self.lat_lon_to_xy(msg.latitude, msg.longitude)

        self.gps_pose.position.x = x
        self.gps_pose.position.y = y
        self.gps_pose.position.z = msg.altitude
        self.gps_pose.orientation.x = 0.0
        self.gps_pose.orientation.y = 0.0
        self.gps_pose.orientation.z = 0.0
        self.gps_pose.orientation.w = 1.0    

        if(self.initial_pose_received == False):
            self.current_pose = self.gps_pose
            self._setInitialPose(self.gps_pose)
            
        # self.printPoseInfo()
        # print("Latitude: ", msg.latitude, " Longitude: ", msg.longitude, " Altitude: ", msg.altitude)
        self.printPoseInfo()

    # # ---

    def gps_filtered_callback(self, msg):

        x, y = self.lat_lon_to_xy(msg.latitude, msg.longitude)

        self.gps_filtered_pose.position.x = x
        self.gps_filtered_pose.position.y = y
        self.gps_filtered_pose.position.z = msg.longitude
        self.gps_filtered_pose.orientation.x = 0.0
        self.gps_filtered_pose.orientation.y = 0.0
        self.gps_filtered_pose.orientation.z = 0.0
        self.gps_filtered_pose.orientation.w = 1.0    

        # self.printPoseInfo()
        # print("Latitude: ", msg.latitude, " Longitude: ", msg.longitude, " Altitude: ", msg.altitude)
        self.printPoseInfo()

    # ---

    def filtered_odom_callback(self, msg):
        # I was still not able to figure out how to make robot_localization_node to work with 
        # namespaces. What that means: I have a class member self.current_pose.
        # Now, in a single robot project, it is possible to set this variable in
        # the callback for filtered odometry, which, in theory, provides maximum accuracy. 
        # Filtered odometry is published by robot_localization_node (ekf).
        # And it doesn't work with namespaces, so in multi-robot project I have to 
        # set the self.current_pose in AMCL callback. This has to be fixed.
        if(self.strRobotName == ""):
            self.current_pose = msg.pose.pose


        self.filtered_odom_pose = msg.pose.pose
        self.printPoseInfo()

    # ------

    def drive(self, linear_speed, angular_speed, dAngleToTurnRobot):
        msg = Twist()
        msg.linear.x = float(linear_speed)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        
        if(dAngleToTurnRobot < 0):
            angular_speed *= -1

        msg.angular.z = float(angular_speed)
         
        self.publisherCmdVel.publish(msg)    

    # ------

    def command_callback(self, msg):
        #print("*** PathFollower.command_callback()")

        print(f'*** Received command: {msg.data}')

        # "start", "stop" #, "reset", "revert", set speed and other params...
        self.navStatus = msg.data

        # Perform any necessary actions based on the received command
        # For example, you can process the command and send a response

        # Publish a response message
        # response_msg = String()
        # response_msg.data = 'Command received and processed.'
        # self.response_publisher.publish(response_msg)

    # ------

    def set_path_callback(self, msg):
        print("*** PathFollower.set_path_callback()")

        # TBD: stop current execution
        self.navStatus == "idle"

        self.path = msg

        # Initialize progress
        self.nTotalSegments = len(self.path.poses)

        self.bPathSet = True
        self.bNearFinish = False


    # ------

    def closest_point_on_segment(self, x1, y1, x2, y2, x, y):
        pt = Pose()
        
        if x1 == x2 and y1 == y2:
            pt.position.x = x1
            pt.position.y = y1
            return pt  # Both endpoints are the same

        # Calculate the direction vector of the segment
        dx = x2 - x1
        dy = y2 - y1

        # Calculate the length of the segment
        segment_length = dx * dx + dy * dy

        # Handle degenerate case where the segment has zero length
        if segment_length == 0:
            return x1, y1

        # Calculate the vector from (x1, y1) to the point (x, y)
        vx = x - x1
        vy = y - y1

        # Calculate the dot product of the segment and the vector to the point
        dot_product = (vx * dx + vy * dy) / segment_length

        # Clamp the dot product to the range [0, 1]
        dot_product = max(0, min(1, dot_product))

        # Calculate the closest point on the segment
        closest_x = x1 + dot_product * dx
        closest_y = y1 + dot_product * dy

        pt.position.x = closest_x
        pt.position.y = closest_y
        return pt

    # ------

    def showCharts(self):
        if(self.strRobotName == ""):
            # Create a figure and axis
            fig, ax = plt.subplots()

            # Plot the data with different colors and labels
            ax.plot(self.arrChartTime, self.arrChartAMCL, label='self.arrChartAMCL', color='red', linestyle='-')
            ax.plot(self.arrChartTime, self.arrChartOdom, label='self.arrChartOdom', color='blue', linestyle='-')
            ax.plot(self.arrChartTime, self.arrChartFilteredOdom, label='self.arrChartFilteredOdom', color='green', linestyle='-')
            ax.plot(self.arrChartTime, self.arrChartGPS, label='self.arrChartGPS', color='purple', linestyle='-')
            #ax.plot(self.arrChartTime, self.arrChartFilteredGPS, label='self.arrChartFilteredGPS', color='orange', linestyle='-')
            #ax.plot(self.arrChartTime, self.arrChartGT, label='self.arrChartGT', color='brown', linestyle='-')

            # Add legend
            ax.legend(loc='upper left')

            # Set labels for X and Y axes
            ax.set_xlabel('Time')
            ax.set_ylabel('Values')

            # Show the plot
            plt.show()

            self.arrChartTime         = []
            self.arrChartAMCL         = []
            self.arrChartOdom         = []
            self.arrChartFilteredOdom = []
            self.arrChartGPS          = []
            self.arrChartFilteredGPS  = []
            #self.arrChartGT           = []    

    # ------

    def navigation_callback(self):

        #print("*** PathFollower.navigation_callback()")

        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        bStatusChanged = False

        if(self.initial_pose_received == False):
            return

        if(self.navStatus == "stop"):
            self.drive(0, 0, 0)    # stop
            self.navStatus = "idle"
            bStatusChanged = True
            
            showCharts()

        elif(self.navStatus == "idle"):
            if delta_ms >= 1000:
                print("*** Status: %s" % (self.navStatus))
                
        elif(self.navStatus == "start"):
            if(self.bPathSet == False):
                print("Path not set, see set_path_callback()")
                self.navStatus = "idle"

                # Publish progress  
                # msg = GoalStatus()
                # msg.progress = progress
                # self.progress_pub.publish(msg)
            else:
                # if(self.markers is not None):
                #     self.markers = None
                
                self.prev_pose = self.ground_truth_pose

                self.markers = self.publish_path_markers(self.path.poses)

                # ---

              

                # --- Uncomment to see a single RViz marker               
                # marker = self.create_path_marker(0., 0., 0., 0.0, 255., 0., 1.0)
                # self.marker_pub.publish(marker)

                # --- Uncomment to delete marker (if you leave the code here,
                # marker will be deleted instantly)
                #marker.action = Marker.DELETE
                #self.marker_pub.publish(marker)                


                # # Call service 
                # delete_model_client = node.create_client(DeleteModel, '/gazebo/delete_model')

                # req = DeleteModel.Request()
                # req.model_name = 'marker1' # Delete single model by name

                # # Or delete all models starting with 'marker'
                # req.model_name = 'marker*'  

                # future = delete_model_client.call_async(req)
                # future.result()

                # def publish_markers(self, markers):
                
                #     # Create marker array
                #     marker_array = MarkerArray()

                #     # Add individual markers 
                #     for marker in markers:
                #     marker_array.markers.append(marker)

                #     # Publish marker array
                #     self.marker_pub.publish(marker_array)                

                # --- Uncomment to see Gazebo marker
                # gz_marker_command = (
                #     "gz marker -m 'action: ADD_MODIFY, type: SPHERE, id: 9, scale: {x: 0.2, y: 0.4, z: 1.2}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}'"
                # )
                # # print("****** ", gz_marker_command)
                # # Use subprocess to run the command
                # try:
                #     subprocess.run(gz_marker_command, shell=True, check=True)
                #     print("gz marker command executed successfully.")
                # except subprocess.CalledProcessError as e:
                #     print(f"Error executing gz marker command: {e}")                 
                
                self.bNearFinish = False
                self.target_pose = Pose()
                self.dDistanceToFinish = float("inf")

                self.navStatus = "calculate"

            bStatusChanged = True

        elif(self.navStatus == "calculate"):
            # 1. Find closest segment end
            dMinDist = float("inf")
            if(self.nCurrentSegment == self.nTotalSegments - 1):
                nMinIdx = self.nCurrentSegment
            else:
                nMinIdx = self.nCurrentSegment + 1 

            for nIdx in range(self.nCurrentSegment, self.nTotalSegments):
                dDistance = self.getDistanceBetweenPoses(self.current_pose, self.path.poses[nIdx].pose)
                if(dDistance <= dMinDist):
                    dMinDist = dDistance
                    nMinIdx = nIdx
                    self.nCurrentSegment = nMinIdx - 1
                else:   # Break as soon as distance start increasing
                    break

            # 2. From closest segment end, scan for first segment ending outside of lookahead range 
            bFound = False
            for nIdx in range(nMinIdx, self.nTotalSegments):
                dDistance = self.getDistanceBetweenPoses(self.current_pose, self.path.poses[nIdx].pose)
                if(dDistance >= self.lookaheadDistance):
                    bFound = True
                    break
            else:
                self.bNearFinish = True

            #print("2. bFound: ", bFound, ", nMinIdx: ", nMinIdx, ", dDistance: ", dDistance)
            
            # 3. On the first segment ending outside of lookahead range,
            # find a point that is self.lookaheadDistance away. That will give us 2 points
            # of intersection of circle with segment, we need one closer to segment end. 
            # Note: nIdx > 0, as we always start with robot at 0th point of a route
            if(bFound == False):
                # print(">>> bFound == False !!! <<<")
                self.target_pose = self.closest_point_on_segment(
                    self.path.poses[nIdx - 1].pose.position.x, self.path.poses[nIdx - 1].pose.position.y, 
                    self.path.poses[nIdx].pose.position.x, self.path.poses[nIdx].pose.position.y, 
                    self.current_pose.position.x, self.current_pose.position.y)
            else:
                pt_1 = self.path.poses[nIdx - 1].pose
                pt_2 = self.path.poses[nIdx].pose

                pt = Pose()

                dDistance = self.getDistanceBetweenPoses(self.current_pose, pt_1)
                if(dDistance >= self.lookaheadDistance):
                    #print(">>> dDistance >= self.lookaheadDistance !!! <<<")
                    pt = pt_2
                else:
                    while(True):
                        pt.position.x = (pt_1.position.x + pt_2.position.x) / 2
                        pt.position.y = (pt_1.position.y + pt_2.position.y) / 2

                        dDistance = self.getDistanceBetweenPoses(self.current_pose, pt)
                        dPlus = self.lookaheadDistance + self.dLookaheadAccuracy
                        dMinus = self.lookaheadDistance - self.dLookaheadAccuracy
                        if(dDistance >= dMinus and dDistance <= dPlus):
                            #print("3: dDistance: ", dDistance)
                            break
                        elif(dDistance <= dMinus):
                            pt_2 = pt
                        else:
                            pt_1 = pt

                self.target_pose = pt
                #print("3: pt: ", pt.position.x, pt.position.y)

            # 4. Turn the robot towards target point
            alpha = math.atan2((self.target_pose.position.y - self.current_pose.position.y),
                (self.target_pose.position.x - self.current_pose.position.x))
            
            _, _, beta = self.euler_from_quaternion(self.current_pose.orientation.x, 
                self.current_pose.orientation.y, self.current_pose.orientation.z, 
                self.current_pose.orientation.w)

            dAngleToTurnRobot = alpha - beta

            nSign = 1 if dAngleToTurnRobot >= 0 else -1
            if(dAngleToTurnRobot > math.pi or dAngleToTurnRobot < -math.pi):
                dAngleToTurnRobot = -1 * nSign * (2 * math.pi - abs(dAngleToTurnRobot))

            # print("[", nMinIdx, ", ", self.nCurrentSegment, ", ", nIdx, 
            #     "]: (", self.current_pose.position.x, self.current_pose.position.y, "); ",
            #     "(", self.target_pose.position.x, self.target_pose.position.y, ")",
            #     math.degrees(alpha), math.degrees(beta), math.degrees(alpha - beta))

            # At this point, dDistance holds value from cycle above: current to target
            dSuggestedLinearSpeed = self.linear_speed
            dSuggestedAngularSpeed = self.angular_speed

            if(abs(dAngleToTurnRobot) > math.pi / 2.0):
                dSuggestedLinearSpeed = 0.0
            elif(dDistance != 0):
                dSpeedCoef = 1.0

                dCurvature = abs(dAngleToTurnRobot / dDistance)
                if(dCurvature < 0.3):
                    dSpeedCoef = 1.0
                elif(dCurvature < 0.6):
                    dSpeedCoef = 0.9
                elif(dCurvature < 1.2):
                    dSpeedCoef = 0.75
                else:
                    dSpeedCoef = 0.5

                dSuggestedLinearSpeed = dSuggestedLinearSpeed * dSpeedCoef

                dTimeEstimate = dDistance / dSuggestedLinearSpeed
                if(dTimeEstimate != 0):
                    dSuggestedAngularSpeed = min(self.angular_speed, 
                        2 * abs(dAngleToTurnRobot) / dTimeEstimate)
                else:
                    dSuggestedAngularSpeed = 0

                #dSuggestedAngularSpeed = dSuggestedAngularSpeed * dSpeedCoef 
                


                # print("Angle:", dAngleToTurnRobot, "; Distance:", dDistance, 
                #     ";Curvature:", dCurvature, "; Speed:", dSuggestedLinearSpeed)

            self.drive(dSuggestedLinearSpeed, 
                dSuggestedAngularSpeed, 
                #self.angular_speed, 
                dAngleToTurnRobot)
            self.navStatus = "driving"
            bStatusChanged = True

        # TBD: revisit
        elif(self.navStatus == "driving"):

            dDistance = self.getDistanceBetweenPoses(self.current_pose, self.target_pose)
            # print("Distance: ", dDistance)

            if(self.bNearFinish == False
                and (delta_ms >= 100    # TBD: check this logic
                    or
                    dDistance <= (self.lookaheadDistance - self.dLookaheadAccuracy))
                and self.nCurrentSegment < self.nTotalSegments - 1):
                self.navStatus = "calculate"
                bStatusChanged = True
            # elif(dDistance <= self.dLookaheadAccuracy):
            #     self.navStatus = "stop"
            #     bStatusChanged = True
            elif(self.bNearFinish == True): #delta_ms >= 1000):
                self.navStatus = "calculate"
                bStatusChanged = True

                dDistanceToFinish = \
                    self.getDistanceBetweenPoses(self.current_pose, self.path.poses[-1].pose)

                if(dDistanceToFinish <= self.dDistanceToFinish):
                    self.dDistanceToFinish = dDistanceToFinish
                else:
                    self.navStatus = "stop"
                    bStatusChanged = True                        

                #print("Distance to Finish: ", dDistanceToFinish)

            if(self.navStatus == "driving" or self.navStatus == "calculate"):
                dRobotShift = self.getDistanceBetweenPoses(self.prev_pose, self.ground_truth_pose)
                # print("$$$", self.ground_truth_pose.position.x, self.ground_truth_pose.position.y,
                #     self.prev_pose.position.x, self.prev_pose.position.y, dRobotShift,
                #     self.dDistBetweenRobotMarkers)

                if(dRobotShift >= self.dDistBetweenRobotMarkers):
                    prev_x = int(self.img_width * (self.prev_pose.position.x 
                        + self.grid_size/2) / self.grid_size)
                    prev_y = int(self.img_height * (self.grid_size/2 - self.prev_pose.position.y)  / self.grid_size)

                    x = int(self.img_width * (self.ground_truth_pose.position.x + self.grid_size/2) / self.grid_size)
                    y = int(self.img_height * (self.grid_size/2 - self.ground_truth_pose.position.y)  / self.grid_size)

                    cv2.line(self.imgCanvas, (prev_x, prev_y), (x, y), self.real_path_color, 2)

                    self.prev_pose = self.ground_truth_pose

                    msg = self.bridge.cv2_to_imgmsg(self.imgCanvas, "bgra8")
                    msg.header.frame_id = str(self.strSlashRobotName + '/images/robot_path_markers')
                    self.frame_num += 1        
                    self.canvas_publisher.publish(msg)                

                    # cv2.imwrite("./transparent_img.png", self.imgCanvas)

        # ---

        # if(bStatusChanged):
        #     print("*** Status: %s" % (self.navStatus))

        if delta_ms >= 1000:
            self.prev_time = now

    # ------

    def getAngleBetweenPoses(self, start, goal):
    
        roll1, pitch1, yaw1 = self.euler_from_quaternion(start.orientation.x, 
            start.orientation.y, start.orientation.z, start.orientation.w)
        roll2, pitch2, yaw2 = self.euler_from_quaternion(goal.orientation.x, 
            goal.orientation.y, goal.orientation.z, goal.orientation.w)
        
        angle = yaw2 - yaw1

        #print("### Angle between poses: ", math.degrees(yaw1), math.degrees(yaw2), math.degrees(yaw2 - yaw1))
        
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi

        return angle      

    # ------ 

    def getDistanceBetweenPoses(self, start, goal):
        
        dx = goal.position.x - start.position.x
        dy = goal.position.y - start.position.y
        dz = goal.position.z - start.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance     

    # --- TBD: move this function to common access area

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians 

    # ------

    def create_path_marker(self, x, y, z, r, g, b, a=1.0):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = Duration().to_msg() # forever
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = r / 255.0
        marker.color.g = g / 255.0 
        marker.color.b = b / 255.0
        marker.color.a = a

        return marker

    # ------

    def publish_path_markers(self, path_points):

        markers = MarkerArray()

        for i, point in enumerate(path_points):
            #print("Processing point ", i)

            marker = self.create_path_marker(
                point.pose.position.x, point.pose.position.y, point.pose.position.z, 0.0, 255., 0., 0.5)
            marker.id = i
            
            markers.markers.append(marker)

            x = int(self.img_width * (point.pose.position.x + self.grid_size/2)  / self.grid_size)
            y = int(self.img_height * (self.grid_size/2 - point.pose.position.y)  / self.grid_size)

            if(i > 0):
                # print("Draw green line:", (prev_x, prev_y), (x, y))
                if(self.arrColorIds[i] == 0):
                    color = self.path_color_0
                else:
                    color = self.path_color_1

                cv2.line(self.imgCanvas, (prev_x, prev_y), (x, y), color, 5)
                
            prev_x = x
            prev_y = y

        msg = self.bridge.cv2_to_imgmsg(self.imgCanvas, "rgba8")
        msg.header.frame_id = str(self.frame_num)   # 'msg' is a ROS2 sensor_msgs/Image.msg
        self.frame_num += 1        
        self.canvas_publisher.publish(msg)
        
    
    # ------

def main(args=None):
    print("*** PathFollower - main()")

    rclpy.init(args=args)
    path_follower = PathFollower()

# ---

if __name__ == '__main__':
    main()