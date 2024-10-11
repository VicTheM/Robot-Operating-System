# (c) robotics.snowcron.com
# Use: MIT license


import time  # Time library
import math
from rclpy.duration import Duration
import rclpy                                # Python client library for ROS 2
from rclpy.node import Node
from BasicNavigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped   # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist         # Velocity command
from sensor_msgs.msg import BatteryState    # Battery status
import time
from datetime import datetime

from rclpy.qos import QoSPresetProfiles
# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

#from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

import tf2_ros
import transformations as tf

from geometry_msgs.msg import PoseWithCovarianceStamped
import copy

# class BatteryStatusPublisher(Node):
#     def __init__(self):
#         super().__init__('BatteryStatusPublisher')
#         self.subscription_battery_status_not_used_except_for_scope = self.create_subscription(
#             BatteryState,
#             '/battery_status',
#             self.get_battery_status,
#             10)

#     def get_battery_status(self, msg: BatteryState):
#         self.nBatteryLevel = msg.percentage
#         print("Battery at", self.nBatteryLevel, "%")

_EPS = np.finfo(float).eps * 4.0

class ChargerNavigator(Node):
    def __init__(self, strRobotName = ""):
        if(strRobotName != ""):
            super().__init__(strRobotName + '_ChargerNavigator')
            self.strSlashRobotName = "/" + strRobotName
            self.strRobotNameSlash = strRobotName + "/"
        else:
            super().__init__('ChargerNavigator')
            self.strSlashRobotName = ""
            self.strRobotNameSlash = ""

        self.strRobotName = strRobotName

        # ---

        self.nBatteryLevel = 100.
        # See navigation_bot_09 article for alternative approaches
        self.navStatus = "not_started"

        self.navigator = BasicNavigator(strRobotName)

        self.aruco_x = None
        self.aruco_y = None
        self.aruco_z = None
        self.aruco_roll = None
        self.aruco_pitch = None
        self.aruco_yaw = None

        self.aruco_distance = None
        self.aruco_angle = None

        self.lastImage = None

        self.aruco_end_move_time = None
        self.aruco_end_angle = None
        
        self.aruco_prev_yaw = 360
        self.bStopRotating = False

        self.angular_speed = 0.2

        # robot1/base_link
        self.robot_base_frame = self.strRobotNameSlash + 'base_link'
        self.camera_frame = self.strRobotNameSlash + 'camera_link'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)   
        #trans = tf2_ros.TransformStamped()

        self.tf_buffer.can_transform(self.robot_base_frame, self.camera_frame, 
            rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))

        self.printedOnce = ""

        # --- <aruco_related>

        # Define the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Define the camera matrix and distortion coefficients (replace with your own values)
        # TBD: this info can be used in other places, so it should be moved to common area.
        # self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        # self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        # self.focal_length = None
        # self.optical_center = None
        # self.distortion_coeffs = None        

        # This are parameters extracted from camera.xacro, quick and dirty.
        self.camera_matrix = np.array([[528.43375656,   0.        , 320.5       ],
            [  0.        , 528.43375656, 240.5       ],
            [  0.        ,   0.        ,   1.        ]], dtype=np.float32)
   
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)        

        # Define the dimensions of the ArUco marker in meters (replace with your own value)
        self.aruco_marker_size = 0.2

        # --- </aruco_related>

        # We are going to publish this value. Then we are going to subscribe to it as 
        # to self.nBatteryLevel. In a real robot, this will be published by node that
        # reads the charge of a real battery.
        self.nBatteryLevelCalculated = 100.
        self.nBatteryLowTreshold = 95.

        # After we done recharging, continue from this waypoint
        self.nNextWaypointIdx = 0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.navigation_callback)
        time = self.navigator.get_clock().now().to_msg()
        # Used to publish status
        self.prev_time = datetime.now()

        # Subscribe to the camera topic (replace with your own topic name)
        camera_pos_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)        
        
        # Subscribe to messages of type nav_msgs/Odometry that provides position and orientation of the robot
        # self.odom_subscriber = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     10)

        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo,
        #     '/camera/camera_info',
        #     self.camera_info_callback,
        #     qos_profile=camera_pos_qos)
                
        self.subscription_camera_image = self.create_subscription(
            Image, 
            self.strSlashRobotName + '/camera/image_raw', 
            self.camera_callback, 
            qos_profile=camera_pos_qos)

        self.subscription_battery_status = self.create_subscription(
            BatteryState,
            self.strSlashRobotName + '/battery_status',
            self.get_battery_status,
            10)
        
        self.publisherBatteryLevel = self.create_publisher(BatteryState, 
            self.strSlashRobotName + '/battery_status', 10)
        #self.image_pub = self.create_publisher(Image, '/aruco_image', 10)

        # Publish the desired linear and angular velocity of the robot (in the
        # robot chassis coordinate frame) to the /cmd_vel_unstamped topic. 
        # The diff_drive controller that our robot uses will read this topic 
        # and move the robot accordingly.
        self.publisherCmdVel = self.create_publisher(
            Twist, #TwistStamped, 
            #'/diff_cont/cmd_vel_unstamped' + self.strSlashRobotName,  # For ROS2 diff. drive controller
            self.strSlashRobotName + "/cmd_vel",                       # For Gazebo diff. drive controller
            10)
        
        self.goal_charger = None
        self.goal_poses = []

    # ---

    def setGoalPoses(self, arrPoses):
        self.goal_poses = copy.deepcopy(arrPoses)

        for pose in self.goal_poses:
            if self.strRobotName != "":
                pose.header.frame_id = self.strRobotName + '/map' 
            else:
                pose.header.frame_id = 'map'

    # ---

    def setChargerPos(self, goal_charger):
        self.goal_charger = copy.deepcopy(goal_charger)
        
        if self.strRobotName != "":
            self.goal_charger.header.frame_id = self.strRobotName + '/map' 
        else:
            self.goal_charger.header.frame_id = 'map'
        

    # ---

    def amcl_callback(self, msg):
        if(self.aruco_yaw != None):
            self.aruco_prev_yaw = self.aruco_yaw
        
        # Access position and orientation information from the message
        self.aruco_x = msg.pose.pose.position.x
        self.aruco_y = msg.pose.pose.position.y
        #self.aruco_z = msg.pose.pose.position.z
        self.aruco_roll, self.aruco_pitch, self.aruco_yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        #self.aruco_roll = np.degrees(self.aruco_roll)
        #self.aruco_pitch = np.degrees(self.aruco_pitch)
        self.aruco_yaw = np.degrees(self.aruco_yaw)

        # We have [0,180] or [-180,0]. We want [0,360]
        if(self.aruco_yaw < 0):
            self.aruco_yaw = 360 + self.aruco_yaw
        
        if(self.aruco_end_angle != None and self.aruco_yaw != None and self.aruco_prev_yaw != None):
            if((self.aruco_end_angle - self.aruco_yaw) * (self.aruco_end_angle - self.aruco_prev_yaw) < 0):
                self.bStopRotating = True
            
        #print(">>> *** >>>", self.aruco_x, self.aruco_y, self.aruco_z, np.degrees(self.aruco_yaw))
        #print(">>>>> amcl_callback: ", self.aruco_x, self.aruco_y, self.aruco_yaw)

    def get_battery_status(self, msg: BatteryState):
        self.nBatteryLevel = msg.percentage

    def publishBatteryStatus(self):
        self.nBatteryLevelCalculated -= 0.01
        msg = BatteryState()
        msg.percentage = self.nBatteryLevelCalculated
        self.publisherBatteryLevel.publish(msg)

    def quaternion_matrix(self, qw, qx, qy, qz):
        q = np.array([qw, qx, qy, qz], dtype=np.float64)
        n = np.dot(q, q)
        if n < _EPS:
            return np.identity(4)
        q *= math.sqrt(2.0 / n)
        q = np.outer(q, q)
        return np.array([
            [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
            [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
            [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
            [                0.0,                 0.0,                 0.0, 1.0]])

    def getDistanceToAruco(self, image):
        # Convert the image to grayscale

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect the ArUco markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Define the world coordinates of the four corners of the marker
            # objp = np.array([[-self.aruco_marker_size/2, self.aruco_marker_size/2, 0], 
            #                 [self.aruco_marker_size/2, self.aruco_marker_size/2, 0], 
            #                 [self.aruco_marker_size/2, -self.aruco_marker_size/2, 0], 
            #                 [-self.aruco_marker_size/2, -self.aruco_marker_size/2, 0]], dtype=np.float32)
                    
            # Check if ArUco marker is detected
            # Estimate the pose of the marker in the camera frame
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size,
                self.camera_matrix, self.dist_coeffs)

            # Calculate the position of the marker in the camera frame
            marker_pos_in_camera = np.array([tvecs[0][0][0], 0, tvecs[0][0][2]], dtype=np.float32)

            # Get the transform from the robot base frame to the camera frame
            try:
                tf = self.tf_buffer.lookup_transform(self.robot_base_frame, self.camera_frame, 
                    rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    'Failed to lookup transform from %s to %s: %s' % (self.robot_base_frame, self.camera_frame, str(e)))
                return None

            # Calculate the position of the camera in the robot base frame
            camera_pos_in_robot = np.array([tf.transform.translation.x, 
                tf.transform.translation.y, tf.transform.translation.z], dtype=np.float32)

            # Calculate the rotation matrix from the camera frame to the robot base frame
            q = [tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z]
            R = self.quaternion_matrix(q[0], q[1], q[2], q[3])
            R = R[:3, :3]

            # Transform the marker position from the camera frame to the robot base frame
            marker_pos_in_robot = np.dot(R, marker_pos_in_camera) + camera_pos_in_robot

            # Calculate the distance from the camera to the point at the base of the wall where the center of the ArUco marker projects
            marker_pos_in_robot[1] = 0  # Set the y-coordinate to 0 to get the projection on the horizontal plane
            distance = math.sqrt(marker_pos_in_robot[0] ** 2 + marker_pos_in_robot[2] ** 2)

            return distance
        

        else:
            self.arucoRotateByTime(0, 0) # Stop rotating
            exit()   
    
    def getDistanceToArucoNorm(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect the ArUco markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:

            # Estimate the pose of the marker in the camera frame
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size,
                self.camera_matrix, self.dist_coeffs)

            R, _ = cv2.Rodrigues(rvecs[0])

            # marker's projection in the marker frame
            marker_proj_marker_frame = np.array([[-self.aruco_marker_size/2, 0, 0], 
                [self.aruco_marker_size/2, 0, 0]], dtype=np.float32)
            
            norm_to_marker_proj_marker_frame = np.array([[0, 0, self.aruco_marker_size/2], 
                [0, 0, -self.aruco_marker_size/2]], dtype=np.float32)

            # concatenate rotation matrix and translation vector to form homogeneous transformation matrix
            homog_transf = np.eye(4)
            homog_transf[:3, :3] = R
            homog_transf[:3, 3] = tvecs.squeeze()

            # apply homogeneous transformation matrix to marker's projection in the marker frame
            marker_proj_camera_frame = homog_transf.dot(np.vstack([marker_proj_marker_frame.T, 
                np.ones((1, marker_proj_marker_frame.shape[0]))])).T[:, :3]
            
            norm_to_marker_proj_camera_frame = homog_transf.dot(np.vstack([norm_to_marker_proj_marker_frame.T, 
                np.ones((1, norm_to_marker_proj_marker_frame.shape[0]))])).T[:, :3]
            
            # ---

            # Extract the forward vector of the robot from the transform
            trans = self.tf_buffer.lookup_transform(self.robot_base_frame, self.camera_frame, 
                    rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            roll, pitch, yaw = self.euler_from_quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
            robot_forward_vector = np.array([math.cos(yaw), 0, math.sin(yaw)], dtype=np.float32)            

            cam_to_robot = np.eye(4)
            cam_to_robot[:3, :3] = tf.transformations.euler_matrix(roll, pitch, yaw)[:3, :3]
            cam_to_robot[:3, 3] = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z], dtype=np.float32)

            marker_proj_cam = np.array([marker_proj_camera_frame[0, 0], 0, marker_proj_camera_frame[0, 2], 1], dtype=np.float32)
            norm_to_marker_proj_cam_1 = np.array([norm_to_marker_proj_camera_frame[0, 0], 
                0, norm_to_marker_proj_camera_frame[0, 2], 1], dtype=np.float32)
            norm_to_marker_proj_cam_2 = np.array([norm_to_marker_proj_camera_frame[1, 0], 
                0, norm_to_marker_proj_camera_frame[1, 2], 1], dtype=np.float32)            

            marker_proj_robot = np.dot(cam_to_robot, marker_proj_cam)[:3]
            norm_to_marker_proj_robot_1 = np.dot(cam_to_robot, norm_to_marker_proj_cam_1)[:3]
            norm_to_marker_proj_robot_2 = np.dot(cam_to_robot, norm_to_marker_proj_cam_2)[:3]

            # ---

            # Calculate the distance between the line passing through norm_to_marker_proj_marker_frame and the robot
            x1, y1, z1 = norm_to_marker_proj_robot_1
            x2, y2, z2 = norm_to_marker_proj_robot_2
            x0, y0, z0 = (0,0,0) #marker_proj_robot

            distance = abs((z2 - z1) * x0 - (x2 - x1) * z0 + x2 * z1 - z2 * x1) / math.sqrt((z2 - z1) ** 2 + (x2 - x1) ** 2)

            return distance  

    def getAngleToArucoNorm(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect the ArUco markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Define the world coordinates of the four corners of the marker
            # marker_corners = np.array([[-self.aruco_marker_size/2, self.aruco_marker_size/2, 0], 
            #                 [self.aruco_marker_size/2, self.aruco_marker_size/2, 0], 
            #                 [self.aruco_marker_size/2, -self.aruco_marker_size/2, 0], 
            #                 [-self.aruco_marker_size/2, -self.aruco_marker_size/2, 0]], dtype=np.float32)

            # Estimate the pose of the marker in the camera frame
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size,
                self.camera_matrix, self.dist_coeffs)

            R, _ = cv2.Rodrigues(rvecs[0])
            #in markerworld the corners are all in the xy-plane so z is zero at first
            X = self.aruco_marker_size * R[:,0] #rotate the x = mhalf
            Z = self.aruco_marker_size * R[:,2] #rotate the y = mhalf
            minusX = X * (-1)
            minusZ = Z * (-1)

            # calculate 4 corners of the marker in camworld. corners are enumerated clockwise
            markercorners = []
            markercorners.append(np.add(minusX, Z)) #was upper left in markerworld
            markercorners.append(np.add(X, Z)) #was upper right in markerworld
            markercorners.append(np.add( X, minusZ)) #was lower right in markerworld
            markercorners.append(np.add(minusX, minusZ)) #was lower left in markerworld
            # if tvec given, move all by tvec
            C = tvecs[0] #center of marker in camworld
            for i, mc in enumerate(markercorners):
                markercorners[i] = np.add(C,mc) #add tvec to each corner
            #print('Vec X, Y, C, dot(X,Y)', X,Y,C, np.dot(X,Y)) # just for debug
            markercorners = np.array(markercorners,dtype=np.float32) 
            
            marker_corners_cam = markercorners

            # ---

            upper_left_cam = marker_corners_cam[0]
            upper_right_cam = marker_corners_cam[1]
            marker_vector_cam = upper_left_cam - upper_right_cam

            camera_vector = np.array([1, 0, 0])
            angle_rad = np.arccos(np.dot(camera_vector, marker_vector_cam[0]) / (np.linalg.norm(camera_vector) * np.linalg.norm(marker_vector_cam)))
            angle_deg = np.degrees(angle_rad)

            if(angle_deg > 90):
                angle_deg = angle_deg - 90
            if(angle_deg < -90):
                angle_deg = angle_deg + 90

            self.turnDirection = 1
            if(angle_deg >= 0):
                self.turnDirection = 1
            else:
                self.turnDirection = -1                

            return angle_deg           



    # ---

    # ---

    # def odom_callback(self, msg):
    #     """
    #     Receive the odometry information containing the position and orientation
    #     of the robot in the global reference frame. 
    #     The position is x, y, z.
    #     The orientation is a x,y,z,w quaternion. 
    #     """                    
    #     roll, pitch, yaw = self.euler_from_quaternion(
    #         msg.pose.pose.orientation.x,
    #         msg.pose.pose.orientation.y,
    #         msg.pose.pose.orientation.z,
    #         msg.pose.pose.orientation.w)
    
    #     self.current_x = msg.pose.pose.position.x
    #     self.current_y = msg.pose.pose.position.y
    #     #self.current_z = msg.pose.pose.position.z
    #     self.current_yaw = yaw

    #     # Publish the estimated state (x position, y position, yaw angle)
    #     # obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]
    #     # self.publish_estimated_state(obs_state_vector_x_y_yaw)
    
    #     #print(">>>>> odom_callback: ", msg.pose.pose.position.x,msg.pose.pose.position.y,yaw) 

    # ---

    # You can use the ros2 topic echo /camera/camera_info command to see if messages are being 
    # published on the /camera/camera_info topic.
    # def camera_info_callback(self, msg):
    #     # extract camera matrix and distortion coefficients from CameraInfo message
    #     # array([[528.43375656,   0.        , 320.5       ],
    #     #     [  0.        , 528.43375656, 240.5       ],
    #     #     [  0.        ,   0.        ,   1.        ]])        
    #     self.camera_matrix = np.array(msg.k).reshape((3, 3))
    #     # [0:5] : [0.0, 0.0, 0.0, 0.0, 0.0]
    #     self.dist_coeffs = np.array(msg.d)

    # Define the callback function for the ROS2 camera subscriber
    def camera_callback(self, msg):
        # Convert the ROS2 image message to a numpy array
        bridge = CvBridge()
        self.lastImage = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        #cv2.imshow('frame', image)
        #cv2.waitKey(1)
            
    # ---

    def arucoRotateByTime(self, angleDegrees, angular_speed):
        angleRadians = math.radians(angleDegrees)
        time_to_turn = 0 if angular_speed == 0 else abs(angleRadians / angular_speed)
        
        if angular_speed == 0:
            self.aruco_end_angle = None
            self.aruco_end_move_time = None
        else:
            self.aruco_end_move_time = self.get_clock().now() + rclpy.time.Duration(seconds=time_to_turn)
        
        if(angleDegrees < 0):
            angular_speed *= -1

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_speed)
         
        self.bStopRotating = False
        self.publisherCmdVel.publish(msg)

    # ---

    def arucoRotateByAngle(self, angleDegrees, angular_speed):
        nMultiplier = 1

        if angular_speed == 0:
            self.aruco_end_angle = None
            self.aruco_end_move_time = None
        else:
            self.aruco_end_angle = self.aruco_yaw + angleDegrees

            if(self.aruco_end_angle > 360):
                self.aruco_end_angle = self.aruco_end_angle - 360
                nMultiplier = -1

            elif(self.aruco_end_angle < 0):
                self.aruco_end_angle = 360 + self.aruco_end_angle
                nMultiplier = -1
                    
        if(angular_speed != 0 and self.aruco_yaw > self.aruco_end_angle):
            angular_speed *= -1
        angular_speed *= nMultiplier

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_speed)
         
        self.bStopRotating = False

        # print(">>>>> amcl_callback: ", self.aruco_x, self.aruco_y, self.aruco_yaw)

        self.publisherCmdVel.publish(msg)

    # ---

    def arucoDrive(self, distance, speed):
        distance = abs(distance)
        time_to_drive = 0 if speed == 0 else abs(distance / speed)

        if speed == 0:
            self.aruco_end_move_time = None 
        else:
            self.aruco_end_move_time = self.get_clock().now() + rclpy.time.Duration(seconds=time_to_drive)        

        msg = Twist()
        msg.linear.x = float(speed)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
         
        self.publisherCmdVel.publish(msg)

    
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

    # ---

    def printOnce(self, str):
        if(self.printedOnce != str):
            print(str)
        self.printedOnce = str

    # ---

    def getDirectLineDockingAction(self):
        # The closer we are the slower we go, so we need distance
        # As an alternative, we can move until robot either bumps 
        # into charger (use bumper to stop) or touches charger's contacts.
        self.aruco_distance = self.getDistanceToAruco(self.lastImage)
        if(self.aruco_distance is None):
            return None, None
        nDistanceToArucoNorm = self.getDistanceToArucoNorm(self.lastImage)

        #print(self.aruco_distance, nDistanceToArucoNorm)

        # If we have arrive
        if(self.aruco_distance <= 1.0):
            return 0.0, 0.0     # Stop

        # Convert the image to grayscale
        gray = cv2.cvtColor(self.lastImage, cv2.COLOR_BGR2GRAY)

        # Detect the ArUco markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Find if we are on the left or on the right of a marker
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]

            # If left and right sides of marker are approx. same length, robot is in front of it
            xMid = (topRight[0] + topLeft[0]) / 2  # Mid. of image
            if(self.lastImage.shape[1] / 2 < xMid):
                return 0.5, -0.1
            else:
                return 0.5, 0.1
        
        return None, None

    # ---

    def arucoRotateAndDrive(self, nLinearSpeed, nAngularSpeed):
        msg = Twist()
        msg.linear.x = nLinearSpeed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = nAngularSpeed
         
        self.publisherCmdVel.publish(msg)    

    # ---        
       
    def navigation_callback(self):
        self.publishBatteryStatus()
        
        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        if delta_ms >= 2000:
            print(">>> %s battery at %.0d" % (self.strRobotName, self.nBatteryLevel))
            self.prev_time = now

        if(self.nBatteryLevel <= 0):
            self.navStatus = "out_of_charge"
        elif(self.navStatus == "not_started"):
        
            # Activate navigation, if not autostarted. This should be called after setInitialPose()
            # or this will initialize at the origin of the map and update the costmap with bogus readings.
            # If autostart, you should `waitForNodesToStart()` instead.
            # self.navigator.lifecycleStartup()
            
            # Wait for navigation to fully activate. Use this line if autostart is set to true.
            self.navigator.waitForNodesToStart()

            # wait for the first callback in BasicNavigator
            #self.navigator.get_localization_pose_sub()
            # ---
        
            self.localization_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.strSlashRobotName + '/amcl_pose', self.amcl_callback, self.navigator.amcl_pose_qos)
            
        
            # ---
      
            # If desired, you can change or load the map as well
            # self.navigator.changeMap('/path/to/map.yaml')
            
            # You may use the self.navigator to clear or obtain costmaps
            # self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
            # global_costmap = self.navigator.getGlobalCostmap()
            # local_costmap = self.navigator.getLocalCostmap()

            self.navStatus = "getting_next_task"

        # Robot is done with current task
        # "coordinator" will assign a task and change
        elif(self.navStatus == "getting_next_task"):
            if delta_ms >= 2000:
                print("%s waiting for the next task" % (self.strRobotName))

        elif((self.navStatus == "ready" or self.navStatus == "navigating"
            or self.navStatus == "getting_next_task") 
            and self.nBatteryLevel <= self.nBatteryLowTreshold):
            
            print("%s battery low" % (self.strRobotName))
            self.navStatus = "preparing_to_go_to_charger"
            self.navigator.cancelNav()

        elif(self.navStatus == "ready"):
            # sanity check a valid path exists
            # path = self.navigator.getPathThroughPoses(initial_pose, self.goal_poses)
            self.navigator.followWaypoints(self.goal_poses[self.nNextWaypointIdx:])
            self.navStatus = "navigating"            

        elif(self.navStatus == "navigating"):
            if(not self.navigator.isNavComplete()):
                feedback = self.navigator.getFeedback()
                self.nNextWaypointIdx = feedback.current_waypoint
                
                if feedback and delta_ms >= 2000:
                    print(">>> ", self.strRobotName, ' executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses)))
            else:
                # This makes sure we do navigation in cycles, infinitely
                self.navStatus = "getting_next_task" #"completed"

        elif(self.navStatus == "preparing_to_go_to_charger"):
            print(">>> ", self.strRobotName, ' cancelling nav. before going to charger')
            if(self.navigator.isNavComplete()):
                self.navStatus = "going_to_charger_waypoint"
                self.navigator.goToPose(self.goal_charger)   

        elif(self.navStatus == "going_to_charger_waypoint"):
            self.printOnce(">>> " + self.strRobotName + " going to charger waypoint")
            if(self.navigator.isNavComplete()):
                self.navStatus = "looking_for_charger"
        
        # Here we turn around until aruco marker is detected
        elif(self.navStatus == "looking_for_charger"):
            self.printOnce(">>> " + self.strRobotName + " looking for charger")
            self.aruco_angle = self.getAngleToArucoNorm(self.lastImage)
            self.aruco_distance = self.getDistanceToArucoNorm(self.lastImage)                                       # 0.06  1.34    2.16
                        
            # We are about to turn 360 degrees. We have to use arucoTurnByTime here.
            if(self.aruco_end_move_time is not None 
                and self.aruco_distance is not None 
                and self.aruco_angle is not None): 
                print(">>> ", self.strRobotName, " found marker. Distance: {:.2f} meters; angle: {:.0f} degrees".
                    format(self.aruco_distance, self.aruco_angle))
                self.arucoRotateByTime(0, 0) # Stop rotating

                self.navStatus = "turning_to_charger_line"
                
            # TBD: due to slippage, this can be imprecise. Replace with getting position from sensors.
            elif(self.aruco_end_move_time is None):                      # Start rotating
                print(">>> ", self.strRobotName, " looking for charger: rotating 360 degrees")

                # 420 instead of 360 to compensate for slippage.
                angle = 420 #360
                self.arucoRotateByTime(angle, self.angular_speed)

            elif(self.aruco_end_move_time < self.get_clock().now()):     # done rotating 360 degrees
                self.arucoRotateByTime(0, 0) # Stop rotating
                print(">>> ", self.strRobotName, ' unable to find charger')
                self.timer.cancel()
                return False

        elif(self.navStatus == "turning_to_charger_line"):
            if(self.aruco_end_angle is None): 
                # and self.aruco_distance is not None and self.aruco_angle is not None):
                # if(self.aruco_x > 0):
                #     angle = self.aruco_angle - 90
                # else:
                #     angle = self.aruco_angle + 90
                angle = self.aruco_angle
                print(">>> ", self.strRobotName, " turning to charger line. Turning {:.0f} degrees".format(angle))
                self.arucoRotateByAngle(angle, self.angular_speed)

            # Check if we passed through target and getting away from it now
            elif(self.bStopRotating == True):
                print(">>> ", self.strRobotName, " done turning to charger line")
                print(">>> ", self.strRobotName, " end angle: {:.1f}, Yaw: {:.1f}, Prev. yaw: {:.1f}".format(
                    self.aruco_end_angle, self.aruco_yaw, self.aruco_prev_yaw))                
                
                self.arucoRotateByAngle(0, 0) # Stop rotating
                self.navStatus = "go_to_charger_line"

        elif(self.navStatus == "go_to_charger_line"):
            if(self.aruco_end_move_time is None): 
                print(">>> ", self.strRobotName, 
                    " going towards charger line, {:.2f} meters".format(self.aruco_distance))
                self.arucoDrive(self.aruco_distance, 1)
            elif(self.aruco_end_move_time < self.get_clock().now()):
                print(">>> ", self.strRobotName, " done going towards charger line")
                self.arucoDrive(0, 0)
                self.navStatus = "turning_towards_charger"

        elif(self.navStatus == "turning_towards_charger"):
            if(self.aruco_end_angle is None): 
                print(">>> ", self.strRobotName, " turning towards charger")
                if(self.turnDirection > 0):
                    self.arucoRotateByAngle(-90, self.angular_speed)
                else:
                    self.arucoRotateByAngle(90, self.angular_speed)
                
            elif(self.bStopRotating == True):
                print(">>> ", self.strRobotName, " done turning towards charger")
                print(">>> ", self.strRobotName, " end angle: {:.1f}, Yaw: {:.1f}, Prev. yaw: {:.1f}".format(
                    self.aruco_end_angle, self.aruco_yaw, self.aruco_prev_yaw))
                
                self.arucoRotateByAngle(0, 0) # Stop rotating
                self.navStatus = "going_to_charger"
                
        elif(self.navStatus == "going_to_charger"):
            self.aruco_distance = self.getDistanceToAruco(self.lastImage)
            if(self.aruco_distance is None):
                self.arucoDrive(0, 0)
                print(">>> ", self.strRobotName, " lost charger")
                
                return False
            
            # If robot is closer than camera can focus
            if(self.aruco_distance <= 1.0):
                print(">>> ", self.strRobotName, " done going towards charger")
                self.arucoDrive(0, 0)
                self.navStatus = "connecting_to_charger"
            else:
                nLinearSpeed, nAngularSpeed = self.getDirectLineDockingAction()
                if(nLinearSpeed is None):
                    self.arucoDrive(0, 0)
                    print(">>> ", self.strRobotName, " something wrong: Lost charger")
                    return False
                else:
                    self.printOnce(">>> " + self.strRobotName + " going towards charger")
                    self.arucoRotateAndDrive(nLinearSpeed, nAngularSpeed)

        elif(self.navStatus == "connecting_to_charger"): # Attach to wires
            if(self.aruco_end_move_time is None):
                print(">>> ", self.strRobotName, " connecting to charger")
                self.arucoDrive(self.aruco_distance - 0.15, 0.5)
            elif(self.aruco_end_move_time < self.get_clock().now()):
                print(">>> ", self.strRobotName, " connected to charger")
                self.arucoDrive(0, 0)
                self.navStatus = "charging"

        elif(self.navStatus == "charging"):
            if(delta_ms >= 2000):
                if(self.nBatteryLevelCalculated < 95.):
                    self.nBatteryLevelCalculated += 5
                else:
                    self.nBatteryLevelCalculated = 100.
                    self.navStatus = "back_from_charger"

        elif(self.navStatus == "back_from_charger"): # Attach to wires
            if(self.aruco_end_move_time is None):
                print(">>> ", self.strRobotName, " connecting to charger")
                self.arucoDrive(1.0, -0.5)
            elif(self.aruco_end_move_time < self.get_clock().now()):
                print(">>> ", self.strRobotName, " charging cycle completed")
                self.arucoDrive(0, 0)
                self.navStatus = "charging_cycle_completed"                    

        elif(self.navStatus == "charging_cycle_completed"):
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print(">>> ", self.strRobotName, ' goal succeeded!')
                self.navStatus = "ready"
            elif result == NavigationResult.CANCELED:
                print(">>> ", self.strRobotName, ' goal was canceled!')
                self.timer.cancel()
                return False
            elif result == NavigationResult.FAILED:
                print(">>> ", self.strRobotName, ' goal failed!')
                self.timer.cancel()
                return False
            else:
                print(">>> ", self.strRobotName, ' goal has an invalid return status!')
                self.timer.cancel()
                return False
        elif(self.navStatus == "out_of_charge"):
            print(">>> ", self.strRobotName, ' battery level reached 0!')
            self.timer.cancel()
            return False

        return True
         
    # def connect_to_dock(self):  
       
    #     # While the battery is not charging
    #     while this_battery_state.power_supply_status != 1:
      
    #       # Publish the current battery state
    #       self.get_logger().info('NOT CHARGING...')
        
    #       # Send the velocity command to the robot by publishing to the topic
    #       cmd_vel_msg = Twist()
    #       cmd_vel_msg.linear.x = self.linear_velocity
    #       cmd_vel_msg.angular.z = self.angular_velocity
    #       self.publisher_cmd_vel.publish(cmd_vel_msg)      
    #       time.sleep(0.1)
      
    #     # Stop the robot
    #     cmd_vel_msg = Twist()
    #     cmd_vel_msg.linear.x = 0.0
    #     cmd_vel_msg.angular.z = 0.0
    #     self.publisher_cmd_vel.publish(cmd_vel_msg)
      
    #     self.get_logger().info('CHARGING...')
    #     self.get_logger().info('Successfully connected to the charging dock!')
 

def main(args=None):

    try:  
        rclpy.init(args=args)
        
        executor = SingleThreadedExecutor()
        navigator = ChargerNavigator()
        executor.add_node(navigator)

        try:
            #rclpy.spin(navigator)
            executor.spin()
            # while(True):
            #     bResult = navigator.navigation_callback()
            #     if(not bResult):
            #         break
            #     time.sleep(0.1)
        finally:
            executor.shutdown()
            navigator.destroy_node()
 
    finally:
        # Shutdown
        rclpy.shutdown()    
 
 
if __name__ == '__main__':
  main()
