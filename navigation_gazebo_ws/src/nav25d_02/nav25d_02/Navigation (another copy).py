# (c) robotics.snowcron.com
# Based on Kalman and Bayesian Filters in Python, Roger R Labbe Jr
# Use: MIT license

# --- Kalman

import argparse
import sys

import numpy as np
from numpy.random import randn

import copy

import scipy.stats as stats
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation as R

from math import tan, sin, cos, sqrt
from math import cos, sin, atan2, pi

import matplotlib.pyplot as plt

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import JulierSigmaPoints
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from filterpy.stats import plot_covariance_ellipse

# --- My Kalman

from nav25d_02.kalman import KalmanNav
#import threading
import time
from datetime import datetime

#from rclpy.time import Time

# GPS projections
#import pyproj
from geopy import distance
from geopy.point import Point

# --- ROS2

import rclpy
from rclpy.node import Node
#from rclpy.timer import Timer

# from rclpy.qos import qos_profile_sensor_data   # quality of service
from sensor_msgs.msg import Image as Ros2MsgImage
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped  
from geometry_msgs.msg import PoseWithCovarianceStamped
#from geometry_msgs.msg import Pose, Twist         # Velocity command

from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener

from gazebo_msgs.msg import ModelStates

from tf2_geometry_msgs import do_transform_vector3
from geometry_msgs.msg import Vector3Stamped

from rosgraph_msgs.msg import Clock

from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Twist
import math
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import NavSatFix

from sensor_msgs.msg import Imu

# Path visualization
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import DeleteModel
from rclpy.duration import Duration

class Navigation(Node):

    def __init__(self, robot_name, x, y, z, grid_size, timer_period, imu_ema_period, cmds):

        self.nCycleCount = 0
        #self.theta = 0

        # self.arrAccelX = []
        # self.arrAccelY = []

        self.arrX = []
        self.arrY = []
        
        print("*** Navigation.init()... ", flush=True)

        # self.declare_parameter('use_sim_time', True)
        # self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        if(robot_name != ""):
            super().__init__(robot_name + '_Navigation')
            self.strSlashRobotName = "/" + robot_name
            self.strRobotNameSlash = robot_name + "/"
        else:
            super().__init__('Navigation')
            self.strSlashRobotName = ""
            self.strRobotNameSlash = ""

        self.strRobotName = robot_name

        print("*** robot_name: ", robot_name, flush=True)

        # --- 

        # --- Used to present data as charts

        self.track_gt = []          # "real" robot's position
        self.track_kalman = []      # predicted position

        self.arrChartTime         = []   # Integer numbers in seconds
        #self.arrChartOdom         = []
        self.arrChartImu         = []
        # self.arrChartFilteredOdom = []
        self.arrChartGPS          = []
        # self.arrChartFilteredGPS  = []
        
        self.arrChartKalman         = []
        self.arrChartGroundTruth  = []

        self.arrChartHeadingError = []

        self.nPointIdx = 0        

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

        self.cmds = cmds
        self.nCurrentCommand = 0

        # ---
        initYaw = np.radians(0)
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, initYaw)

        self.current_pose = Pose()
        self.current_pose.position.x = x
        self.current_pose.position.y = y
        self.current_pose.position.z = z
        self.current_pose.orientation.x = qx
        self.current_pose.orientation.y = qy
        self.current_pose.orientation.z = qz
        self.current_pose.orientation.w = qw

        self.bNodesActive = False
        self.initial_pose_received = False

        # ---

        self.ground_truth_pose = copy.deepcopy(self.current_pose)
        
        # --- I do not see any reasons to have it as Pose(), so I don't

        # _, _, kalman_yaw = self.euler_from_quaternion(
        #     self.current_pose.orientation.x,
        #     self.current_pose.orientation.y,
        #     self.current_pose.orientation.z,
        #     self.current_pose.orientation.w)        

        # self.odom_pose = copy.deepcopy(self.current_pose)
        # self.odom_twist = np.array([
        #     0,                              # initial linear speed x
        #     0                               # initial angular speed z
        # ])
        

        # --- GPS

        self.gps_pose = copy.deepcopy(self.current_pose)

        # As this is a Gazebo simulation, I will not bother with GPS subscription,
        # signal will be simulated instead.
        self.bGpsAvailable = True
        # Set GPS initial pos
        # self.origin_latitude = 44.98
        # self.origin_longitude = -93.27
        # self.origin_altitude = 254.99

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Adjust the depth according to your needs
            durability=QoSDurabilityPolicy.VOLATILE  # You can use TRANSIENT_LOCAL for durability
        )

        # self.gps_subscriber = self.create_subscription(
        #     NavSatFix,
        #     self.strSlashRobotName + '/gps/fix',
        #     self.gps_callback,
        #     qos_profile = qos_profile  # Set the queue size
        # )

        # # nZone = int((self.origin_longitude + 180) / 6) + 1
        # # self.wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
        # # self.utm = pyproj.Proj(proj="utm", zone=nZone, datum='WGS84')  # Change the zone according to your location
        # # self.origin_x, self.origin_y = pyproj.transform(self.wgs84, self.utm, self.origin_longitude, self.origin_latitude)

        # --- IMU

        # Note: IMU <update_rate>100</update_rate>
        self.imu_ema_period = imu_ema_period

        self.imu_ema_ax = 0.0
        self.imu_ema_ay = 0.0
        self.imu_ema_az = 0.0        

        self.timeImuLast = self.get_clock().now()

        self.imu_subscriber = self.create_subscription(
            Imu,
            self.strSlashRobotName + '/imu',
            self.imu_callback,
            qos_profile = qos_profile
        )

        self.imu_data = np.array([
            0.,                              # ax
            0.,                              # ay
            0.                               # angular vel
        ])

        self.mag_yaw = 0.

        # For charting only
        # self.imu_vel = 0
        #self.imu_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])

        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0]) 

        # --- Odometry

        # # Subscribe to messages of type nav_msgs/Odometry that provides position and orientation of the robot

        # self.odom_subscriber = self.create_subscription(
        #     Odometry,
        #     self.strSlashRobotName + '/odom',
        #     self.odom_callback,
        #     10)          

        # --- Ground Truth

        ground_truth_pos_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.ground_truth_pose_callback,
            10  # QoS profile depth
        )

        # --- Commands publishing

        self.publisherCmdVel = self.create_publisher(
            Twist,
            self.strSlashRobotName + "/cmd_vel",           # For Gazebo diff. drive controller
            10) 

        # ---

        # TBD: reinit. filter if crashes due to negative leading minor
        print("*** KalmanNav.init()... ", flush=True)

        #landmarks = np.array([])
        landmarks = np.array([[5, 10], [10, 5], [15, 15], [20, 5], [0, 30], [50, 30], [40, 10]])

        initPos = [
            self.current_pose.position.x, self.current_pose.position.y, initYaw, 
            0., 0., np.radians(0), 
            0., 0.,
            0.,
            0., 0.
        ]

        self.posX = initPos[0]
        self.posY = initPos[1]
        self.theta = initPos[2]
        # self.vX = initPos[3]
        # self.vY = initPos[4]
        # self.vTheta = initPos[5]

        self.arrX.append(initPos[0])
        self.arrY.append(initPos[1])

        # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>", self.current_pose.position.x, 
        #     self.current_pose.position.y, 
        #     initYaw)

        # ---

        ##self.tf_broadcaster = TransformBroadcaster(self)

        ##self.tf_buffer = Buffer()
        ##self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robotToGlobalTransform = TransformStamped()    

        # ---

        # Comment in/out any combination
        dictSensors = { 
            #"gps": 2, 
            "imu_accel": 4, 
            #"imu_gyro": 1,
            #"imu_magnet": 1,
            #"landmarks": 2 * len(landmarks) 
        }

        nGpsStep = 1
        nImuStep = 1 
        dt = timer_period

        self.nav = KalmanNav(landmarks, dictSensors, initPos,
            sigma_gps = 10,
            sigma_imu_accel = 0.01,
            sigma_imu_accel_bias = 0.001,

            sigma_imu_gyro = 0.01,
            sigma_imu_gyro_bias = 0.001,

            sigma_imu_magnet = 0.1,
            sigma_range=10, 
            sigma_bearing=1,
            dt=dt,
            gps_step=nGpsStep,
            imu_step=nImuStep)      

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.timer_subscriber = self.create_subscription(
        #     Clock,
        #     '/clock',
        #     self.timer_callback,
        #     1000  # QoS profile depth
        # )
        # self.timer_subscriber  # prevent unused variable warning

        self.prev_time = self.get_clock().now()
        
        # self.camera_subscription = self.create_subscription(
        #     Ros2MsgImage,
        #     '/camera/image_raw',
        #     self.camera_callback,
        #     qos_profile_sensor_data)
        # self.camera_subscription


        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
            self.strSlashRobotName + '/initialpose', 10) 
            
        self.waitForNodesToStart()

        print("*** Navigation.init() done", flush=True)

    # ---

    # def camera_callback(self, msg):
    #     print("*** Camera callback triggered! Performing task...", flush=True)

    # ---

    # def broadcast_transform(self):

    #     # Set the timestamp
    #     self.robotToGlobalTransform.header.stamp = self.get_clock().now().to_msg()
    #     self.robotToGlobalTransform.header.frame_id = 'world'  # Global frame
    #     self.robotToGlobalTransform.child_frame_id = self.strRobotNameSlash + 'base_link'  # Robot's frame

    #     # Dummy values for the robot's pose
    #     self.robotToGlobalTransform.transform.translation.x = self.nav.ukf.x[0]
    #     self.robotToGlobalTransform.transform.translation.y = self.nav.ukf.x[1]
    #     self.robotToGlobalTransform.transform.translation.z = 0.0

    #     # Dummy orientation (Quaternion)
    #     q = self.euler_to_quaternion(0, 0, self.nav.ukf.x[2])  # Roll, Pitch, Yaw
    #     self.robotToGlobalTransform.transform.rotation.x = q[0]
    #     self.robotToGlobalTransform.transform.rotation.y = q[1]
    #     self.robotToGlobalTransform.transform.rotation.z = q[2]
    #     self.robotToGlobalTransform.transform.rotation.w = q[3]

    #     # Note that we use this transform locally, without broadcasting it
    #     # otherwise it will interfere with same transforms Gazebo sends to RViz
    #     #self.tf_broadcaster.sendTransform(self.robotToGlobalTransform)

    # ---

    def printPoseInfo(self):
        #print("Pose Info", ">>>", self.strRobotName, "<<<", self.initial_pose_received, flush=True)
        if(self.strRobotName != "" or self.initial_pose_received == False):
            return

        now = self.get_clock().now()
        #delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.
        duration = now - self.prev_time
        delta_ms = duration.to_msg().sec * 1000 + duration.to_msg().nanosec / 1e6

        if delta_ms >= 1000:
            # ---            

            ground_truth_pose = self.ground_truth_pose
            # ground_truth_pose = self.getGroundTruthCoordinates()

            _, _, ground_truth_yaw = self.euler_from_quaternion(
                ground_truth_pose.orientation.x,
                ground_truth_pose.orientation.y,
                ground_truth_pose.orientation.z,
                ground_truth_pose.orientation.w)

            # ---

            # odom_pose = self.odom_pose

            # _, _, odom_yaw = self.euler_from_quaternion(
            #     odom_pose.orientation.x,
            #     odom_pose.orientation.y,
            #     odom_pose.orientation.z,
            #     odom_pose.orientation.w)

            # ---

            # amcl_pose = self.amcl_pose

            # _, _, amcl_yaw = self.euler_from_quaternion(
            #     amcl_pose.orientation.x,
            #     amcl_pose.orientation.y,
            #     amcl_pose.orientation.z,
            #     amcl_pose.orientation.w)

            # # ---

            # filtered_odom_pose = self.filtered_odom_pose

            # _, _, filtered_odom_yaw = self.euler_from_quaternion(
            #     filtered_odom_pose.orientation.x,
            #     filtered_odom_pose.orientation.y,
            #     filtered_odom_pose.orientation.z,
            #     filtered_odom_pose.orientation.w)

            # ---
            gps_pose = self.gps_pose  
            #gps_filtered_pose = self.gps_filtered_pose

            # ---
            
            #print("*** Status: %s; " % "working...") #self.navStatus)
            print("{:<16} {:<7} {:<7} {:<7} {:<7}".format("", "X", "Y", "Z", "Yaw"))
            #print("AMCL:           %7.2f  %7.2f  %7.2f  %7.0f", amcl_pose.position.x, 
            #    amcl_pose.position.y, amcl_pose.position.z, math.degrees(amcl_yaw)) 
            #print("Odom:           %7.2f  %7.2f  %7.2f  %7.0f", odom_pose.position.x, odom_pose.position.y, 
            #    odom_pose.position.z, math.degrees(odom_yaw)) 
            # print("Filtered Odom:  %7.2f  %7.2f  %7.2f  %7.0f", filtered_odom_pose.position.x, filtered_odom_pose.position.y, 
            #    filtered_odom_pose.position.z, math.degrees(filtered_odom_yaw))

            # print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7.2f}".format("Odom:", 
            #     odom_pose.position.x, odom_pose.position.y, 0.0, math.degrees(odom_yaw)), flush=True)

            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7}".format("GPS:", gps_pose.position.x, gps_pose.position.y, gps_pose.position.z, "---"))
            
            #print("{:<16} {:<7.2f} {:<7.2f} {:<7} {:<7}".format("IMU:", self.imu_pos[0], self.imu_pos[1], "---", "---"))
            
            # print("Filtered GPS:   %7.2f  %7.2f  %7.2f     ---", gps_filtered_pose.position.x, gps_filtered_pose.position.y, 
            #     gps_filtered_pose.position.z)
            #print("Ground Truth:   %7.2f  %7.2f  %7.2f  %7.0f\n===\n\n" % 
            #(ground_truth_pose.position.x, ground_truth_pose.position.y,
            #    ground_truth_pose.position.z, math.degrees(ground_truth_yaw)), flush=True)

            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7.2f}".format("Kalman:", 
                self.nav.ukf.x[0], self.nav.ukf.x[1], 0.0, math.degrees(self.nav.normalize_angle(self.nav.ukf.x[2]))), flush=True)

            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7.2f}\n===\n".format("Ground Truth:", 
                ground_truth_pose.position.x, ground_truth_pose.position.y,
                ground_truth_pose.position.z, math.degrees(ground_truth_yaw)), flush=True)

            # ---

            if(self.strRobotName == "" and self.nPointIdx > 0):
                self.arrChartTime.append(len(self.arrChartTime))
                self.arrChartKalman.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - self.nav.ukf.x[0])**2 + 
                        (ground_truth_pose.position.y - self.nav.ukf.x[1])**2)) 
                # self.arrChartOdom.append(
                #     math.sqrt(
                #         (ground_truth_pose.position.x - odom_pose.position.x)**2 + 
                #         (ground_truth_pose.position.y - odom_pose.position.y)**2))
                # self.arrChartFilteredOdom.append(
                #     math.sqrt(
                #         (ground_truth_pose.position.x - filtered_odom_pose.position.x)**2 + 
                #         (ground_truth_pose.position.y - filtered_odom_pose.position.y)**2))
                self.arrChartGPS.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - gps_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - gps_pose.position.y)**2))
                # self.arrChartFilteredGPS.append(
                #     math.sqrt(
                #         (ground_truth_pose.position.x - gps_filtered_pose.position.x)**2 + 
                #         (ground_truth_pose.position.y - gps_filtered_pose.position.y)**2))

                self.arrChartHeadingError.append(np.degrees(abs(self.nav.normalize_angle(ground_truth_yaw - self.nav.ukf.x[2]))))

            self.nPointIdx += 1

    # ---

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    # ---

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

    # def lat_lon_to_xy(self, latitude, longitude):
    #     # Create Point objects for the origin and the target location
    #     origin = Point(self.origin_latitude, self.origin_longitude)
    #     target = Point(latitude, longitude)

    #     # Calculate the distance and bearing from origin to target
    #     d = distance.distance(origin, target)
    #     bearing = d.bearing

    #     # Convert bearing to radians
    #     bearing_rad = math.radians(bearing)

    #     # Calculate x and y
    #     y = d.meters * math.cos(bearing_rad)
    #     x = d.meters * math.sin(bearing_rad)

    #     return -x, -y

    # ------

    def _waitForInitialPose(self):
        print("\t***_waitForInitialPose...")
        while not self.initial_pose_received:
            print("\t\t***Waiting for initial pose...")
            rclpy.spin_once(self, timeout_sec=1.0)
            time.sleep(1)
        print("\tInitial pose received")
        return

    # ---

    def waitForNodesToStart(self):
        print("*** Navigation.waitForNodesToStart()...")
        # print("\t*** amcl starting...")
        # self._waitForNodeToActivate(self.strRobotNameSlash + 'amcl')
        # print("\t*** amcl started successfully")

        # print("\t*** Datum starting...")
        # self._waitForNodeToActivate(self.strRobotNameSlash + 'datum')
        # print("\t*** datum started successfully")        

        self._waitForInitialPose()
        
        print("*** Navigation.waitForNodesToStart() done")

        self.bNodesActive = True

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
        #print("\t\t\t*** Publishing Initial Pose")
        self.initial_pose_pub.publish(msg)

        self.initial_pose_received = True
        
    # ---

    # def update_quaternion(self, wx, wy, wz, dt):
    #     q0, q1, q2, q3 = self.quaternion
    #     q0_dot = -0.5 * (q1 * wx + q2 * wy + q3 * wz)
    #     q1_dot = 0.5 * (q0 * wx - q3 * wy + q2 * wz)
    #     q2_dot = 0.5 * (q3 * wx + q0 * wy - q1 * wz)
    #     q3_dot = 0.5 * (-q2 * wx + q1 * wy + q0 * wz)

    #     q0 += q0_dot * dt
    #     q1 += q1_dot * dt
    #     q2 += q2_dot * dt
    #     q3 += q3_dot * dt

    #     norm = np.sqrt(q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2)
    #     self.quaternion = np.array([q0, q1, q2, q3]) / norm

    # def compute_rotation_matrix(self):
    #     q0, q1, q2, q3 = self.quaternion
    #     r11 = 2 * (q0 ** 2 + q1 ** 2) - 1
    #     r12 = 2 * (q1 * q2 - q0 * q3)
    #     r13 = 2 * (q1 * q3 + q0 * q2)
    #     r21 = 2 * (q1 * q2 + q0 * q3)
    #     r22 = 2 * (q0 ** 2 + q2 ** 2) - 1
    #     r23 = 2 * (q2 * q3 - q0 * q1)
    #     r31 = 2 * (q1 * q3 - q0 * q2)
    #     r32 = 2 * (q2 * q3 + q0 * q1)
    #     r33 = 2 * (q0 ** 2 + q3 ** 2) - 1

    #     rotation_matrix = np.array([[r11, r12, r13],
    #                                 [r21, r22, r23],
    #                                 [r31, r32, r33]])
    #     return rotation_matrix

    # ---

    def transform_vector_from_robot_to_global(self, vector, translation, quaternion):
        # Convert the quaternion to a rotation matrix
        rotation = R.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()
        
        # Apply rotation
        rotated_vector = rotation_matrix.dot(vector)
        
        # Apply translation
        global_vector = rotated_vector #+ translation
        
        return global_vector

    # ---    

    def imu_callback(self, msg):
        timeCurrent = self.get_clock().now()
        dt = (timeCurrent - self.timeImuLast).nanoseconds * 1e-9  # Convert nanoseconds to seconds

        self.timeImuLast = timeCurrent

        # Get linear acceleration in robot frame
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        alpha = 2 / (self.imu_ema_period + 1)
        self.imu_ema_ax = alpha * ax + (1 - alpha) * self.imu_ema_ax
        self.imu_ema_ay = alpha * ay + (1 - alpha) * self.imu_ema_ay
        self.imu_ema_az = alpha * az + (1 - alpha) * self.imu_ema_az

        ax = self.imu_ema_ax
        ay = self.imu_ema_ay
        az = self.imu_ema_az

        #print(f"{ax:.2f}, {ay:.2f}, {az:.2f}", flush=True)
        
        # Create a Vector3Stamped message to hold the robot frame accelerations
        # imu_data_robot = Vector3Stamped()
        # imu_data_robot.header.stamp = msg.header.stamp  # Use the same timestamp as IMU message
        # imu_data_robot.header.frame_id = self.strRobotNameSlash + 'base_link'  # This is the robot's frame
        # imu_data_robot.vector.x = ax
        # imu_data_robot.vector.y = ay
        # imu_data_robot.vector.z = az

        try:
            # Get the transform from the robot frame to the global frame
            #transform = self.tf_buffer.lookup_transform('map', self.strRobotNameSlash + 'base_link', msg.header.stamp, timeout=Duration(seconds=1.0))

            # Transform the IMU data to the global frame using the do_transform_vector3 method
            #imu_data_global = do_transform_vector3(imu_data_robot, self.robotToGlobalTransform)

            vector = [ax, ay, az]
            translation = np.array([0.0, 0.0, 0.0])#np.array([self.nav.ukf.x[0], self.nav.ukf.x[1], 0.0])
            
            q = self.euler_to_quaternion(0, 0, self.nav.ukf.x[2])  # Roll, Pitch, Yaw
            self.robotToGlobalTransform.transform.rotation.x = q[0]
            self.robotToGlobalTransform.transform.rotation.y = q[1]
            self.robotToGlobalTransform.transform.rotation.z = q[2]
            self.robotToGlobalTransform.transform.rotation.w = q[3]
            quaternion = np.array([q[0], q[1], q[2], q[3]])

            imu_data_global = self.transform_vector_from_robot_to_global(vector, translation, quaternion)

            # ax = imu_data_global.vector.x
            # ay = imu_data_global.vector.y
            # az = imu_data_global.vector.z - 9.81

            ax = imu_data_global[0]
            ay = imu_data_global[1]
            az = imu_data_global[2] - 9.81

            # self.arrAccelX.append(ax)
            # self.arrAccelY.append(ay)

            # self.vX += self.imu_data[0] * dt
            # self.vY += self.imu_data[1] * dt
            # self.posX += self.vX * dt
            # self.posY += self.vY * dt

            # self.arrX.append(self.posX)
            # self.arrY.append(self.posY)

            #print(f">>> {ax:.2f}, {ay:.2f}, {az:.2f}", flush=True)

            # Global frame
            self.imu_data[0] = ax
            self.imu_data[1] = ay

            angular_velocities = msg.angular_velocity
            self.imu_data[2] = angular_velocities.z
            #self.theta = self.theta + angular_velocities.z * dt

            #print(f"### {self.imu_data[0]:.2f}, {self.imu_data[1]:.2f}, {az:.2f}", flush=True)
            #print(f"### {self.imu_data[2]:.6f}, {math.degrees(self.theta):.2f}", flush=True)

            
            # # Access orientation (quaternion)
            # orientation = msg.orientation
            # self.get_logger().info('Received orientation: x={}, y={}, z={}, w={}'.format(
            #     orientation.x, orientation.y, orientation.z, orientation.w))

            # # Access magnetic field strength
            # magnetic_field = msg.magnetic_field
            # self.get_logger().info('Received magnetic field: x={}, y={}, z={}'.format(
            #     magnetic_field.x, magnetic_field.y, magnetic_field.z))

            # # Access temperature
            # temperature = msg.temperature
            # self.get_logger().info('Received temperature: {}'.format(temperature))

            # # Access timestamp
            # timestamp = msg.header.stamp
            # self.get_logger().info('Received timestamp: {}'.format(timestamp))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(f">>> Error transforming IMU data: {e}")  




    # ---

    # def odom_callback(self, msg):
    #     self.odom_pose = msg.pose.pose

    #     # if(self.initial_pose_received == False):
    #     #     self.current_pose = msg.pose.pose
    #     #     self._setInitialPose(msg.pose.pose)

    #     self.odom_twist = np.array([
    #         msg.twist.twist.linear.x,
    #         msg.twist.twist.angular.z
    #     ])

    #     # print("*****", msg.twist.twist.linear.x, 
    #     #     math.degrees(msg.twist.twist.angular.z), flush=True)



    # # ---

    def gps_callback(self, msg):

        x, y = self.lat_lon_to_xy(msg.latitude, msg.longitude)

        self.gps_pose.position.x = x
        self.gps_pose.position.y = y
        
        # For convenience, Gazebo model is at 0 meters altitude
        self.gps_pose.position.z = msg.altitude - self.origin_altitude
        self.gps_pose.orientation.x = 0.0
        self.gps_pose.orientation.y = 0.0
        self.gps_pose.orientation.z = 0.0
        self.gps_pose.orientation.w = 1.0

        if(self.initial_pose_received == False):
            self.current_pose = self.gps_pose
            self._setInitialPose(self.gps_pose)
            
        #print("Latitude: ", msg.latitude, " Longitude: ", msg.longitude, " Altitude: ", msg.altitude, flush=True)
        #print(x, y, flush=True)

    # --- Simulate GPS, Gazebo only

    def on_gps_callback(self):

        if(self.bGpsAvailable):
            self.gps_pose.position.x = self.ground_truth_pose.position.x + np.random.normal(0, self.nav.sigma_gps, size=(1))[0]
            self.gps_pose.position.y = self.ground_truth_pose.position.y + np.random.normal(0, self.nav.sigma_gps, size=(1))[0]
        else:
            self.gps_pose.position.x = self.nav.ukf.x[0]
            self.gps_pose.position.y = self.nav.ukf.x[1]

        if(self.initial_pose_received == False):
            self.current_pose = self.gps_pose
            self._setInitialPose(self.gps_pose)            

    # ---

    def ground_truth_pose_callback(self, msg):
        strName = self.strRobotName
        if(self.strRobotName == ""):
            strName = "nav25d_02"
        robot_index = msg.name.index(strName)
        self.ground_truth_pose = msg.pose[robot_index]

        # ---

        # # Position
        # x = self.ground_truth_pose.position.x
        # y = self.ground_truth_pose.position.y
        # z = self.ground_truth_pose.position.z

        # # Orientation (convert quaternion to Euler angles)
        # q = self.ground_truth_pose.orientation
        # roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        # # Angular velocity
        # angular_speed_x = msg.twist[robot_index].angular.x
        # angular_speed_y = msg.twist[robot_index].angular.y
        # angular_speed_z = msg.twist[robot_index].angular.z      

        # print(f"x={x:.2f}, y={y:.2f}, z={z:.2f}, \
        #     roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}, \
        #     angular_speed_x={angular_speed_x:.2f}, angular_speed_y={angular_speed_y:.2f}, angular_speed_z={angular_speed_z:.2f}",
        #     flush=True)    

    # ---

    def showCharts(self):
        if(self.strRobotName == ""):
            # Create a figure and axis
            fig, ax = plt.subplots()

            # Plot the data with different colors and labels
            
            # Not shown as error is accumulating to large values
            #ax.plot(self.arrChartTime, self.arrChartOdom, label='self.arrChartOdom', color='blue', linestyle='-')
            
            #ax.plot(self.arrChartTime, self.arrChartFilteredOdom, label='self.arrChartFilteredOdom', color='green', linestyle='-')
            #ax.plot(self.arrChartTime, self.arrChartGPS, label='GPS', color='purple', linestyle='-')
            #ax.plot(self.arrChartTime, self.arrChartFilteredGPS, label='self.arrChartFilteredGPS', color='orange', linestyle='-')
            #ax.plot(self.arrChartTime, self.arrChartGT, label='self.arrChartGT', color='brown', linestyle='-')

            ax.plot(self.arrChartTime, self.arrChartKalman, label='Kalman Error', color='red', linestyle='-')

            # Add legend
            ax.legend(loc='upper left')

            # Set labels for X and Y axes
            ax.set_xlabel('Time')
            ax.set_ylabel('Values')

            # Show the plot
            plt.show()

            # ---

            # fig, ax = plt.subplots()

            # ax.plot(self.arrChartTime, self.arrChartGPS, label='GPS Error', color='red', linestyle='-')

            # # Add legend
            # ax.legend(loc='upper left')

            # # Set labels for X and Y axes
            # ax.set_xlabel('Time')
            # ax.set_ylabel('Values')

            # # Show the plot
            # plt.show()

            # ---

            fig, ax = plt.subplots()

            ax.plot(self.arrChartTime, self.arrChartHeadingError, label='Heading Error', color='red', linestyle='-')

            # Add legend
            ax.legend(loc='upper left')

            # Set labels for X and Y axes
            ax.set_xlabel('Time')
            ax.set_ylabel('Error')

            # Show the plot
            plt.show()

            # ---

            self.arrChartTime         = []
            self.arrChartKalman         = []
            #self.arrChartOdom         = []
            #self.arrChartFilteredOdom = []
            self.arrChartGPS          = []
            #self.arrChartFilteredGPS  = []
            #self.arrChartGT           = [] 
            self.arrChartHeadingError = []


    # ---    

    def drive(self, linear_speed, angular_speed):
        msg = Twist()
        msg.linear.x = float(linear_speed)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_speed)
         
        self.publisherCmdVel.publish(msg) 

    # ---

    def on_magnetometer_callback(self):
        _, _, ground_truth_yaw = self.euler_from_quaternion(
            self.ground_truth_pose.orientation.x,
            self.ground_truth_pose.orientation.y,
            self.ground_truth_pose.orientation.z,
            self.ground_truth_pose.orientation.w)

        imuMag = self.nav.transform_magnetic_field(
            self.nav.world_magnet, (ground_truth_yaw + np.random.normal(0, self.nav.sigma_imu_magnet, size=(1)))[0])

        self.mag_yaw = -np.arctan2(imuMag[0], imuMag[1])

        #print(np.degrees(self.mag_yaw), flush=True)


    #def timer_callback(self, msg):
    def timer_callback(self):
        
        # sim_time_sec = msg.clock.sec + msg.clock.nanosec / 1e9
        # print(f'Simulation time: {sim_time_sec:.9f} seconds', flush=True)

        #print(time.time(), flush=True)

        #print("*** Navigation - timer callback: ", flush=True, end="")

        ellipse_step = 50
        nMaxCycles = 1

        # Gazebo is odd when it comes to GPS to xyz, so simulate one
        self.on_gps_callback()

        if(not self.initial_pose_received):
            return

        if(self.nCurrentCommand < len(self.cmds) and self.nCycleCount < nMaxCycles):
            
            # Gazebo has no magnetometer worth mentioning, so simulate one
            #self.on_magnetometer_callback()

            # Gazebo is odd when it comes to GPS to xyz, so simulate one
            self.on_gps_callback()

            cmd = self.cmds[self.nCurrentCommand]
            linear_speed = cmd[0]
            angular_speed = cmd[1]

            #print(self.nCurrentCommand, ": ", linear_speed, angular_speed, flush=True)

            # if(self.nCurrentCommand > 100 and self.nCurrentCommand < 200):
            #     self.drive(linear_speed / 2, angular_speed)
            # else:
            #     self.drive(linear_speed, angular_speed)
            self.drive(linear_speed, angular_speed)

            self.nCurrentCommand += 1

            # for i, u in enumerate(cmds):
            #     # Set random elements in self.arrShowLandmarks to False (hide landmarks)
            #     nHideLandmarks = np.random.randint(0, 4)
            #     self.arrShowLandmarks = np.ones(len(self.landmarks), dtype=bool)
            #     arrFalse = np.random.choice(np.arange(len(self.landmarks)), size=nHideLandmarks, replace=False)
            #     self.arrShowLandmarks[arrFalse] = False

            self.nav.nCurrentKalmanStep += 1
            #self.nav.sim_pos = self.nav.move(self.nav.sim_pos, self.nav.dt / self.nav.step, cmd)

            # _, _, ground_truth_yaw = self.euler_from_quaternion(
            #     self.ground_truth_pose.orientation.x,
            #     self.ground_truth_pose.orientation.y,
            #     self.ground_truth_pose.orientation.z,
            #     self.ground_truth_pose.orientation.w)

            self.track_gt.append(np.array([ self.ground_truth_pose.position.x, self.ground_truth_pose.position.y])) #, ground_truth_yaw ]))
            
            # ---

            # odom_linear_x = self.odom_twist[0]
            # odom_angular_z = self.odom_twist[1]

            # _, _, odom_yaw = self.euler_from_quaternion(
            #     self.odom_pose.orientation.x,
            #     self.odom_pose.orientation.y,
            #     self.odom_pose.orientation.z,
            #     self.odom_pose.orientation.w)

            # self.track_odom.append(np.array([ self.odom_pose.position.x, self.odom_pose.position.y, odom_yaw ]))
            
            # ---
            
            #self.track_gt.append([ self.ground_truth_pose.position.x, self.ground_truth_pose.position.y, ground_truth_yaw ])

            # ---

            kalman_prev = copy.deepcopy(self.nav.ukf.x)
            self.nav.ukf.predict(u=cmd)

            # if self.nav.nCurrentKalmanStep % self.nav.step == 0:
            #         if("landmarks" in self.dictSensors):
            #             j = 0
            #             if("gps" in self.dictSensors):
            #                 j += self.dictSensors["gps"] 
            #             if("imu_orientation" in self.dictSensors):
            #                 j += self.dictSensors["imu_orientation"]
            #             if("imu_angular_vel" in self.dictSensors):
            #                 j += self.dictSensors["imu_angular_vel"]  
            #             if("imu_accel" in self.dictSensors):
            #                 j += self.dictSensors["imu_accel"]                                        
            #             for lmark in self.landmarks:
            #                 distance_adjusted_sigma_range = self.sigma_range * sqrt((self.sim_pos[0] - lmark[0])**2 
            #                     + (self.sim_pos[1] - lmark[1])**2)
            #                 self.arrSigmas[j] = distance_adjusted_sigma_range**2
            #                 j += 2
            #             self.ukf.R = np.diag(self.arrSigmas)

            #     x, y = self.sim_pos[0], self.sim_pos[1]
            z = []

            # if("odom" in self.nav.dictSensors):
            #     z.extend([odom_linear_x, odom_angular_z])
            #     # z.extend([self.nav.dictState["prev_x"] + odom_linear_x * cos(self.nav.dictState["prev_heading"] * self.nav.dt), 
            #     #     self.nav.dictState["prev_y"] + odom_linear_x * sin(self.nav.dictState["prev_heading"] * self.nav.dt),
            #     #     self.nav.normalize_angle(self.nav.dictState["prev_heading"] + odom_angular_z * self.nav.dt)])

            bNeedUpdate = False

            if("gps" in self.nav.dictSensors):
                if(self.nav.nCurrentKalmanStep % self.nav.gps_step == 0):
                    # 0 for yaw, as GPS does not provide orientation
                    gpsPos = np.array([ self.gps_pose.position.x, self.gps_pose.position.y, 0 ])
                    bNeedUpdate = True
                else:
                    gpsPos = self.nav.ukf.x[:2]
                z.extend(gpsPos[:2])

            if("imu_accel" in self.nav.dictSensors):

                if(self.nav.nCurrentKalmanStep % self.nav.imu_step == 0):
                    imuAccel = [self.imu_data[0], self.imu_data[1]]
                    self.nav.dictState["accel_drift_vx"] += imuAccel[0] * self.nav.dt
                    self.nav.dictState["accel_drift_vy"] += imuAccel[1] * self.nav.dt
                    bNeedUpdate = True
                else:
                    imuAccel = self.nav.ukf.x[6:]
                    self.nav.dictState["accel_drift_vx"] = self.nav.ukf.x[9]
                    self.nav.dictState["accel_drift_vy"] = self.nav.ukf.x[10]

                z.extend([imuAccel[0], imuAccel[1], self.nav.dictState["accel_drift_vx"], self.nav.dictState["accel_drift_vy"]])

            if("imu_gyro" in self.nav.dictSensors):
                if(self.nav.nCurrentKalmanStep % self.nav.imu_step == 0):
                    imuGyroAngularVel = self.imu_data[2]
                    self.nav.dictState["gyro_drift"] += self.nav.normalize_angle(imuGyroAngularVel * self.nav.dt)
                    yaw = self.nav.normalize_angle(self.nav.dictState["gyro_drift"])

                    bNeedUpdate = True
                else:
                    self.nav.dictState["drift"] = self.ukf.x[8]
                    yaw = self.nav.ukf.x[2]

                #if("imu_magnet" not in self.nav.dictSensors):
                z.extend([yaw])

            if("imu_magnet" in self.nav.dictSensors):

                if(self.nav.nCurrentKalmanStep % self.nav.imu_step == 0):

                    mag_yaw = self.mag_yaw
                    
                    # # Here we run Complimentary filter on accel and magnetometer
                    # if("imu_gyro" in self.nav.dictSensors):
                    #     # theta[n] = alpha*(theta[n-1] + g*dt) + (1-alpha)*a
                    #     # alphaYaw = atan(accelZ/sqrt(accelX**2 + accelY**2))
                    #     # alphaYaw = atan(-ay, ax)                        

                    #     # Integrate gyroscope data
                    #     gyro_yaw = np.array(self.nav.ukf.x[2] + imuGyroAngularVel * self.nav.dt)

                    #     # Calculate yaw from magnetometer data
                    #     yaw = 0.98 * gyro_yaw + 0.02 * mag_yaw
                    # else:
                    #     yaw = mag_yaw
                    yaw = mag_yaw

                    bNeedUpdate = True

                else:
                    yaw = self.ukf[2]                            

                z.extend([yaw])

            # if("landmarks" in self.nav.dictSensors):
            #     for lmark in self.nav.landmarks:
            #         dx, dy = lmark[0] - x, lmark[1] - y
            #         d = sqrt(dx**2 + dy**2) + randn()*self.nav.sigma_range
            #         bearing = atan2(lmark[1] - y, lmark[0] - x)
            #         a = (self.nav.normalize_angle(bearing - self.sim_pos[2] + randn()*self.sigma_bearing))
            #         z.extend([d, a])
                
            if(bNeedUpdate == True):
                self.nav.ukf.update(z, u = cmd, dt=self.nav.dt)
                self.track_kalman.append(np.array([self.nav.ukf.x[0], self.nav.ukf.x[1]]))

            # self.broadcast_transform()

            #if self.nav.nCurrentKalmanStep % ellipse_step == 0:
            #    plot_covariance_ellipse((self.nav.ukf.x[0], self.nav.ukf.x[1]), self.nav.ukf.P[0:2, 0:2], std=6, facecolor='g', alpha=0.8)

            self.printPoseInfo()
        
        elif(self.nCycleCount < nMaxCycles):
            self.nCurrentCommand = 0
            self.nCycleCount += 1

        else:
            # Due to the way FilterPy implements plotting of covariance ellises, it is 
            # either one chart or another. I don't have time to fix other people's 
            # bugs

            # This code draws track (ground truth trajectory)
            track_gt = np.array(self.track_gt)
            plt.plot(track_gt[:, 0], track_gt[:,1], color='k', lw=2)

            track_kalman = np.array(self.track_kalman)
            plt.plot(track_kalman[:, 0], track_kalman[:,1], color='m', lw=2)

            plt.axis('equal')
            plt.title("Trajectory")
            # while this code shows both track and ellipses on the same chart,
            # because of the way FilterPy draws ellipses on whatever plot
            # is awailable.
            plt.show()

            self.showCharts()            

            #print('final covariance', self.nav.ukf.P.diagonal())

            # arrVelX = [0]
            # arrVelY = [0]
            # for i in range(1, len(self.arrAccelX)):
            #     arrVelX.append(arrVelX[i-1] + self.arrAccelX[i] * self.nav.dt)
            #     arrVelY.append(arrVelY[i-1] + self.arrAccelY[i] * self.nav.dt)

            # arrX = [0]
            # arrY = [0]
            # for i in range(1, len(arrVelX)):
            #     arrX.append(arrX[i-1] + arrVelX[i] * self.nav.dt)
            #     arrY.append(arrY[i-1] + arrVelY[i] * self.nav.dt)

            # fig, ax = plt.subplots()
            # ax.plot(arrX, arrY, label='Accel based path', color='red', linestyle='-')

            # # Add legend
            # ax.legend(loc='upper left')

            # # Set labels for X and Y axes
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')

            # # Show the plot
            # plt.show()

            # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", len(self.arrAccelX))

            # fig, ax = plt.subplots()
            # ax.plot(self.arrX, self.arrY, label='Accel based path-1', color='red', linestyle='-')

            # # Add legend
            # ax.legend(loc='upper left')

            # # Set labels for X and Y axes
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')

            # # Show the plot
            # plt.show()            

            return            

        #print("done()", flush=True)

    # ---

    def destroy(self):
        # Cancel the timer and perform any other cleanup
        if self.timer:
            self.timer.cancel()
        super().destroy()

# ---

def turn(v, t0, t1, steps):
    return [[v, a] for a in np.linspace(np.radians(t0), np.radians(t1), steps)]    

def main(args=None):
    print("*** Navigation - main()", flush=True)

    robot_name = None

    # # Accelerate
    # cmds = [[v, 0.] for v in np.linspace(0.001, 1.1, 50)]       # 50
    # v = cmds[-1][0]

    # # Drive straight
    # cmds.extend([cmds[-1]]*200)

    # # left
    # cmds.extend(turn(v, 0, 54.3, 9))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21])

    # # Drive straight
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*200)

    # # left
    # cmds.extend(turn(v, 0, 54.3, 9))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21])

    # # Drive straight
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*200)

    # # left
    # cmds.extend(turn(v, 0, 54.3, 9))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21])

    # # Drive straight
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*200)

    # # left
    # cmds.extend(turn(v, 0, 54.3, 9))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21])    

    # # stop
    # cmds.append([0,0])

    # Accelerate
    # cmds = [[v, vv] for v, vv in zip(np.linspace(0.001, 1.1, 50), np.linspace(0.001, 0.2, 50))]       # 50
    # # Drive in circle
    # cmds.extend([cmds[-1]]*550)

    # cmds = [[v, vv] for v, vv in zip(np.linspace(0.001, 1.1, 50), np.linspace(0.001, 0.02, 50))]
    # cmds.extend([cmds[-1]]*10000)
    # cmds.append([0,0])
    
    # cmds = []
    # for i in range(51):
    #     cmds.append([i / 50., 0.0])
    # for i in range(50):
    #     cmds.append([1 - i / 50., 0.0])
    # for i in range(51):
    #     cmds.append([-i / 50., 0.0])
    # for i in range(50):
    #     cmds.append([-1 + i / 50., 0.0])   
    # stop
    #cmds.append([0,0])  

    cmds = []
    cmds.extend(turn(0, 0, 25., 9))
    cmds.append([0,0])
    for i in range(51):
        cmds.append([3*i / 50., 0.0])
    cmds.extend([cmds[-1]]*10000)
    
    # # stop
    # cmds.append([0,0])    

    # TBD: handle multirobot case
    x = 5.5
    y = 3.5
    z = 2.0
    for i, arg in enumerate(sys.argv):
        if(arg == 'robot_name'):
            try:
                robot_name = sys.argv[i + 1]
            except IndexError:
                print("Error: robot_name argument requires a value")
                return
        elif(arg == '-x'):
            x = float(sys.argv[i + 1])
        elif(arg == '-y'):
            y = float(sys.argv[i + 1])
        elif(arg == '-z'):
            z = float(sys.argv[i + 1])

    if robot_name is None:
        #print("Error: robot_name argument not provided")
        #return
        robot_name = ""

    # print("All arguments:", sys.argv)

    rclpy.init(args=args)

    grid_size = 100
    timer_period = 0.1
    imu_ema_period = 10
    navigation = Navigation(robot_name, x, y, z, grid_size, timer_period, imu_ema_period, cmds)

    try:
        rclpy.spin(navigation)

    except KeyboardInterrupt:
        pass
    finally:
        navigation.destroy_node()
        rclpy.shutdown()

# ---

if __name__ == '__main__':
    main()

# ---
