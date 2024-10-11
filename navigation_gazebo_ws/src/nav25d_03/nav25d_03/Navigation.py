# (c) robotics.snowcron.com
# Based on Kalman and Bayesian Filters in Python, Roger R Labbe Jr
#
# Use: Commercial. This code can only be used if purchased on
# robotics.snowcron.com, in which case the code can only be 
# distributed as a part of a signifficantly larger package and
# with this (c) preserved. 
# Generally, this code is part of a tutorial and it is expected
# that it will help developers to create independent code of 
# their own. 
# Quoting small code fragments and refering the logic of a tutorial
# is permitted with proper reference to robotics.snowcron.com

# --- ROS2 node that uses Kalman filter for localization

import argparse
import sys

import numpy as np
from numpy.random import randn

import copy

import scipy.stats as stats
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation as R

import math
from math import sin, cos
from math import tan, atan2, pi, sqrt

import matplotlib.pyplot as plt

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import JulierSigmaPoints
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from filterpy.stats import plot_covariance_ellipse

import time
from datetime import datetime

# --- ROS2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image as Ros2MsgImage
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped  
from geometry_msgs.msg import PoseWithCovarianceStamped

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
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry

from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from sensor_msgs.msg import NavSatFix

from sensor_msgs.msg import Imu

# Path visualization
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import DeleteModel
from rclpy.duration import Duration

# --- As kalman.py was moved to a commercial section, this code either uses
# (commercial) kalman.py if it is present, or simulates it. Simulation is imperfect
# (it simply adds some errors) and will not work without Gazebo (it is cheating 
# by accessing ground truth position provided by Gazebo). It allows, however, 
# to run this and following simulations without purchasing access to a commercial 
# section. 

import os

# Checks if kalman.py is present and either uses it or runs quick and dirty
# simulator instead
bUseKalman = False

strCurrentDir = os.path.dirname(__file__)
strFilePath = os.path.join(strCurrentDir, "kalman.py")

if os.path.exists(strFilePath):
    bUseKalman = True
    from nav25d_03.kalman import KalmanNav
    print(">>>> Use Kalman", flush=True)
else:
    print(">>>> Simulate Kalman", flush=True)
    
from nav25d_03.kalman_utils import euler_from_quaternion
from nav25d_03.kalman_utils import euler_to_quaternion
from nav25d_03.kalman_utils import normalize_angle
from nav25d_03.kalman_utils import DynamicObject

# ---

# Set to 1, 2, 3... to run same trajectory multiple times, 
# for example, to run 10 circles instead of one, set it to 10.
nMaxCycles = 1

class Navigation(Node):

    def __init__(self, robot_name, x, y, z, grid_size, timer_period, 
        cmds, arrLandmarks, initPos, ellipse_step, bShowEllipses):

        print("*** Navigation.init()... ", flush=True)

        # Used to simulate accelerations. 
        # Note: it is still possible to use Gazebo-simulated accelerometer. 
        # But it only provides more or less accurate results if high accuracy
        # simulation is done, which requires more resources. To do it:
        # in imu_sensor.xacro, change
        #       <update_rate>10</update_rate>
        # to 
        #       <update_rate>100</update_rate>
        # in multi_simulation_launch.py, uncomment
        # '--',  # Separator between Gazebo args and ROS args
        # '--ros-args', '-p', 'publish_rate:=100.0'
        # in this file, set timer_period = 0.01 (100 Hz)
        # Then comment out the use of on_acceleration_callback()
        # and uncomment the use of accelerometer in imu_callback()
        self.last_gt_velocity = None
        self.current_gt_velocity = None
        
        # How often should error ellipses be displayed
        self.ellipse_step = ellipse_step

        # Works together with nMaxCycles (above)
        self.nCycleCount = 0

        # The way we work with namespaces was described in earlier sections
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

        # --- Used to present data as charts

        #self.track = []
        self.track_odom = []
        self.track_gt = []                  # "real" robot's position
        self.track_kalman = []              # predicted position
        
        self.arrChartTime         = []      # Integer numbers in seconds

        self.arrChartOdom         = []
        self.arrChartGpsError     = []
        self.arrChartKalman       = []
        self.arrChartGroundTruth  = []
        self.arrChartHeadingError = []

        # How many points chart already has
        self.nPointIdx = 0

        # --- Set colors for individual robots

        if(self.strRobotName == ""):
            self.path_color_0 = (0, 255, 0, 128)          # bgra, green
            self.path_color_1 = (255, 0, 0, 128)          # bgra, blue
            self.real_path_color = (0, 0, 255, 255)       # bgra, red
        elif(self.strRobotName == "robot1"):
            self.path_color_0 = (255, 255, 0, 128)        # bgra, cyan
            self.path_color_1 = (255, 0, 255, 128)        # bgra, magenta
            self.real_path_color = (0, 255, 255, 255)     # bgra, yellow
        else:
            self.path_color_0 = (0, 255, 0, 128)          # bgra, green
            self.path_color_1 = (255, 0, 0, 128)          # bgra, blue
            self.real_path_color = (0, 0, 255, 255)       # bgra, red

        # List of commands to move robot. Commands are issued at interval
        # self.nav.dt
        self.cmds = cmds
        self.nCurrentCommand = 0

        # --- Current position 
        
        initYaw = np.radians(0)
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, initYaw)

        current_pose = Pose()
        current_pose.position.x = x
        current_pose.position.y = y
        current_pose.position.z = z
        current_pose.orientation.x = qx
        current_pose.orientation.y = qy
        current_pose.orientation.z = qz
        current_pose.orientation.w = qw

        # This is a flag we use to make sure all subsctiptions work
        # For example, we do not pring robot's coordinates until it is set
        self.initial_pose_received = False

        # --- Used for reporting and to simulate sensor readings
        # NOT for real positioning (unless kalman.py is not available,
        # in which case it is used for "quick and dirty" simulation)
        self.ground_truth_pose = copy.deepcopy(current_pose)
        
        # TBD: in 3d world, odometry reports 2d data which is, technically,
        # not coordinates, but line on 2.5d landscape. So we might need to adjust
        # it at every step, to account for tilt/pitch (so we recalculate path
        # travelled to 2d coords on projection to horiz. plane)
        self.odom_pose = copy.deepcopy(current_pose)
        
        # Will be used in future
        # self.odom_twist = np.array([
        #     0,                              # initial linear speed x
        #     0                               # initial angular speed z
        # ])

        # --- GPS
        self.gps_pose = copy.deepcopy(current_pose)

        # As this is a Gazebo simulation, I will not bother with GPS subscription,
        # signal will be simulated instead. The reason is, Gazebo world if 
        # FLAT and Earth is a globe, and existing libraries to do proper
        # coordinates transformations are not very easy to use. In a tutorial
        # like this one it will result in unnecessarily complex code.
        # TBD: instead, it all has to work between spherical real world
        # and flat Gazebo simulation, but for now I will skip the part
        # where we receive lat/lon and transform it into x,y and assume that
        # we already have x,y
        
        # This flag is not currently used, but will be in future, when dealing with
        # sensor fusion; it allows to simulate the "GPS lost" situation
        self.bGpsAvailable = True

        # Setting up sensors policies: a very useful code snippet that 
        # helps avoiding problems with GPS and especially cameras.
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Adjust the depth according to your needs
            durability=QoSDurabilityPolicy.VOLATILE  # You can use TRANSIENT_LOCAL for durability
        )

        # Not used, but hypothetically, can be
        # self.gps_subscriber = self.create_subscription(
        #     NavSatFix,
        #     self.strSlashRobotName + '/gps/fix',
        #     self.gps_callback,
        #     qos_profile = qos_profile  # Set the queue size
        # )

        ## Do not trust this code!
        # # nZone = int((self.origin_longitude + 180) / 6) + 1
        # # self.wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
        # # self.utm = pyproj.Proj(proj="utm", zone=nZone, datum='WGS84')  # Change the zone according to your location
        # # self.origin_x, self.origin_y = pyproj.transform(self.wgs84, self.utm, self.origin_longitude, self.origin_latitude)

        # --- IMU

        # Note: In my examples, IMU has <update_rate>10 or 100</update_rate> depending 
        # on what we want; it should be in sync with Gazebo simulation rate in world and 
        # gazebo parameters in multi_simulation_launch.py

        # Note: currently, imu_callback provides Gyro data, while accelerometer
        # is simulated in on_acceleration_callback()
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

        # --- Simulated here, as Gazebo is not very friendly with magnetometers
        self.mag_yaw = 0.0
        
        # --- Odometry

        # Subscribe to messages of type nav_msgs/Odometry that provides 
        # position and orientation of the robot

        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.strSlashRobotName + '/odom',
            self.odom_callback,
            10)          

        # --- Ground Truth: used for charting and error (deviations) reporting.
        # It is obtained from Gazebo, in real world we will not have this sort
        # of information.

        ground_truth_pos_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.ground_truth_pose_callback,
            10  # QoS profile depth
        )

        # --- Publishing commands for dif. drive to use, same as "teleop" util does

        self.publisherCmdVel = self.create_publisher(
            Twist,
            self.strSlashRobotName + "/cmd_vel",           # For Gazebo diff. drive controller
            10) 

        # ---

        print("*** KalmanNav.init()... ", flush=True)

        # --- To be ignored for now

        ##self.tf_broadcaster = TransformBroadcaster(self)
        ##self.tf_buffer = Buffer()
        ##self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.robotToGlobalTransform = TransformStamped()    

        # ---

        # Comment in/out any sensor and in future, when fusion is done,
        # any combination 
        dictSensors = { 
            "gps": 2, 
            #"odom_pos_xy": 2,
            #"imu_accel": 2, 
            #"imu_gyro": 1,
            #"imu_magnet": 1,
            #"landmarks": 2 * len(landmarks) 
        }  

        # Slowdown multiplyer for particular sensors. If timer_period is
        # 0.1 Hz and nMagStep == 10, then new magnetometer data are 
        # available once per second.
        nGpsStep = 1
        nImuStep = 1
        nMagStep = 10
        nOdomStep = 1

        dt = timer_period

        # --- Sigmas for Kalman filter

        sigma_gps = 10.0

        sigma_odom_vxy = 0.03
        sigma_odom_wz = 0.03       

        # When we change from using acceleration measurements to velocity measurements 
        # (derived from acceleration), we need to adjust the measurement noise 
        # covariance accordingly. 
        # Integration effect: By integrating acceleration to get velocity, we accumulate
        # uncertainty over time.
        # Time step: The size of a time step (dt) affects how much uncertainty is accumulated.
        # Units: The units of velocity (e.g., m/s) are different from acceleration (e.g., m/s^2).
        # How to estimate the new sigma:
        # sigma_imu_vel = sigma_imu_accel * self.dt * np.sqrt(self.imu_step)
        # Here
        # sigma_imu_accel * self.dt accounts for the integration of acceleration 
        # to velocity over one time step.
        # np.sqrt(self.imu_step) accounts for the accumulation of error over multiple 
        # time steps (assuming independent errors).
        # For example, if:
        # sigma_imu_accel = 0.01 (m/s^2)
        # self.dt = 0.1 (seconds)
        # self.imu_step = 10 (meaning you update every 10th step)
        # Then:
        # sigma_imu_vel = 0.01 * 0.1 * np.sqrt(10) ≈ 0.00316 (m/s)
        sigma_imu_accel = 0.01
        sigma_imu_accel = sigma_imu_accel * dt * np.sqrt(nImuStep)

        accel_mean=0.0
        accel_mean = accel_mean * dt

        accel_bias_mean=0.0005
        accel_bias_mean = accel_bias_mean * dt

        accel_bias_stddev=0.00001
        accel_bias_stddev = accel_bias_stddev * dt

        # ---

        sigma_imu_gyro = 2e-4
        # Same logic as with accel above
        sigma_imu_gyro = sigma_imu_gyro * dt * np.sqrt(nImuStep)

        gyro_mean = 0.0
        gyro_mean = gyro_mean * dt

        gyro_bias_mean = 0.0000075 
        gyro_bias_mean = gyro_bias_mean * dt

        gyro_bias_stddev = 0.0000008        
        gyro_bias_stddev = gyro_bias_stddev * dt

        # ---
    
        sigma_imu_magnet = 0.1
    
        sigma_range=0.1 
        sigma_bearing=0.05    

        # ---

        self.initPos = copy.deepcopy(initPos)
        
        # --- This flag allows some additional dynamic adaptation of 
        # Kalman filter parameters. It definitely makes results better, 
        # though it is not an ultimate cure.

        bAdaptive = False

        if(bUseKalman):        
            self.nav = KalmanNav(arrLandmarks, dictSensors, initPos, 
                sigma_gps = sigma_gps,

                sigma_odom_vxy = sigma_odom_vxy,
                sigma_odom_wz = sigma_odom_wz,

                sigma_imu_accel = sigma_imu_accel,
                accel_mean=accel_mean, 
                accel_bias_mean=accel_bias_mean,
                accel_bias_stddev=accel_bias_stddev,

                sigma_imu_gyro = sigma_imu_gyro,
                gyro_mean = gyro_mean, 
                gyro_bias_mean = gyro_bias_mean,
                gyro_bias_stddev = gyro_bias_stddev,

                sigma_imu_magnet = sigma_imu_magnet,
                
                sigma_range=sigma_range, 
                sigma_bearing=sigma_bearing,
                
                dt=dt,
                gps_step=nGpsStep,
                imu_step=nImuStep,
                mag_step=nMagStep, 
                odom_step=nOdomStep,
                
                bUseAdaptive=bAdaptive)

            self.nav.bShowEllipses = bShowEllipses
        else:
            # Create object with same structure as KalmanNav
            # so existing code doesn't see the difference between
            # using kalman.py and simulating it.
            self.nav = DynamicObject(ukf={'x': initPos})
            #print(self.nav.ukf.x)

            self.nav.sigma_gps = sigma_gps

            self.nav.sigma_odom_vxy = sigma_odom_vxy
            self.nav.sigma_odom_wz = sigma_odom_wz
            self.nav.sigma_odom_pos_xy = sigma_odom_vxy * np.sqrt(dt)
            self.nav.sigma_odom_theta = sigma_odom_wz * np.sqrt(dt)

            self.nav.sigma_imu_accel = sigma_imu_accel
            
            self.nav.sigma_imu_gyro = sigma_imu_gyro
            
            self.nav.sigma_imu_magnet = sigma_imu_magnet

            self.nav.sigma_range = sigma_range 
            self.nav.sigma_bearing = sigma_bearing   

            self.nav.dt = dt         

        # We send commands by this timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.prev_time = self.get_clock().now()
        
        # To use in future with landmarks (unless it is done in another file)
        # self.camera_subscription = self.create_subscription(
        #     Ros2MsgImage,
        #     '/camera/image_raw',
        #     self.camera_callback,
        #     qos_profile_sensor_data)
        # self.camera_subscription

        # Publisher of initial pose, currently not used
        # self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
        #     self.strSlashRobotName + '/initialpose', 10)
            
        self.waitForNodesToStart()

        print("*** Navigation.init() done", flush=True)

    # --- To be used in future

    # def camera_callback(self, msg):
    #     print("*** Camera callback triggered! Performing task...", flush=True)

    # --- Not used, but might in future

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
    #     q = euler_to_quaternion(0, 0, self.nav.ukf.x[2])  # Roll, Pitch, Yaw
    #     self.robotToGlobalTransform.transform.rotation.x = q[0]
    #     self.robotToGlobalTransform.transform.rotation.y = q[1]
    #     self.robotToGlobalTransform.transform.rotation.z = q[2]
    #     self.robotToGlobalTransform.transform.rotation.w = q[3]

    #     # Note that we use this transform locally, without broadcasting it
    #     # otherwise it will interfere with same transforms Gazebo sends to RViz
    #     #self.tf_broadcaster.sendTransform(self.robotToGlobalTransform)

    # --- Reporting only

    def printPoseInfo(self):
        if(self.strRobotName != "" or self.initial_pose_received == False):
            return

        now = self.get_clock().now()
        duration = now - self.prev_time
        delta_ms = duration.to_msg().sec * 1000 + duration.to_msg().nanosec / 1e6

        if delta_ms >= 1000:

            ground_truth_pose = self.ground_truth_pose

            _, _, ground_truth_yaw = euler_from_quaternion(
                ground_truth_pose.orientation.x,
                ground_truth_pose.orientation.y,
                ground_truth_pose.orientation.z,
                ground_truth_pose.orientation.w)

            # ---

            odom_pose = self.odom_pose

            _, _, odom_yaw = euler_from_quaternion(
                odom_pose.orientation.x,
                odom_pose.orientation.y,
                odom_pose.orientation.z,
                odom_pose.orientation.w)

            # ---

            gps_pose = self.gps_pose  

            # ---
            
            #print("*** Status: %s; " % "working...") #self.navStatus)
            print("{:<16} {:<7} {:<7} {:<7} {:<7}".format("", "X", "Y", "Z", "Yaw"))
            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7.2f}".format("Odom:", odom_pose.position.x, odom_pose.position.y, 0.0, 
                math.degrees(normalize_angle(odom_yaw))), flush=True)

            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7}".format("GPS:", gps_pose.position.x, gps_pose.position.y, gps_pose.position.z, "---"))
            
            #print("{:<16} {:<7.2f} {:<7.2f} {:<7} {:<7}".format("IMU:", self.imu_pos[0], self.imu_pos[1], "---", "---"))
            
            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7.2f}".format("Kalman:", 
                self.nav.ukf.x[0], self.nav.ukf.x[1], 0.0, math.degrees(normalize_angle(self.nav.ukf.x[2]))), flush=True)

            print("{:<16} {:<7.2f} {:<7.2f} {:<7.2f} {:<7.2f}\n===\n".format("Ground Truth:", 
                ground_truth_pose.position.x, ground_truth_pose.position.y,
                ground_truth_pose.position.z, math.degrees(ground_truth_yaw)), flush=True)

            # --- Here we only do charting for a single robot simulation
            # Might change in future though currently I see no reason for it

            if(self.strRobotName == "" and self.nPointIdx > 0):
                self.arrChartTime.append(len(self.arrChartTime))
                               
                self.arrChartKalman.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - self.nav.ukf.x[0])**2 + 
                        (ground_truth_pose.position.y - self.nav.ukf.x[1])**2)) 
                
                self.arrChartGpsError.append(
                    math.sqrt(
                        (ground_truth_pose.position.x - gps_pose.position.x)**2 + 
                        (ground_truth_pose.position.y - gps_pose.position.y)**2))

                self.arrChartHeadingError.append(np.degrees(abs(normalize_angle(ground_truth_yaw - self.nav.ukf.x[2]))))

            self.nPointIdx += 1

    # --- Currently not used, as I simulate GPS for flat world 
    # instead of dealing with spherical one. This is a simple
    # simulation, more accurate are available in specialized 
    # libraries

    # def lat_lon_to_xy(self, latitude, longitude):

    #     # Earth radius in meters (approximate)
    #     earth_radius = 6371000.0

    #     # Convert degrees to radians
    #     lat1 = math.radians(latitude)
    #     lon1 = math.radians(longitude)
    #     lat2 = math.radians(self.origin_latitude)
    #     lon2 = math.radians(self.origin_longitude)

    #     # Calculate the differences in latitude and longitude
    #     delta_lat = lat1 - lat2
    #     delta_lon = lon1 - lon2

    #     # Calculate X and Y coordinates in meters
    #     x = -earth_radius * delta_lon * math.cos((lat1 + lat2) / 2)
    #     y = -earth_radius * delta_lat

    #     return x, y

    # --- Wait for robot to get its initial coordinates from 
    # any source, like GPS, landmarks, external knowlege (you are 
    # parked here) and so on.

    def _waitForInitialPose(self):
        print("\t***_waitForInitialPose...")
        while not self.initial_pose_received:
            print("\t\t***Waiting for initial pose...")
            rclpy.spin_once(self, timeout_sec=1.0)
            time.sleep(1)
        print("\tInitial pose received")
        return

    # --- In addition to _waitForInitialPose, other 
    # conditions can be used here.

    def waitForNodesToStart(self):
        print("*** Navigation.waitForNodesToStart()...")
        self._waitForInitialPose()
        print("*** Navigation.waitForNodesToStart() done")

    # ------    

    def _setInitialPose(self, pose):
                
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = self.strRobotNameSlash + 'map'
        
        # In ROS2, to work with time, it is recommended to use the Time class from rclpy.time 
        # rather than the Python standard library time module or datetime module.
        # Here's why:
        # ROS2 provides its own time management system to handle both real time and simulated time. 
        # ROS2 API Compatibility: ROS2 messages and other ROS2 components (e.g., publishers, 
        # subscribers, and actions) use rclpy.time.Time, so using it ensures compatibility.
        
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
        #self.initial_pose_pub.publish(msg)

        self.initial_pose_received = True

    # --- Currently not used because corresponding sensors' data
    # is simulated.

    # def transform_vector_from_robot_to_global(self, vector, translation, quaternion):
    #     rotation = R.from_quat(quaternion)
    #     rotation_matrix = rotation.as_matrix()
        
    #     # Apply rotation
    #     rotated_vector = rotation_matrix.dot(vector)
        
    #     # Apply translation: no translation as we use derivatives (vel, accel)
    #     global_vector = rotated_vector #+ translation
        
    #     return global_vector

    # --- Note that some code is commented out because I simulate accelerometer
    # myself instead of using Gazebo simulator.

    def imu_callback(self, msg):
        timeCurrent = self.get_clock().now()
        dt = (timeCurrent - self.timeImuLast).nanoseconds * 1e-9  # nanoseconds to seconds

        self.timeImuLast = timeCurrent

        # Get linear acceleration in robot frame
        
        # Comment to to simulate accelerometer instead of using Gazebo
        # ax = msg.linear_acceleration.x
        # ay = msg.linear_acceleration.y
        # az = msg.linear_acceleration.z
        # --- Comment till here
        
        # Uncomment to use Gazebo simulated accelerometer
        # ax, ay, az = self.ground_truth_acceleration

        try:
            # Comment to to simulate accelerometer instead of using Gazebo

            # vector = [ax, ay, az]
            # translation = np.array([0.0, 0.0, 0.0])
            #     #np.array([self.nav.ukf.x[0], self.nav.ukf.x[1], 0.0])
            
            # q = euler_to_quaternion(0, 0, self.nav.ukf.x[2])  # Roll, Pitch, Yaw
            # self.robotToGlobalTransform.transform.rotation.x = q[0]
            # self.robotToGlobalTransform.transform.rotation.y = q[1]
            # self.robotToGlobalTransform.transform.rotation.z = q[2]
            # self.robotToGlobalTransform.transform.rotation.w = q[3]
            # quaternion = np.array([q[0], q[1], q[2], q[3]])

            # imu_data_global = self.transform_vector_from_robot_to_global(vector, translation, quaternion)

            # ax = imu_data_global[0]
            # ay = imu_data_global[1]
            # az = imu_data_global[2] - 9.81

            # # Global frame
            # self.imu_data[0] = ax
            # self.imu_data[1] = ay

            # --- Comment till here

            angular_velocities = msg.angular_velocity
            self.imu_data[2] = angular_velocities.z
            
            # Currently not used, but can be used in future

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

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose

        self.odom_pose.position.x += self.initPos[0]
        self.odom_pose.position.y += self.initPos[1]

        if(self.initial_pose_received == False):
            self.current_pose = self.odom_pose
            self._setInitialPose(self.current_pose)

        # Currently, only odometry position is used, but not twist
        # self.odom_twist = np.array([
        #     msg.twist.twist.linear.x,
        #     msg.twist.twist.angular.z
        # ])

        # print("*****", msg.twist.twist.linear.x, 
        #     math.degrees(msg.twist.twist.angular.z), flush=True)

    # --- Currently not used, as simulated instead in on_gps_callback()

    # def gps_callback(self, msg):

    #     x, y = self.lat_lon_to_xy(msg.latitude, msg.longitude)

    #     self.gps_pose.position.x = x
    #     self.gps_pose.position.y = y
        
    #     # For convenience, Gazebo model is at 0 meters altitude
    #     self.gps_pose.position.z = msg.altitude - self.origin_altitude
    #     self.gps_pose.orientation.x = 0.0
    #     self.gps_pose.orientation.y = 0.0
    #     self.gps_pose.orientation.z = 0.0
    #     self.gps_pose.orientation.w = 1.0

    #     if(self.initial_pose_received == False):
    #         self.current_pose = self.gps_pose
    #         self._setInitialPose(self.gps_pose)
            
    # --- Simulate GPS, instead of using Gazebo
    # TBD: Currently, we set initial pose here. It is ok, as it is only used in
    # reporting, not in navigation. But in future, it should be set using
    # a separate function, as we can choose to not use GPS. The way it is now
    # is fine, but "not nice", so it is not a bug.
    def on_gps_callback(self):

        if(self.bGpsAvailable):
            self.gps_pose.position.x = self.ground_truth_pose.position.x + np.random.normal(0, self.nav.sigma_gps, size=(1))[0]
            self.gps_pose.position.y = self.ground_truth_pose.position.y + np.random.normal(0, self.nav.sigma_gps, size=(1))[0]
        else:
            self.gps_pose.position.x = self.nav.ukf.x[0]
            self.gps_pose.position.y = self.nav.ukf.x[1]

        if(self.initial_pose_received == False):
            self._setInitialPose(self.gps_pose)

    # ---

    def ground_truth_pose_callback(self, msg):
        strName = self.strRobotName
        if(self.strRobotName == ""):
            strName = "nav25d_03"
        robot_index = msg.name.index(strName)
        self.ground_truth_pose = msg.pose[robot_index]

        # ---

        # Get current velocity and time
        self.current_gt_velocity = msg.twist[robot_index].linear

    # --- Simulating accelerometer, instead of using Gazebo

    def on_acceleration_callback(self):

        # Calculate acceleration if we have previous velocity data
        if(self.last_gt_velocity is not None and self.current_gt_velocity is not None):
            
            # Calculate acceleration
            accel = Vector3()
            accel.x = (self.current_gt_velocity.x - self.last_gt_velocity.x) / self.nav.dt
            accel.y = (self.current_gt_velocity.y - self.last_gt_velocity.y) / self.nav.dt
            accel.z = (self.current_gt_velocity.z - self.last_gt_velocity.z) / self.nav.dt

            acceleration = [accel.x, accel.y, 0.0]
            accel_data = self.nav.accel_sim.simulate(acceleration)

            # Store the calculated acceleration
            self.imu_data[0] = accel_data[0]
            self.imu_data[1] = accel_data[1]

        self.last_gt_velocity = self.current_gt_velocity

    # --- Charts are displayed in two places. Here we display time series (like GPS error vs time)
    # and later in timer_callback() we also display robot's path. That path is a temporary measure,
    # in future it should be displayed at a higher level, so that trajectories for multiple robots
    # could be displayed on the same chart.

    def showCharts(self):
        if(self.strRobotName == ""):
            # Create a figure and axis
            fig, ax = plt.subplots()

            # Plot the data with different colors and labels
            
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

            # ax.plot(self.arrChartTime, self.arrChartGpsError, label='GPS Error', color='red', linestyle='-')

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
            self.arrChartGpsError          = []
            self.arrChartHeadingError = []


    # --- Publish command for dif. drive to use   

    def drive(self, linear_speed, angular_speed):
        msg = Twist()
        msg.linear.x = float(linear_speed)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_speed)
         
        self.publisherCmdVel.publish(msg) 

    # --- Simulate magnetometer, instead of using Gazebo

    def on_magnetometer_callback(self):
        _, _, ground_truth_yaw = euler_from_quaternion(
            self.ground_truth_pose.orientation.x,
            self.ground_truth_pose.orientation.y,
            self.ground_truth_pose.orientation.z,
            self.ground_truth_pose.orientation.w)

        imuMag = self.nav.transform_magnetic_field(
            self.nav.world_magnet, 
            (ground_truth_yaw + np.random.normal(0, self.nav.sigma_imu_magnet, size=(1)))[0])

        self.mag_yaw = normalize_angle(-np.arctan2(imuMag[0], imuMag[1]))

    # --- Runs by timer, one navigation command in a time

    def timer_callback(self):
        global nMaxCycles

        # Gazebo is odd when it comes to GPS to spherical xyz, so simulate one
        # We need to call something like that here in order to get initial
        # position and to be able to start.
        # TBD: in this example we KNOW the initial position, so we can set it 
        # directly. In some other example we might have to rely on GPS, or 
        # something else. So ideally, there should be a function to call,
        # so on_gps_callback() can be moved inside the "if" statement below.
        self.on_gps_callback()
        if(not self.initial_pose_received):
            return

        if(self.nCurrentCommand < len(self.cmds) and self.nCycleCount < nMaxCycles):

            cmd = self.cmds[self.nCurrentCommand]
            linear_speed = cmd[0]
            angular_speed = cmd[1]

            # Set so that when Kalman calls "move()", it uses slippage
            self.nav.nSlip = 1

            self.drive(linear_speed, angular_speed)
            self.nCurrentCommand += 1

            # Not implemented yet
            # for i, u in enumerate(cmds):
            #     # Set random elements in self.arrShowLandmarks to False (hide landmarks)
            #     nHideLandmarks = np.random.randint(0, 4)
            #     self.arrShowLandmarks = np.ones(len(self.landmarks), dtype=bool)
            #     arrFalse = np.random.choice(np.arange(len(self.landmarks)), size=nHideLandmarks, replace=False)
            #     self.arrShowLandmarks[arrFalse] = False

            if(bUseKalman):
                self.nav.nCurrentKalmanStep += 1

            self.track_gt.append(np.array([ self.ground_truth_pose.position.x, self.ground_truth_pose.position.y]))
            self.track_odom.append(np.array([ self.odom_pose.position.x, self.odom_pose.position.y]))
            
            # ---

            kalman_prev = copy.deepcopy(self.nav.ukf.x)

            if(bUseKalman):
                self.nav.ukf.predict(u=cmd)
            else:
                # A very primitive "quick and dirty" simulation of a Kalman filter
                # It is "just" to have a line on a map, so that future simulations can
                # work without (commercial) kalman.py
                ground_truth_pose = self.ground_truth_pose

                _, _, ground_truth_yaw = euler_from_quaternion(
                    ground_truth_pose.orientation.x,
                    ground_truth_pose.orientation.y,
                    ground_truth_pose.orientation.z,
                    ground_truth_pose.orientation.w)

                self.nav.ukf.x[0] = ground_truth_pose.position.x + np.random.normal(0, 0.5, size=(1))[0]
                self.nav.ukf.x[1] = ground_truth_pose.position.y + + np.random.normal(0, 0.5, size=(1))[0]
                self.nav.ukf.x[2] = ground_truth_yaw + np.random.normal(0, 0.01, size=(1))[0]
                self.nav.ukf.x[3] = (ground_truth_pose.position.x - kalman_prev[3]) / self.nav.dt
                self.nav.ukf.x[4] = (ground_truth_pose.position.y - kalman_prev[4]) / self.nav.dt
                self.nav.ukf.x[5] = (ground_truth_yaw - kalman_prev[5]) / self.nav.dt
                self.nav.ukf.x[6] = (self.nav.ukf.x[6] - kalman_prev[6]) / self.nav.dt
                self.nav.ukf.x[7] = (self.nav.ukf.x[7] - kalman_prev[7]) / self.nav.dt
                self.nav.ukf.x[8] = 1.0
                self.nav.ukf.x[9] = 1.0
                
            # Not implemented yet
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
            #     #     normalize_angle(self.nav.dictState["prev_heading"] + odom_angular_z * self.nav.dt)])

            bNeedUpdate = False

            # For sensors that are enabled, set sensor values to be passed to 
            # Kalman's update()
            if(bUseKalman):
                if("gps" in self.nav.dictSensors):
                    if(self.nav.nCurrentKalmanStep % self.nav.gps_step == 0):
                        gpsPos = np.array([ self.gps_pose.position.x, self.gps_pose.position.y ])
                        bNeedUpdate = True
                    else:
                        gpsPos = self.nav.ukf.x[:2]
                    z.extend(gpsPos)

                if("odom_pos_xy" in self.nav.dictSensors):
                    if(self.nav.nCurrentKalmanStep % self.nav.odom_step == 0):
                        odomPos = np.array([ self.odom_pose.position.x, self.odom_pose.position.y ])
                        bNeedUpdate = True
                    else:
                        odomPos = self.ukf.x[:2]

                    z.extend(odomPos[:2])                     

                if("imu_accel" in self.nav.dictSensors):
                    self.on_acceleration_callback()

                    if(self.nav.nCurrentKalmanStep % self.nav.imu_step == 0):
                        accel_data = [self.imu_data[0], self.imu_data[1]]
                        imuAccel = [
                            kalman_prev[3] + accel_data[0] * self.nav.dt,
                            kalman_prev[4] + accel_data[1] * self.nav.dt                        
                        ]

                        bNeedUpdate = True
                    else:
                        imuAccel = self.ukf.x[3:5]

                    z.extend(imuAccel)

                if("imu_gyro" in self.nav.dictSensors):
                    if(self.nav.nCurrentKalmanStep % self.nav.imu_step == 0):
                        gyro_yaw = kalman_prev[2] + self.imu_data[2] * self.nav.dt
                        gyro_yaw = normalize_angle(gyro_yaw)

                        bNeedUpdate = True
                    else:
                        gyro_yaw = self.nav.ukf.x[2]

                    z.extend([gyro_yaw])

                if("imu_magnet" in self.nav.dictSensors):
                    # Gazebo has no magnetometer worth mentioning, so simulate one
                    # Note that we do it here, while on_gps_callback() was called above.
                    # the reason is, we needed to get initial position. If there is another way
                    # of calling set initial pos, it can be used, then GPS simulation
                    # can be moved here.
                    self.on_magnetometer_callback()

                    if(self.nav.nCurrentKalmanStep % self.nav.mag_step == 0):

                        mag_yaw = self.mag_yaw
                        yaw = mag_yaw

                        bNeedUpdate = True

                    else:
                        yaw = self.nav.ukf.x[2]

                    z.extend([yaw])

                # Not implemented yet
                # if("landmarks" in self.nav.dictSensors):
                #     for lmark in self.nav.landmarks:
                #         dx, dy = lmark[0] - x, lmark[1] - y
                #         d = sqrt(dx**2 + dy**2) + randn()*self.nav.sigma_range
                #         bearing = atan2(lmark[1] - y, lmark[0] - x)
                #         a = (normalize_angle(bearing - self.sim_pos[2] + randn()*self.sigma_bearing))
                #         z.extend([d, a])
                
            if(bNeedUpdate == True):
                self.nav.ukf.update(z, u = cmd, dt=self.nav.dt)
            
            if(bNeedUpdate == True or not bUseKalman):
                self.track_kalman.append(np.array([self.nav.ukf.x[0], self.nav.ukf.x[1]]))

            # Currently not used
            # self.broadcast_transform()

            if(self.nav.bShowEllipses and self.nav.nCurrentKalmanStep % self.ellipse_step == 0):
                plot_covariance_ellipse((self.nav.ukf.x[0], self.nav.ukf.x[1]), self.nav.ukf.P[0:2, 0:2], std=6, facecolor='g', alpha=0.8)

            self.printPoseInfo()
        
        # If all commands were sent, check if we need to run more cycles
        elif(self.nCycleCount < nMaxCycles):
            self.nCurrentCommand = 0
            self.nCycleCount += 1

        else:
            # This code draws track (ground truth trajectory)
            track_gt = np.array(self.track_gt)
            plt.plot(track_gt[:, 0], track_gt[:,1], color='k', lw=2)

            track_odom = np.array(self.track_odom)
            plt.plot(track_odom[:, 0], track_odom[:,1], color='g', lw=2)            

            # track = np.array(self.track)
            # plt.plot(track[:, 0], track[:,1], color='b', lw=2)

            track_kalman = np.array(self.track_kalman)
            plt.plot(track_kalman[:, 0], track_kalman[:,1], color='m', lw=2)

            plt.axis('equal')
            plt.title("Trajectory")

            # this code shows both track and ellipses on the same chart,
            # because of the way FilterPy draws ellipses on whatever plot
            # is awailable.
            plt.show()

            self.showCharts()            

            #print('final covariance', self.nav.ukf.P.diagonal())

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
    global nMaxCycles

    print("*** Navigation - main()", flush=True)

    robot_name = None
    nMaxCycles = 1

    # timer_period = 0.01
    timer_period = 0.1

    dTimerCoef = int(0.1 / timer_period)

    # --- Rectangle ----------------------

    arrLandmarks = np.array([[5, 10], [10, 5], [15, 15], [20, 5], [0, 30], [50, 30], [40, 10]])

    # This path is for dif2wd

    # accelerate from a stop
    cmds = []
    for i in range(51):
        cmds.append([i / 25., 0.])

    for i in range(4):
        # Drive straight
        cmds[-1][1] = 0.
        cmds.extend([cmds[-1]]*100)

        # turn left
        v = cmds[-1][0]
        cmds.extend(turn(v, 0, 37.3, 50))   

    # stop
    cmds.append([0,0])    

    # This path is for dif4wd
    # # Accelerate
    # cmds = [[v, 0.] for v in np.linspace(0.001, 1.1, 50 * dTimerCoef)]       # 50
    # v = cmds[-1][0]

    # # Drive straight
    # cmds.extend([cmds[-1]]*200 * dTimerCoef)

    # # left
    # cmds.extend(turn(v, 0, 50.5, 9 * dTimerCoef))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21 * dTimerCoef])

    # # Drive straight
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*200 * dTimerCoef)

    # # left
    # cmds.extend(turn(v, 0, 50.5, 9 * dTimerCoef))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21 * dTimerCoef])

    # # Drive straight
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*200 * dTimerCoef)

    # # left
    # cmds.extend(turn(v, 0, 50.5, 9 * dTimerCoef))
    # cmds.extend([v.copy() for v in [cmds[-1]] * 21 * dTimerCoef])

    # # Drive straight
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*200 * dTimerCoef)

    # # left
    # cmds.extend(turn(v, 0, 50.5, 9 * dTimerCoef))
    # #cmds.extend([v.copy() for v in [cmds[-1]] * 21])    

    # stop
    cmds.append([0,0])

    # --- Circle -------------------------

    # arrLandmarks = np.array([[5, 10], [10, 5], [15, 15], [20, 5], [0, 30], [50, 30], [40, 10]])

    # This path is for dif4wd

    # # Accelerate
    # cmds = []
    # for i in range(21 * dTimerCoef):
    #     cmds.append([i / 10. * dTimerCoef, 0. ])
    
    # cmds.append([2.0, 0.2])

    # # Drive in circle
    # cmds.extend([cmds[-1]]*500 * dTimerCoef)

    # # Stop
    # cmds.append([0,0])

    # --- Large Circle -------------------------

    # arrLandmarks = np.array([
    #     [-200,   0], [0,   0], [200,   0], [400,   0], 
    #     [-200, 200], [0, 200], [200, 200], [400, 200], 
    #     [-200, 400], [0, 400], [200, 400], [400, 400], 
    # ]) 

    # This path is for dif4wd
    # # Accelerate
    # cmds = []
    # cmds.append([10.0, 0.1])

    # # Drive in circle
    # cmds.extend([cmds[-1]]*1200 * dTimerCoef)

    # # Stop
    # cmds.append([0,0])

    # -----------------------------------    

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
    
    initPos = [
        x, y, np.radians(0), 
        0., 0., np.radians(0), 
        0., 0.,
        1.0, 1.0
    ]

    ellipse_step = 500

    bShowEllipses = False

    # TBD: when simulation becomes 3d, z,y,z should be removed, as initPos will have them
    navigation = Navigation(robot_name, x, y, z, grid_size, timer_period, cmds, arrLandmarks, initPos, ellipse_step, bShowEllipses)

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
