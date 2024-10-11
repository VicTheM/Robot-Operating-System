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

class ImgLayersManager(Node):

    def __init__(self, arrTopicsIn, strTopicOut, img_width, img_height, n_channels, nOutFrequency):
        print("*** ImgLayersManager.init()")

        # ---

        super().__init__("ImgLayersManager")

        # ---

        self.img_height = img_height
        self.img_width = img_width
        self.n_channels = n_channels

        #self.strTopicOut = strTopicOut
        self.imgCanvas = np.zeros((self.img_height, self.img_width, self.n_channels), dtype=np.uint8)
        self.bridge = CvBridge()
        self.frame_num = 0

        self.nOutFrequency = nOutFrequency

        self.arrImages = []
        for strTopic in arrTopicsIn:
            subscriber = self.create_subscription(
                Image, strTopic, self.incomingImageCallback,
                10  # QoS profile depth
            )        

            self.arrImages.append({ 'topic': strTopic, 'image': None, 'subscriber': subscriber })

        self.canvas_publisher = self.create_publisher(Image, strTopicOut, 10)

        # ---

        timer_period = 1.0 / nOutFrequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bNeedUpdate = False

        # ---

        print("*** ImgLayersManager.init(): done")

    # ------

    def incomingImageCallback(self, msg):
        strTopic = msg.header.frame_id
        #print(">>> Incoming image: ", strTopic)
                
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # If the image has 3 channels (RGB), add an alpha channel
        if cv_image.shape[2] == 3:
            alpha_channel = np.ones((cv_image.shape[0], cv_image.shape[1]), dtype=cv_image.dtype) * 255
            cv_image = cv2.merge((cv_image, alpha_channel))

        if(cv_image.shape[0] != self.img_height
            or cv_image.shape[1] != self.img_height):
            cv_image = cv2.resize(cv_image, (self.img_width, self.img_height))

        # Now cv_image is a 4-channel (RGBA) NumPy array
        
        nFound = -1
        for i, topic in enumerate(self.arrImages):
            if(topic['topic'] == strTopic):
                nFound = i
                break

        if(nFound != -1):
            self.arrImages[nFound]['image'] = cv_image
            self.bNeedUpdate = True
        #else:
        #    print(">>> Not found") 


    # ------

    def addTopic(self, strTopic):
        #print(">>> Adding", strTopic)
        bFound = False
        for topic in self.arrImages:
            if(topic['topic'] == strTopic):
                bFound = True
                break
        #print(">>>", bFound)

        if(bFound == False):
            subscriber = self.create_subscription(
                Image, strTopic, self.incomingImageCallback,
                10  # QoS profile depth
            )

            #print(">>> topic", strTopic, " added")

            self.arrImages.append({ 'topic': strTopic, 'image': None, 'subscriber': subscriber })

    # ------

    def removeTopic(self, strTopic):
        nFound = -1
        for i, topic in enumerate(arrImages):
            if(topic['topic'] == strTopic):
                nFound = i
                break

        if(nFound != -1):
            arrImages[nFound]['subscriber'].destroy_subscription()
            del arrImages[nFound]
            self.bNeedUpdate = True

    # ------

    def timer_callback(self):
        if(self.bNeedUpdate == False):
            return
        # ---

        self.imgCanvas = np.zeros((self.img_height, self.img_width, self.n_channels), dtype=np.uint8)

        self.bNeedUpdate = False
        for topic in self.arrImages:
            #print(">>> Processing topic", topic["topic"])
            if(topic['image'] is None):
            #    print(">>> topic doesn't tave image")
                continue
            
            arrNonTransparentIdxs = topic['image'][:, :, 3] > 0
            self.imgCanvas[arrNonTransparentIdxs] = topic['image'][arrNonTransparentIdxs]
            self.bNeedUpdate = True
                        
            # Alternative approach that respects transparency, not tested
            # # Extract BGRA channels from the input image
            # b, g, r, a = cv2.split(input_image)

            # # Blend the input image onto the destination image using the alpha channel
            # destination_image[:, :, 0] = (1 - a / 255.0) * destination_image[:, :, 0] + (a / 255.0) * b
            # destination_image[:, :, 1] = (1 - a / 255.0) * destination_image[:, :, 1] + (a / 255.0) * g
            # destination_image[:, :, 2] = (1 - a / 255.0) * destination_image[:, :, 2] + (a / 255.0) * r
            # destination_image[:, :, 3] = 255  # Set the alpha channel of the destination image to 255


        # ---

        if(self.bNeedUpdate == True):
            msg = self.bridge.cv2_to_imgmsg(self.imgCanvas, "bgra8")
            msg.header.frame_id = str(self.frame_num)
            self.frame_num += 1        
            self.canvas_publisher.publish(msg)                

            cv2.imwrite("./transparent_img.png", self.imgCanvas)

            self.bNeedUpdate = False

    # ------

def main(args=None):
    print("*** ImgLayerManager - main()")

    rclpy.init(args=args)
    imgLayerManager = ImgLayerManager([], "", 640, 480, 4, 5)

# ---

if __name__ == '__main__':
    main()
