import time  # Time library
 
from rclpy.duration import Duration
import rclpy                                # Python client library for ROS 2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from basic_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped   # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist         # Velocity command
from sensor_msgs.msg import BatteryState    # Battery status
import time
from datetime import datetime

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

class ChargerNavigator(Node):
    def __init__(self):
        super().__init__('ChargerNavigator')
        self.nBatteryLevel = 100.
        timer_period = 0.1
        self.navStatus = "not_started"

        self.navigator = BasicNavigator()

        # Set the robot's goal poses

        self.goal_poses = []

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 4.26
        goal_pose.pose.position.y = -0.9900000000000011
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.6743382882342813
        goal_pose.pose.orientation.w = -0.7384225572267272
        self.goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 6.0
        goal_pose.pose.position.y = 0.35999999999999943
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.998758526924799
        goal_pose.pose.orientation.w = -0.0498137018801605
        self.goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 4.859999999999999
        goal_pose.pose.position.y = 3.0599999999999996
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.6943732675901293
        goal_pose.pose.orientation.w = -0.7196150118335545
        self.goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 3.51
        goal_pose.pose.position.y = 1.3099999999999996
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.1534180888727564
        goal_pose.pose.orientation.w = -0.988161368404286
        self.goal_poses.append(goal_pose)

        # ---
        self.goal_charger = PoseStamped()
        self.goal_charger.header.frame_id = 'map'
        self.goal_charger.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_charger.pose.position.x = 1.7599999999999998
        self.goal_charger.pose.position.y = 6.359999999999999
        self.goal_charger.pose.position.z = 0.0
        self.goal_charger.pose.orientation.x = 0.0
        self.goal_charger.pose.orientation.y = 0.0
        self.goal_charger.pose.orientation.z = 0.9356050226785755
        self.goal_charger.pose.orientation.w = 0.3530484974314751

        # We are going to publish this value. Then we are going to subscribe to it as 
        # to self.nBatteryLevel. In a real robot, this will be published by node that
        # reads the charge of a real battery.
        self.nBatteryLevelCalculated = 100.
        self.nBatteryLowTreshold = 20.

        # After we done recharging, continue from this waypoint
        self.nNextWaypointIdx = 0

        # Used to publish status
        self.prev_time = datetime.now()

        #self.timer = self.create_timer(timer_period, self.navigate_to_waypoints_or_charger)
        self.subscription_battery_status = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.get_battery_status,
            10)
        self.publisherBatteryLevel = self.create_publisher(BatteryState, '/battery_status', 10)

    def get_battery_status(self, msg: BatteryState):
        self.nBatteryLevel = msg.percentage

    def publishBatteryStatus(self):
        self.nBatteryLevelCalculated -= 0.1
        msg = BatteryState()
        msg.percentage = self.nBatteryLevelCalculated
        self.publisherBatteryLevel.publish(msg)

       
    def navigate_to_waypoints_or_charger(self):
        self.publishBatteryStatus()
        
        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        if delta_ms >= 1000:
            print(">>> Battery at %.0d" % (self.nBatteryLevel))
            self.prev_time = now

        if(self.navStatus == "not_started"):

            # Set the robot's initial pose if necessary
            # initial_pose = PoseStamped()
            # initial_pose.header.frame_id = 'map'
            # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            # initial_pose.pose.position.x = -6.5
            # initial_pose.pose.position.y = -4.2
            # initial_pose.pose.position.z = 0.0
            # initial_pose.pose.orientation.x = 0.0
            # initial_pose.pose.orientation.y = 0.0
            # initial_pose.pose.orientation.z = 0.0
            # initial_pose.pose.orientation.w = 1.0
            # self.navigator.setInitialPose(initial_pose)
        
            # Activate navigation, if not autostarted. This should be called after setInitialPose()
            # or this will initialize at the origin of the map and update the costmap with bogus readings.
            # If autostart, you should `waitUntilNav2Active()` instead.
            # self.navigator.lifecycleStartup()
            
            # Wait for navigation to fully activate. Use this line if autostart is set to true.
            self.navigator.waitUntilNav2Active()
      
            # If desired, you can change or load the map as well
            # self.navigator.changeMap('/path/to/map.yaml')
            
            # You may use the self.navigator to clear or obtain costmaps
            # self.navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
            # global_costmap = self.navigator.getGlobalCostmap()
            # local_costmap = self.navigator.getLocalCostmap()

            self.navStatus = "ready"

        elif((self.navStatus == "ready" or self.navStatus == "navigating") 
            and self.nBatteryLevel <= self.nBatteryLowTreshold):
            
            print("Battery low")
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
                
                if feedback and delta_ms >= 1000:
                    print('Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(self.goal_poses)))
            else:
                self.navStatus = "completed"

        elif(self.navStatus == "preparing_to_go_to_charger"):
            print('Cancelling nav. before going to charger')
            if(self.navigator.isNavComplete()):
                self.navStatus = "going_to_charger"
                self.navigator.goToPose(self.goal_charger)   

        elif(self.navStatus == "going_to_charger"):
            if(self.navigator.isNavComplete()):
                self.nBatteryLevelCalculated = 100.
                self.navStatus = "completed"

        elif(self.navStatus == "completed"):
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print('Goal succeeded!')
                self.navStatus = "ready"
            elif result == NavigationResult.CANCELED:
                print('Goal was canceled!')
                self.timer.cancel()
                return False
            elif result == NavigationResult.FAILED:
                print('Goal failed!')
                self.timer.cancel()
                return False
            else:
                print('Goal has an invalid return status!')
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
 
