# (c) robotics.snowcron.com
# Use: MIT license
 
import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from basic_navigator import BasicNavigator, NavigationResult # Helper module
from charger_navigator import ChargerNavigator #, BatteryStatusPublisher

#from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

def main(args=None):

    try:  
        rclpy.init(args=args)
        
        #executor = SingleThreadedExecutor()
        navigator = ChargerNavigator()
        #executor.add_node(navigator)

        try:
            #executor.spin()
            while(True):
                bResult = navigator.navigate_to_waypoints_or_charger()
                if(not bResult):
                    break
                time.sleep(0.1)
        finally:
            #executor.shutdown()
            navigator.destroy_node()
 
    finally:
        # Shutdown
        rclpy.shutdown()    
 
 
if __name__ == '__main__':
  main()




