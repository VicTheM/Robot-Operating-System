# (c) robotics.snowcron.com
# Use: MIT license

from ChargerNavigator import *

class ChargerNavigatorMulti(Node):
    
    def __init__(self):
        super().__init__('ChargerNavigatorMulti')

        # See navigation_bot_09 article for alternative approaches
        timer_period = 0.1
        # self.printedOnce = ""

        # ---
        robot1 = ChargerNavigator("robot1")
        #robot2 = ChargerNavigator("robot2")

        self.arrRobots = [ robot1 ] #, robot2 ]
        
        # Waypoints to cycle through

        self.timer = self.create_timer(timer_period, self.coordinator_callback)
        self.prev_time = datetime.now()
        time = self.get_clock().now().to_msg()

        goal_poses = []

        # TBD: pass this coord from actual robot (by subscribing once to its pose)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 0.91
        goal_pose.pose.position.y = 3.1
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.7
        goal_pose.pose.orientation.w = -0.7
        goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 6.8
        goal_pose.pose.position.y = 3.1
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = -1.0
        goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 6.8
        goal_pose.pose.position.y = -1.2
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.7
        goal_pose.pose.orientation.w = 0.6
        goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = -6.49
        goal_pose.pose.position.y = -0.09
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 1.0
        goal_pose.pose.orientation.w = 0.03
        goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = -6.5
        goal_pose.pose.position.y = 3.1
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.67
        goal_pose.pose.orientation.w = -0.7
        goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = -2.99
        goal_pose.pose.position.y = 2.86
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.16
        goal_pose.pose.orientation.w = -0.99
        goal_poses.append(goal_pose)

        # --- Go to this waypoint to start looking for charger

        goal_charger = PoseStamped()
        goal_charger.header.frame_id = 'map'
        goal_charger.header.stamp = time
        goal_charger.pose.position.x = 6.5
        goal_charger.pose.position.y = 5.5
        goal_charger.pose.position.z = 0.0
        goal_charger.pose.orientation.x = 0.0
        goal_charger.pose.orientation.y = 0.0
        goal_charger.pose.orientation.z = 0.9
        goal_charger.pose.orientation.w = 0.39

        robot1.setGoalPoses(goal_poses)
        #robot2.setGoalPoses(goal_poses[::-1])  # reverse

        robot1.setChargerPos(goal_charger)
        #robot2.setChargerPos(goal_charger)


    # ---

    # def printOnce(self, str):
    #     if(self.printedOnce != str):
    #         print(str)
    #     self.printedOnce = str

    # ---

    def coordinator_callback(self):
        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        # if delta_ms >= 2000:
        #     print("### coordinator_callback")
    # ---        
       
def main(args=None):

    try:  
        rclpy.init(args=args)

        executor = SingleThreadedExecutor()
        coordinator = ChargerNavigatorMulti()
        for robot in coordinator.arrRobots:
            executor.add_node(robot)
            #executor.add_node(robot.navigator)
        executor.add_node(coordinator)

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
            
            for robot in coordinator.arrRobots:
                robot.destroy_node()
            coordinator.destroy_node()
 
    finally:
        # Shutdown
        rclpy.shutdown()    
 
 
if __name__ == '__main__':
  main()
