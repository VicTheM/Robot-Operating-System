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
        robot2 = ChargerNavigator("robot2")

        self.arrRobots = [ 
            { 'robot': robot1, 'waypoint_idx': 0 },
            { 'robot': robot2, 'waypoint_idx': 2 } 
        ]

        # Waypoints to cycle through

        self.timer = self.create_timer(timer_period, self.coordinator_callback)
        self.prev_time = datetime.now()
        time = self.get_clock().now().to_msg()
        
        self.goal_poses = []

        # TBD: pass initial coord from actual robot (by subscribing once to its pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 1.66
        goal_pose.pose.position.y = -1.19
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.71
        goal_pose.pose.orientation.w = -0.7
        self.goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 6.76
        goal_pose.pose.position.y = -1.14
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.99
        goal_pose.pose.orientation.w = 0.02
        self.goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 6.5
        goal_pose.pose.position.y = 2.85
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.7
        goal_pose.pose.orientation.w = 0.7
        self.goal_poses.append(goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = 2.26
        goal_pose.pose.position.y = 2.9
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = -0.09
        goal_pose.pose.orientation.w = -0.99
        self.goal_poses.append(goal_pose)



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

        robot1.setChargerPos(goal_charger)
        robot2.setChargerPos(goal_charger)


    # ---

    # def printOnce(self, str):
    #     if(self.printedOnce != str):
    #         print(str)
    #     self.printedOnce = str

    # ---

    def coordinator_callback(self):
        now = datetime.now()
        delta_ms = (now - self.prev_time).seconds * 1000 + (now - self.prev_time).microseconds / 1000.

        if delta_ms >= 2000:
        #   print("### coordinator_callback")
            if(self.arrRobots[0]['robot'].navStatus == "getting_next_task" 
                and self.arrRobots[1]['robot'].navStatus == "getting_next_task"):
                for robot in self.arrRobots:
                    if(robot['waypoint_idx'] == len(self.goal_poses) - 1):
                        robot['waypoint_idx'] = 0
                    else:
                        robot['waypoint_idx'] += 1

                    target_pose = self.goal_poses[robot['waypoint_idx']]
                    
                    robot['robot'].setGoalPoses([target_pose])
                    robot['robot'].navStatus = 'ready'

    # ---        
       
def main(args=None):

    try:  
        rclpy.init(args=args)

        executor = SingleThreadedExecutor()
        coordinator = ChargerNavigatorMulti()
        for robot in coordinator.arrRobots:
            executor.add_node(robot['robot'])
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
                robot['robot'].destroy_node()
            coordinator.destroy_node()
 
    finally:
        # Shutdown
        rclpy.shutdown()    
 
 
if __name__ == '__main__':
  main()
