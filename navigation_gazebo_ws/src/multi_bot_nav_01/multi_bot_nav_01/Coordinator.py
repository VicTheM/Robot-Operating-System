# (c) robotics.snowcron.com
# Use: MIT license

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import Pose, Twist

from multi_bot_nav_01.GlobalPlannerStraightLine import GlobalPlannerStraightLine
from multi_bot_nav_01.PathFollower import PathFollower

class Coordinator(Node):
    def __init__(self, planner, path_follower, robot_name = ""):
        print("*** Coordinator.init()... ", end = '')

        # ---

        # TBD: this should be moved to common code area
        if(robot_name != ""):
            super().__init__(robot_name + '_Coordinator')
            self.strSlashRobotName = "/" + robot_name
            self.strRobotNameSlash = robot_name + "/"
        else:
            super().__init__('Coordinator')
            self.strSlashRobotName = ""
            self.strRobotNameSlash = ""

        self.strRobotName = robot_name

        # --- 

        # self.get_straight_line_path_planner_client = self.create_client(GetPlan, 
        #     self.strRobotNameSlash + '/plan')

        # ---

        self.planner = planner
        self.path_follower = path_follower

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.coordinator_callback)

        self.plan = None

        self.status = "not_started"

        print("*** Coordinator.init(): done")


    def coordinator_callback(self):
        # print("*** Coordinator.coordinator_callback()")

        if(self.status == "not_started"):
            self.status = "started"
        
            print("*** Coordinator.coordinator_callback(); calling goToPose()")

            # --- TBD: Important. In future, move this code elsewhere, so that
            # waypoints can be passed from outside. This is just a demo.
                
            # goal = PoseStamped()
            # goal.pose.position.x = 4.0
            # goal.pose.position.y = 4.0            
            #self.goToPose(goal)

            arrWaypoints = []
            
            if(self.strRobotName == "" or self.strRobotName == "robot1"):
                wp_1 = PoseStamped()
                wp_1.pose.position.x = 4.0
                wp_1.pose.position.y = 0.0 
                arrWaypoints.append(wp_1)
                
                wp_2 = PoseStamped()
                wp_2.pose.position.x = 4.0
                wp_2.pose.position.y = 7.0 
                arrWaypoints.append(wp_2)

                wp_3 = PoseStamped()
                wp_3.pose.position.x = 6.0
                wp_3.pose.position.y = 1.0 
                arrWaypoints.append(wp_3)

                wp_4 = PoseStamped()
                wp_4.pose.position.x = 0.0
                wp_4.pose.position.y = 0.0                 
                arrWaypoints.append(wp_4)
            else:
                wp_1 = PoseStamped()
                wp_1.pose.position.x = -4.0
                wp_1.pose.position.y = 0.0 
                arrWaypoints.append(wp_1)
                
                wp_2 = PoseStamped()
                wp_2.pose.position.x = -4.0
                wp_2.pose.position.y = 2.0 
                arrWaypoints.append(wp_2)

                wp_3 = PoseStamped()
                wp_3.pose.position.x = -2.0
                wp_3.pose.position.y = 2.0 
                arrWaypoints.append(wp_3)

            self.goThroughPoses(arrWaypoints)

        elif(self.status == "started"):
            # TBD: get status from path_follower
            # self.status = "done"

            #print("*** Coordinator.coordinator_callback(); status == 'started'")
            pass

    def goToPose(self, goal):
        print("*** Coordinator.GoToPose()")

        print("*** Initial pose received: ", self.path_follower.initial_pose_received)
        print("current pose: ", self.path_follower.current_pose)

        start = PoseStamped()
        start.pose = self.path_follower.current_pose
                
        path = self.planner.getStraightLinePath(start, goal)
        # TBD: for advanced path planner, check if the goal was rejected:
        # if not self.goal_handle.accepted:
        #     print('Goal to ' + str(pose.pose.position.x) + ' ' +
        #                    str(pose.pose.position.y) + ' was rejected!')
        #     return False

        if(path is not None):
            print("*** Coordinator.GoToPose(); publishing path")
            self.path_follower.path_follower_path_publisher.publish(path)
            self.path_follower.command("start")

    # ------
    # The difference between goThroughPoses and goThroughWaypoints is: when
    # going throudh waypoints, we have to arrive EXACTLY at each waypoint, while 
    # in case of goThroughPoses, we can slightly miss.
    def goThroughPoses(self, arrWaypoints):
        print("*** Coordinator.goThroughPoses()")

        print("current pose: ", self.path_follower.current_pose)

        start = PoseStamped()
        start.pose = self.path_follower.current_pose

        path = Path()
        path.header.frame_id = self.strRobotNameSlash + 'map'

        start_pt = start

        for goal in arrWaypoints:                
            p = self.planner.getStraightLinePath(start_pt, goal)
            if(p is not None):
                for pt in p.poses[:-1]:
                    path.poses.append(pt)

                start_pt = p.poses[-1]               # Otherwise last and 1st points are the same
            # TBD: handle error "path is None"
        
        print("*** Coordinator.goThroughPoses(); publishing path")
        self.path_follower.path_follower_path_publisher.publish(path)
        self.path_follower.command("start")

        # for pt in path.poses:
        #     print("%.2f, %.2f" % (pt.pose.position.x, pt.pose.position.y))

    # ------

def main(args=None):
    print("*** Coordinator - main()")

    try:  
        arrRobotNames = [""]
        # arrRobotNames = ["robot1", "robot2"]

        rclpy.init(args=args)
        executor = SingleThreadedExecutor()

        for robot_name in arrRobotNames:
            planner = GlobalPlannerStraightLine(robot_name)
            path_follower = PathFollower(robot_name)
            coordinator = Coordinator(planner, path_follower, robot_name)
        
            executor.add_node(planner)
            executor.add_node(path_follower)
            executor.add_node(coordinator)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            #planner.destroy_node()
            #path_follower.destroy_node()
        
 
    finally:
        # Shutdown
        rclpy.shutdown() 

# ---

if __name__ == '__main__':
    main()