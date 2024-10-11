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

from nav25d_04.GlobalPlannerStraightLine import GlobalPlannerStraightLine
from nav25d_04.GlobalPlannerAStarGraph import GlobalPlannerAStarGraph
from nav25d_04.GlobalPlannerAStarGrid import GlobalPlannerAStarGrid

from nav25d_04.PathFollower import PathFollower
from nav25d_04.ImgLayersManager import ImgLayersManager
from gazebo_msgs.srv import SpawnEntity

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import LaserScan

class Coordinator(Node):
    def __init__(self, grid_planner, graph_planner, path_follower, robot_name):
        print("*** Coordinator.init()... ", end = '')

        # ---

        self.nPathColorId = 0

        # Waypoints to go through for grid-based algos
        self.arrWaypoints = []

        # Indices of waypoint nodes for nav. graph based algos
        self.arrWaypointNodes = []

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

        self.grid_planner = grid_planner
        self.graph_planner = graph_planner

        self.strPlannerName = ""

        self.path_follower = path_follower

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.coordinator_callback)

        self.status = "not_started"

        print("*** Coordinator.init(): done")

    # ------

    # Note: used on start, only once, so I do not keep publisher as a class member
    def publish_image(self, strTopic):
        # TBD: put it in "maps"
        img = cv2.imread('world_01.png')
        if img is None:
            print('Could not read image from disk')
            return
        img = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(img, "bgra8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = strTopic
        
        publisher_ = self.create_publisher(Image, strTopic, 10) 
        publisher_.publish(msg)              


    def coordinator_callback(self):
        # print("*** Coordinator.coordinator_callback()")

        if(self.status == "not_started" and self.path_follower.initial_pose_received == True):
            print("*** Coordinator.coordinator_callback(); calling goToPose()/goToNode()")

            # ---

            self.publish_image("/images/background")

            # ---

            # --- TBD: Important. In future, move this code elsewhere, so that
            # waypoints can be passed from outside. This is just a demo.

            if(self.grid_planner is not None):
                print("Grid planner mode:", self.grid_planner.strMode)
            if(self.graph_planner is not None):
                print("Graph planner mode:", self.graph_planner.strMode)                

            if(self.grid_planner is not None and self.strPlannerName == "straight_line_planner"):
                self.goThroughPoses()
            # #     goal = PoseStamped()
            # #     goal.pose.position.x = 4.0
            # #     goal.pose.position.y = 4.0    
            # #     self.goToPose(goal) 

            elif(self.grid_planner is not None and self.strPlannerName == "a_star_grid_planner"):

                self.goThroughPoses()
            # #     goal = PoseStamped()
            # #     goal.pose.position.x = -5.22
            # #     goal.pose.position.y = -4.75          
            # #     self.goToPose(goal)

            elif(self.graph_planner is not None and self.strPlannerName == "a_star_graph_planner"):
                # self.goToNode(None, 10, True)

                self.goThroughNodes()

            self.status = "started"

        elif(self.status == "started"):
            # TBD: get status from path_follower
            # self.status = "done"

            #print("*** Coordinator.coordinator_callback(); status == 'started'")
            pass

    # ---

    # TBD
    def getClosestNode(self):
        current_pose = self.path_follower.current_pose
        
        dist = float("inf")
        nIdxOfMin = 0
        for i in range(len(self.graph_planner.nodes)):
            pose = Pose()
            pose.position.x = self.graph_planner.nodes[i][0]
            pose.position.y = self.graph_planner.nodes[i][1]

            d = self.path_follower.getDistanceBetweenPoses(current_pose, pose)
            #print(">>> Node:", node)
            if(d < dist):
                #print("This is min node:", node)
                nIdxOfMin = i
                dist = d

        return nIdxOfMin

    # ---

    def goToNode(self, nStartNodeIdx, nGoalNodeIdx, bStart):
        print("*** Coordinator.GoToNode()")

        print("*** Initial pose received: ", self.path_follower.initial_pose_received)
        #print("current pose: ", self.path_follower.current_pose)

        start_pose = PoseStamped()
        goal_pose = PoseStamped()

        # The very first node in a list
        if(nStartNodeIdx is None):
            nStartNodeIdx = self.getClosestNode()
            print(">>> Closest node:", nStartNodeIdx)

            # Note that in start_pose we pass current pose, 
            # while in end pose we pack start and goal nodes
            start_pose.pose = self.path_follower.current_pose

            # graph_planner will use straight line to get to closest node
            if(self.grid_planner is None):  
                goal_pose.pose.position.x = float(nStartNodeIdx)
                goal_pose.pose.position.y = float(nGoalNodeIdx)

                print("graph_planner-1")
                path = self.graph_planner.getPath(start_pose, goal_pose)
            # Use grid_planner to get to closest node
            else:
                goal_pose.pose.position.x = self.graph_planner.nodes[nGoalNodeIdx][0]
                goal_pose.pose.position.y = self.graph_planner.nodes[nGoalNodeIdx][1]

                print("grid_planner-1: ", goal_pose.pose.position.x, goal_pose.pose.position.y)
                path = self.grid_planner.getPath(start_pose, goal_pose)

        # Not first waypoint, use graph planner only
        else:
            start_pose.pose.position.x = self.graph_planner.nodes[nStartNodeIdx][0]
            start_pose.pose.position.y = self.graph_planner.nodes[nStartNodeIdx][1]

            goal_pose.pose.position.x = float(nStartNodeIdx)
            goal_pose.pose.position.y = float(nGoalNodeIdx)

            print("graph_planner-2", start_pose.pose, goal_pose.pose)
            path = self.graph_planner.getPath(start_pose, goal_pose)

        #print("goToNode, self.nPathColorId: ", self.nPathColorId)
        for node in path.poses:
            self.path_follower.arrColorIds.append(self.nPathColorId)
        self.nPathColorId = 0 if self.nPathColorId == 1 else 1
        
        if(bStart):
            if(path is not None):
                print("*** Coordinator.GoToPose(); publishing path")
                self.path_follower.path_follower_path_publisher.publish(path)
                self.path_follower.command("start")
        
        return path

    # ---

    def goThroughNodes(self):

        print("*** Coordinator.goThroughNodes()")
        #print("current pose: ", self.path_follower.current_pose)

        path = self.goToNode(None, self.arrWaypointNodes[0], False)

        if(path is not None):
            local_path = None
            for i in range(1, len(self.arrWaypointNodes)):
                local_path = self.goToNode(self.arrWaypointNodes[i - 1], self.arrWaypointNodes[i], False)
                # print("Color Id:", self.nPathColorId)

                if(local_path is not None):
                    for pt in local_path.poses[:-1]:
                        path.poses.append(pt)
                else:
                    print(">>> Unable to build path from", self.arrWaypointNodes[i-1],
                        "to", self.arrWaypointNodes[i])
                    return

        else:
            print(">>> Unable to build path from current pose to", self.arrWaypointNodes[0])
            return

        print("*** Coordinator.goThroughNodes(); publishing path")
        self.path_follower.path_follower_path_publisher.publish(path)
        self.path_follower.command("start")

    # ---

    def goToPose(self, goal):
        print("*** Coordinator.GoToPose()")

        print("*** Initial pose received: ", self.path_follower.initial_pose_received)
        #print("current pose: ", self.path_follower.current_pose)

        start = PoseStamped()
        start.pose = self.path_follower.current_pose
                
        print("graid_planner-2")
        path = self.grid_planner.getPath(start, goal)
        # TBD: for advanced path planner, check if the goal was rejected:
        # if not self.goal_handle.accepted:
        #     print('Goal to ' + str(pose.pose.position.x) + ' ' +
        #                    str(pose.pose.position.y) + ' was rejected!')
        #     return False

        print("goToPose, self.nPathColorId: ", self.nPathColorId)
        for node in path.poses:
            self.path_follower.arrColorIds.append(self.nPathColorId)
        self.nPathColorId = 0 if self.nPathColorId == 1 else 1

        if(path is not None):
            print("*** Coordinator.GoToPose(); publishing path")
            self.path_follower.path_follower_path_publisher.publish(path)
            self.path_follower.command("start")

    # ---

    # The difference between goThroughPoses and goThroughWaypoints is: when
    # going throudh waypoints, we have to arrive EXACTLY at each waypoint, while 
    # in case of goThroughPoses, we can slightly miss.
    def goThroughPoses(self):
        print("*** Coordinator.goThroughPoses()")

        #print("current pose: ", self.path_follower.current_pose)

        # Important: in case of nav. graph algorithms, initial pose
        # should be on start node.
        # TBD: If not, need to add "grid based navigate to closest node"
        start = PoseStamped()
        start.pose = self.path_follower.current_pose

        path = Path()
        path.header.frame_id = self.strRobotNameSlash + 'map'

        start_pt = start

        for goal in self.arrWaypoints:
            print("graid_planner-3")
            p = self.grid_planner.getPath(start_pt, goal)
            if(p is not None):
                print("goThroughPoses, self.nPathColorId: ", self.nPathColorId)
                for pt in p.poses[:-1]:
                    path.poses.append(pt)
                    self.path_follower.arrColorIds.append(self.nPathColorId)
                
                self.nPathColorId = 0 if self.nPathColorId == 1 else 1

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
        rclpy.init(args=args)
        
        arrRobotNames = [""]
        #arrRobotNames = ["robot1", "robot2"]

        # strPlannerName = "straight_line_planner"
        # strPlannerName = "a_star_grid_planner"
        strPlannerName = "a_star_graph_planner"

        # ---

        node_ground = rclpy.create_node('spawn_model')
        client = node_ground.create_client(SpawnEntity, '/spawn_entity')
        request = SpawnEntity.Request()
        request.name = "my_model"

        # TBD. For now: cube.urdf is a very flat cube (plane) to display map image on.
        # Its size should be the same as size of a map image (again: make adjustable)
        # For example, for image 640x640 and resolution: 0.05, we need 
        # 640*0.05 = 32m surface.
        request.xml = open("cube.urdf").read()
        request.robot_namespace = ""

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node_ground, future)
                    
        # ---            
        
        executor = SingleThreadedExecutor()

        img_layer_manager = ImgLayersManager([], "/images/ground_plane", 640, 640, 4, 5)
        executor.add_node(img_layer_manager)

        # Add the background map
        strTopic = '/images/background'
        img_layer_manager.addTopic(strTopic)

        for robot_name in arrRobotNames:
            grid_planner = None
            graph_planner = None

            arrWaypoints = []
            arrWaypointNodes = []

            if(strPlannerName == "straight_line_planner"):
                grid_planner = GlobalPlannerStraightLine(robot_name)

                if(robot_name == "" or robot_name == "robot1"):
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

            elif(strPlannerName == "a_star_grid_planner"):
                grid_planner = GlobalPlannerAStarGrid(robot_name)
                grid_planner.initMap("../../maps/keepout_world_01.yaml", 64, 64)

                if(robot_name == "" or robot_name == "robot1"):
                    # wp_1 = PoseStamped()
                    # wp_1.pose.position.x = 3.55
                    # wp_1.pose.position.y = 1.15
                    # arrWaypoints.append(wp_1)
                    
                    wp_1 = PoseStamped()
                    wp_1.pose.position.x = -5.8
                    wp_1.pose.position.y = 5.5
                    arrWaypoints.append(wp_1)
                else:
                    wp_1 = PoseStamped()
                    wp_1.pose.position.x = -5.22
                    wp_1.pose.position.y = -4.75
                    arrWaypoints.append(wp_1)
                    
                    wp_2 = PoseStamped()
                    wp_2.pose.position.x = 6.05
                    wp_2.pose.position.y = -2.47
                    arrWaypoints.append(wp_2)

            elif(strPlannerName == "a_star_graph_planner"):
                graph_planner = GlobalPlannerAStarGraph(robot_name)

                # Note / TBD: if we want to be able to change roads graph dynamically, we need to 
                # publish it instead of assigning here, same way it is done for base map image
                # in publish_image() 
                strRoadsGraphFileName = "../../maps/nav_graph_world_01.json"
                graph_planner.loadGraph(strRoadsGraphFileName)

                # ---

                grid_planner = GlobalPlannerAStarGrid(robot_name)
                grid_planner.initMap("../../maps/keepout_world_01.yaml", 64, 64)

                # ---

                if(robot_name == "" or robot_name == "robot1"):
                    arrWaypointNodes.append(22)
                    arrWaypointNodes.append(3)
                    arrWaypointNodes.append(1)
                else:
                    arrWaypointNodes.append(35)
                    arrWaypointNodes.append(13)

            path_follower = PathFollower(robot_name)
            coordinator = Coordinator(grid_planner, graph_planner, path_follower, robot_name)
            
            coordinator.path_follower.arrColorIds = []
            coordinator.nPathColorId = 0            

            coordinator.arrWaypoints = arrWaypoints
            coordinator.arrWaypointNodes = arrWaypointNodes 

            coordinator.strPlannerName = strPlannerName

            if(grid_planner is not None):
                executor.add_node(grid_planner)
            if(graph_planner is not None):
                executor.add_node(graph_planner)

            executor.add_node(path_follower)
            executor.add_node(coordinator)

            strTopic = coordinator.strSlashRobotName + '/images/robot_path_markers'
            img_layer_manager.addTopic(strTopic)

        print("Ready to spin")
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