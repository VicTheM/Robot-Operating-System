# (c) robotics.snowcron.com
# Use: MIT license

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped  
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Twist
import math
from geometry_msgs.msg import Quaternion
from nav25d_03.a_star import AStar

from PIL import Image
import math
import cv2
import numpy as np

class GlobalPlannerAStarGrid(Node, AStar):

    def __init__(self, robot_name):

        print("*** GlobalPlannerAStarGrid.init()... ")

        # ---

        AStar.__init__(self)
        # or 
        # super(AStar, self, arrKeepoutGrid).__init__() # Calls AStar.__init__()

        # ---

        self.strMode = "a_star_grid"

        self.dSegmentLen = 0.1

        # TBD: this should be moved to common code area
        if(robot_name != ""):
            Node.__init__(self, robot_name + '_GlobalPlannerAStarGrid')
            self.strSlashRobotName = "/" + robot_name
            self.strRobotNameSlash = robot_name + "/"
        else:
            Node.__init__(self, 'GlobalPlannerAStarGrid')
            self.strSlashRobotName = ""
            self.strRobotNameSlash = ""

        self.strRobotName = robot_name

        # ---

        self.strYamlImageFilePath = None
        self.nGridSizeX = None
        self.nGridSizeY = None
        self.arrGrid = None
        self.dictMapData = None

        # ---

        self.plan_publisher = self.create_publisher(Path, self.strSlashRobotName + "/plan/a_star_grid", 1)

        self.make_plan_service = self.create_service(GetPlan, self.strSlashRobotName + "/plan/a_star_grid", 
            self.make_plan_callback)
    
        self.get_plan_client = self.create_client(GetPlan, self.strSlashRobotName + "/plan/a_star_grid")
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            print("\t*** Service not available, waiting...")

        print("*** GlobalPlannerAStarGrid.init() done")

    # ---

    def poseToGrid(self, p):
        x = (int)((p.pose.position.x - self.dictMapData['originMetersX']) / self.dictMapData['gridCellSizeX'])
        #y = (int)((p.pose.position.y - self.dictMapData['originMetersY']) / self.dictMapData['gridCellSizeY'])
        nImgSizeY = self.dictMapData['img'].size[1]
        dOriginY = self.dictMapData['originMetersY']
        dResolution = self.dictMapData['resolution']
                
        #print("***", nImgSizeY, dResolution, p.pose.position.y, self.dictMapData['gridCellSizeY'])
        y = (int)((nImgSizeY * dResolution - (p.pose.position.y +  - self.dictMapData['originMetersY'])) / self.dictMapData['gridCellSizeY'])

        #print("pPoseToGrid: ", x, y)

        return (y, x)

    # ---

    def gridToPose(self, node):
        x = (node[1] + 0.5) * self.dictMapData['gridCellSizeX'] + self.dictMapData['originMetersX']
        y = (self.nGridSizeY - node[0]  + 0.5) * self.dictMapData['gridCellSizeY'] + self.dictMapData['originMetersY']

        #print("***", node, x, y)
        return (x, y)

    # ---

    def make_plan_callback(self, request, response):
    
        print("*** GlobalPlannerAStarGrid.make_plan_callback()", flush=True)

        start = request.start
        goal = request.goal

        print(">>> grid make_plan_callback:", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y)

        path = Path()
        path.header.frame_id = self.strRobotNameSlash + 'map'

        # TBD: 32 is hardcoded
        arrGridPath = self.get_astar_grid_path(self.poseToGrid(start), self.poseToGrid(goal), 32)
        if(arrGridPath is None):
            print(">>> Unable to find path")
        else:
            # for node in arrGridPath:
            #     print(node)
            
            # Note: at this point, path contains cells of a grid through which
            # the path goes: y, x = node[0], node[1]
            # We need to convert it to Poses        

            # TBD: nNegate currently not used

            start_pose = request.start

            # Copy first point to a path
            path.poses.append(start_pose)

            for node in arrGridPath:
                start_pose = path.poses[-1]

                # x = node[1] * self.dictMapData['gridCellSizeX'] + self.dictMapData['originMetersX']
                # y = node[0] * self.dictMapData['gridCellSizeY'] + self.dictMapData['originMetersY']
                x, y = self.gridToPose(node) 
                
                # --- This code will not break it to smaller segments, TBD
                # pose = PoseStamped()
                # pose.pose.position.x = x
                # pose.pose.position.y = y
                # path.poses.append(pose) 

                # --- This code will not break it to smaller segments: TBD?

                # Calculate delta x and y
                dx = x - start_pose.pose.position.x
                dy = y - start_pose.pose.position.y

                # print(node, x, y, dx, dy)

                # Simple straight line path
                    
                # Calculate distance between start and goal
                distance = math.sqrt(dx**2 + dy**2)

                # Number of points to include in path
                num_points = int(distance / self.dSegmentLen) # 0.1m resolution

                for i in range(1, num_points + 1):
                    pose_prev = path.poses[-1]

                    pose = PoseStamped()
                    pose.pose.position.x = start_pose.pose.position.x + i * dx / num_points
                    pose.pose.position.y = start_pose.pose.position.y + i * dy / num_points

                    # Currently not used
                    # # Calculate quaternion based on segment direction
                    # alpha = math.atan2(pose.pose.position.y - pose_prev.pose.position.y, 
                    #     pose.pose.position.x - pose_prev.pose.position.x)
                    # quat = self.quaternion_from_euler(0, 0, alpha)
                    # # print(f"\t*** x: {pose.pose.position.x}; y: {pose.pose.position.y}; \
                    # #     alpha: {math.degrees(alpha)}")                 

                    # # Set orientation
                    # pose.pose.orientation = quat
                    # ---
                    
                    path.poses.append(pose)            

            # print("*** Received straight line path: ", flush=True)
            # for pose in path.poses:
            #     print(f"\t*** x: {pose.pose.position.x}  y: {pose.pose.position.y}")                 

            self.plan_publisher.publish(path) 

        response.plan = path
        return response

    # ---

    def getPath(self, start, goal):
        print("*** GlobalPlannerAStarGrid.getPath()")
        #print("*** ", self.__class__.__name__, ".getPath()")

        print("*** Waiting for 'GlobalPlannerAStarGrid' service")
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            print("*** 'GlobalPlannerAStarGrid' service not available, waiting...", flush = True)

        print("*** Calculating path from (", start.pose.position.x, ',', start.pose.position.y, 
            ') to (', 
            goal.pose.position.x, ',', goal.pose.position.y, ")", flush = True)

        request = GetPlan.Request()
        request.start = start
        request.goal = goal
        # print(">>> *** request: ", start.pose.position.x, start.pose.position.y,
        #     goal.pose.position.x, goal.pose.position.y)        

        future = self.get_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            path = future.result().plan
  
            # print("*** Received straight line path: ")
            # i = 0
            # for pose in path.poses:
            #     print(f"\t*** {i}. x: {pose.pose.position.x}  y: {pose.pose.position.y}") 
            #     i += 1

            return path

        else:
            print('*** Failed to get plan')
            return None  

    # ------

    def initMap(self, strYamlImageFilePath, nGridSizeX, nGridSizeY):
        self.strYamlImageFilePath = strYamlImageFilePath
        self.nGridSizeX = nGridSizeX
        self.nGridSizeY = nGridSizeY

        self.dictMapData = self.loadKeepoutGrid(strYamlImageFilePath, nGridSizeX, nGridSizeY)
        self.strYamlImageFilePath = strYamlImageFilePath
        self.nGridSizeX = nGridSizeX
        self.nGridSizeY = nGridSizeY

        self.setMap(self.dictMapData["grid"])

        # for row in self.arrGrid:
        #     print(row)

    # ---

def main(args=None):
    print("*** GlobalPlannerAStarGrid - main()", flush=True)

    rclpy.init(args=args)
    planner = GlobalPlannerAStarGrid("")
    planner.initMap("../../maps/keepout_world_01.yaml", 64, 64)

    start = PoseStamped()
    start.pose.position.x = 5.93
    start.pose.position.y = 1.15

    goal = PoseStamped()
    goal.pose.position.x = 3.62
    goal.pose.position.y = 1.15

    planner.getPath(start, goal)

    planner.destroy_node()
    rclpy.shutdown()

# ---

if __name__ == '__main__':
    main()

# ---
