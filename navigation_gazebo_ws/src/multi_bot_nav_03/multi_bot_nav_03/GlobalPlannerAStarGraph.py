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
#from multi_bot_nav_03.GlobalPlannerStraightLine import GlobalPlannerStraightLine
from multi_bot_nav_03.a_star_graph import AStarGraph

import json

class GlobalPlannerAStarGraph(Node, AStarGraph):

    def __init__(self, robot_name):

        # Part of initialization is done in parent
        #super().__init__()

        print("*** GlobalPlannerAStarGraph.init()... ")

        # ---

        self.strMode = "a_star_graph"

        self.dSegmentLen = 0.1

        # TBD: this should be moved to common code area
        if(robot_name != ""):
            super().__init__(robot_name + '_GlobalPlannerStraightLine')
            self.strSlashRobotName = "/" + robot_name
            self.strRobotNameSlash = robot_name + "/"
        else:
            super().__init__('GlobalPlannerStraightLine')
            self.strSlashRobotName = ""
            self.strRobotNameSlash = ""

        self.strRobotName = robot_name

        # ---        

        self.plan_publisher = self.create_publisher(Path, self.strSlashRobotName + "/plan/a_star_graph", 1)

        self.make_plan_service = self.create_service(GetPlan, self.strSlashRobotName + "/plan/a_star_graph", 
            self.make_plan_callback)
    
        self.get_plan_client = self.create_client(GetPlan, self.strSlashRobotName + "/plan/a_star_graph")
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            print("\t*** Service not available, waiting...")

        # ---

        self.nodes = None 
        self.adj_list = None

        print("*** GlobalPlannerAStarGraph.init() done")

    # ---

    def make_plan_callback(self, request, response):
    
        print("*** GlobalPlannerAStarGraph.make_plan_callback()", flush=True)

        #start = request.start
        goal = request.goal

        # Indexes of start and finish nodes
        nStartIdx = int(goal.pose.position.x)
        nFinishIdx = int(goal.pose.position.y)

        # TBD: what treshold do we really need here?

        print(">>> graph make_plan_callback:", request.start.pose.position.x, request.start.pose.position.y, 
            goal.pose.position.x, goal.pose.position.y)
        nodes_path = self.get_astar_graph_path(nStartIdx, nFinishIdx, treshold=63)

        if(nodes_path is None):
            print("Path not found")
            return None
        else:
            print("*** Nodes in path:", nodes_path)

            path = Path()
            path.header.frame_id = self.strRobotNameSlash + 'map'
            
            start_pose = request.start

            # Copy first point to a path
            path.poses.append(start_pose)            
            
            for nNodeIdx in nodes_path:
                start_pose = path.poses[-1]
                # print("start_pose:", start_pose)
                # print("nodes[", nNodeIdx, "]:", self.nodes[nNodeIdx])

                # Calculate delta x and y
                dx = self.nodes[nNodeIdx][0] - start_pose.pose.position.x
                dy = self.nodes[nNodeIdx][1] - start_pose.pose.position.y

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

                    # Calculate quaternion based on segment direction
                    alpha = math.atan2(pose.pose.position.y - pose_prev.pose.position.y, 
                        pose.pose.position.x - pose_prev.pose.position.x)

                    quat = self.quaternion_from_euler(0, 0, alpha)

                    # print(f"\t*** x: {pose.pose.position.x}; y: {pose.pose.position.y}; \
                    #     alpha: {math.degrees(alpha)}")                 

                    # Set orientation
                    pose.pose.orientation = quat
                    # ---
                    
                    path.poses.append(pose)

        # pose = PoseStamped()
        # pose.pose.position.x = goal.pose.position.x
        # pose.pose.position.y = goal.pose.position.y
        # path.poses.append(pose)

        # print("*** Received straight line path: ", flush=True)
        # for pose in path.poses:
        #     print(f"\t*** x: {pose.pose.position.x}  y: {pose.pose.position.y}")                 

        self.plan_publisher.publish(path) 

        response.plan = path
        return response

    # ---

    def quaternion_from_euler(self, roll, pitch, yaw):

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
            - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) \
            + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) \
            - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) \
            + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw

        return quat

    # ---            

    def getPath(self, start, goal):
        print("*** GlobalPlannerAStarGraph.getPath()")
        #print("*** ", self.__class__.__name__, ".getPath()")

        print("*** Waiting for 'GlobalPlannerAStarGraph' service")
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            print("*** 'GlobalPlannerAStarGraph' service not available, waiting...", flush = True)

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

    # ---

    # Alternative to copying dictionary in order to have int keys
    # def convert_keys_to_int(ordered_pairs):
    #     result = {}
    #     for key, value in ordered_pairs:
    #          result[int(key)] = value
    #     return result
    # data_int_keys = json.loads(data, object_pairs_hook=convert_keys_to_int)
    # print(data_int_keys)    

    # Format: 
    # nodes = {
    #     0: [199, 69],
    #     1: [428, 159],
    #     2: [421, 222],
    #     3: [234, 218],
    #     4: [288, 269],
    # }

    # adj_list = {
    #     0: [ [1, 128], ],
    #     1: [ [2, 128], ],
    #     2: [ ],
    #     3: [ [4, 128], ],
    #     4: [ ],
    # }

    def loadGraph(self, strFilePath):
        #print(os.getcwd())
        with open(strFilePath) as f:
            data = f.read()

            #print(data)

            nodes_str, adj_list_str = data.split('adj_list = ')
            nodes_str = nodes_str.split("nodes = ")[1]

            # print("***")
            # print(nodes_str)
            # print("***")
            # print(adj_list_str)

            nodes_str = json.loads(nodes_str)  
            adj_list_str = json.loads(adj_list_str)

            nodes = {}
            for k,v in nodes_str.items():
                nodes[int(k)] = v

            adj_list = {}
            for k,v in adj_list_str.items():
                adj_list[int(k)] = v

            # print(nodes)
            # print("***")
            # print(adj_list)

            self.nodes = nodes
            self.adj_list = adj_list

            return nodes, adj_list

    # ---

def main(args=None):
    print("*** GlobalPlannerAStarGraph - main()", flush=True)

    rclpy.init(args=args)
    planner = GlobalPlannerAStarGraph("")

    # ---

    # Note / TBD: if we want to be able to change roads graph dynamically, we need to 
    # publish it instead of assigning here, same way it is done for base map image
    # in publish_image() 
    strRoadsGraphFileName = "../../maps/nav_graph_world_01.json"
    nodes, adj_list = planner.loadGraph(strRoadsGraphFileName)

    # ---    
    # To minimize changes, let's pass indexes of start and finish nodes
    # (0 and 42) in pose coordinates. In case of nav. graph, we do not need
    # the pose, as we extract coordinates from nodes list.
    start = PoseStamped()
    start.pose.position.x = 3.55
    start.pose.position.y = 1.15

    goal = PoseStamped()
    goal.pose.position.x = 22.
    goal.pose.position.y = 3.
    
    planner.getPath(start, goal)

    planner.destroy_node()
    rclpy.shutdown()

# ---

if __name__ == '__main__':
    main()

# ---
