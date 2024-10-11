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

class GlobalPlannerStraightLine(Node):

    def __init__(self, robot_name = ""):

        print("*** GlobalPlannerStraightLine.init()... ")

        # ---

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
        
        self.plan_publisher = self.create_publisher(Path, self.strSlashRobotName + "/plan", 1)

        self.make_plan_service = self.create_service(GetPlan, self.strSlashRobotName + "/plan", 
            self.make_plan_callback)
    
        self.get_plan_client = self.create_client(GetPlan, self.strSlashRobotName + "/plan")
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            print("\t*** Service not available, waiting...")

        print("*** GlobalPlannerStraightLine.init() done")

    # ---

    def make_plan_callback(self, request, response):
    
        print("*** GlobalPlannerStraightLine.make_plan_callback()", flush=True)

        start = request.start
        goal = request.goal

        #print(start.pose.position.x, start.pose.position.y)
        #print(goal.pose.position.x, goal.pose.position.y)

        # Calculate delta x and y
        dx = goal.pose.position.x - start.pose.position.x
        dy = goal.pose.position.y - start.pose.position.y

        # Simple straight line path
        path = Path()
        path.header.frame_id = self.strRobotNameSlash + 'map'

        # Copy start orientation to first point
        path.poses.append(start)

        # Calculate distance between start and goal
        distance = math.sqrt(dx**2 + dy**2)

        #print("distance: ", distance, flush=True)

        # Number of points to include in path
        num_points = int(distance / self.dSegmentLen) # 0.1m resolution

        for i in range(1, num_points + 1):
            pose_prev = path.poses[-1]

            pose = PoseStamped()
            pose.pose.position.x = start.pose.position.x + i * dx / num_points
            pose.pose.position.y = start.pose.position.y + i * dy / num_points

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

    def getStraightLinePath(self, start, goal):
        print("*** GlobalPlannerStraightLine.getStraightLinePath()")

        print("*** Waiting for 'GlobalPlannerStraightLine' service")
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            print("*** 'GlobalPlannerStraightLine' service not available, waiting...", flush = True)

        print("*** Calculating path from (", start.pose.position.x, ',', start.pose.position.y, 
            ') to (', 
            goal.pose.position.x, ',', goal.pose.position.y, ")", flush = True)

        request = GetPlan.Request()
        request.start = start
        request.goal = goal

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

def main(args=None):
    print("*** GlobalPlannerStraightLine - main()", flush=True)

    rclpy.init(args=args)
    planner = GlobalPlannerStraightLine()

    start = PoseStamped()
    start.pose.position.x = -2.0
    start.pose.position.y = -2.0 

    goal = PoseStamped()
    goal.pose.position.x = 2.0
    goal.pose.position.y = 2.0 
    
    planner.getStraightLinePath(start, goal)

    # Alternative way of calling it:
    #  
    # request = GetPlan.Request()
    # request.start.pose.position.x = -1.0
    # request.start.pose.position.y = -1.0
    # request.goal.pose.position.x = 2.0
    # request.goal.pose.position.y = 2.0

    # future = planner.get_plan_client.call_async(request)

    # rclpy.spin_until_future_complete(planner, future)

    # if future.result() is not None:
    #     response = future.result()
    #     if response.plan:
    #         print("Received plan:", flush=True)
    #         for pose in response.plan.poses:
    #             print(f"x: {pose.pose.position.x}, y: {pose.pose.position.y}", flush=True)
    #     else:
    #         print("Received empty plan.", flush=True)
    # else:
    #     print("Service call failed.", flush=True)

    planner.destroy_node()
    rclpy.shutdown()

# ---

if __name__ == '__main__':
    main()

# ---
