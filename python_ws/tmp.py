import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json

class InfoJsonNode(Node):
    def __init__(self):
        super().__init__('info_json_node')

        # Subscribers
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publisher
        self.json_publisher = self.create_publisher(String, '/info/json', 10)

        # Data storage
        self.gps_data = None
        self.odom_data = None
        self.imu_data = None

    def gps_callback(self, msg):
        self.gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        self.publish_json()

    def odom_callback(self, msg):
        self.odom_data = {
            'xpose': msg.pose.pose.position.x,
            'ypose': msg.pose.pose.position.y,
            'zpose': msg.pose.pose.position.z
        }
        self.publish_json()

    def imu_callback(self, msg):
        self.imu_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            }
        }
        self.publish_json()

    def publish_json(self):
        if self.gps_data and self.odom_data and self.imu_data:
            # Combine data into JSON format
            combined_data = {
                'gps': self.gps_data,
                'odom': self.odom_data,
                'imu': self.imu_data
            }
            json_string = json.dumps(combined_data)

            # Publish JSON string
            msg = String()
            msg.data = json_string
            self.json_publisher.publish(msg)
            self.get_logger().info(f'Published JSON: {json_string}')

def main(args=None):
    rclpy.init(args=args)
    node = InfoJsonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
