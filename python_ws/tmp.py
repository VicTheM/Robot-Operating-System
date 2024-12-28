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


"""
// Define pins for UART communication
const int rxPin = 10; // RX pin of Arduino connected to TX of sensor
const int txPin = 11; // TX pin of Arduino connected to RX of sensor

// Include SoftwareSerial library for UART communication
#include <SoftwareSerial.h>

// Create a SoftwareSerial object
SoftwareSerial mySerial(rxPin, txPin);

// Variable to store the distance
float distance;

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(9600);

  // Initialize the UART communication with the sensor
  mySerial.begin(9600);

  Serial.println("A02YYUW Sensor Initialized");
}

void loop() {
  // Request distance measurement from the sensor
  mySerial.write(0x55); // Command to request distance

  // Wait for the sensor to respond
  delay(100);

  // Check if data is available from the sensor
  if (mySerial.available() >= 4) {
    // Read the response (4 bytes expected)
    byte header = mySerial.read();
    byte highByte = mySerial.read();
    byte lowByte = mySerial.read();
    byte checksum = mySerial.read();

    // Validate the header and checksum
    if (header == 0xFF && checksum == (0xFF + highByte + lowByte) % 256) {
      // Combine high and low bytes to get distance in millimeters
      int distanceMM = (highByte << 8) + lowByte;
      distance = distanceMM / 10.0; // Convert to centimeters

      // Print the distance to the Serial Monitor
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    } else {
      Serial.println("Invalid data received");
    }
  } else {
    Serial.println("No data received");
  }

  // Add a delay before the next measurement
  delay(500);
}
"""
