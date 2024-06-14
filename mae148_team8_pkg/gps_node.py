import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random
from workspace3 import Workspace
from ROS2.coord_handling import initialize_wksp_obs,LonLat_To_XY,XY_To_LonLat
import shapely


class GPSNode(Node):

    servo_command = Int32()
    servo_command.data = random.randint(0,3)
    servo_message = ''
    
    if servo_command.data == 0:
        servo_message + 'Standing by at Destination'
    elif servo_command.data == 1:
        servo_message + 'Delivering Left Package'
    elif servo_command.data == 2:
        servo_message + 'Delivering Right Package'
    else:
        servo_message + 'Delivering Both Packages'
    
    
    def __init__(self):
        super().__init__('gps_node')
        self.get_logger().info('Initialized Node')

        #Hitch Subscriber
        self.hitch_subscription_ = self.create_subscription(
            Bool,
            'successful_latch',
            self.hitch_callback,
            10
        )

        #GPS Subscriber
        self.gps_subscription_ = self.create_subscription(
            NavSatFix,
            'fix',
            self.gps_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        
        #VESC Publisher
        self.vesc_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        #Success Publisher
        self.servo_publisher_ = self.create_publisher(Int32,'pathing_complete',10)
        self.get_logger().info(self.servo_message)

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude
        self.get_logger().info(f'Latitude: {self.lat:.2f}, Longitude: {self.long:.2f}, Altitude: {self.alt:.2f}')
    

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
