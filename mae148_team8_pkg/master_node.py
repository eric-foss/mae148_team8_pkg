import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile


class MasterNode(Node):

    motor_on = Bool()
    motor_on.data = True


    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('Initialized Node')


    	#Motor Publishers/Subscribers
        self.motor_publisher_ = self.create_publisher(Bool, 'motor_status', 10)
        self.get_logger().info('Sending Motor Rotate Request')
        self.motor_publisher_.publish(self.motor_on)
        self.cart_subscription_ = self.create_subscription(
            Bool,
            'cart_status',
            self.cart_callback,
            10)

        #GPS Subscriber
        self.gps_subscription_ = self.create_subscription(
            NavSatFix,
            'fix',
            self.gps_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        #VESC Publisher
        self.vesc_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


    def cart_callback(self, msg):
        
        if msg.data:
            self.get_logger().info('Sending Motor Rotate Request')
            self.motor_publisher_.publish(self.motor_on)
        else:
            self.motor_on.data = False

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude
        self.get_logger().info(f'Latitude: {self.lat:.2f}, Longitude: {self.long:.2f}, Altitude: {self.alt:.2f}')



def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
