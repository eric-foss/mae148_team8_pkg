import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

class MasterNode(Node):

    motor_on = Bool()
    motor_on.data = True


    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('Initialized Node')


    	#Motor Publishers/Subscribers
        self.motor_publisher_ = self.create_publisher(Bool, 'motor_status', 10)
        self.motor_publisher_.publish(self.motor_on)
        self.completion_subscription_ = self.create_subscription(
            Empty,
            'operation_complete',
            self.completion_callback,
            10)

        #GPS Subscriber
        self.gps_subscription_ = self.create_subscription(
            NavSatFix,
            'fix',
            self.gps_callback,
            10)

        #VESC Publisher
        self.vesc_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)


    def completion_callback(self, msg):
        self.get_logger().info('Received completion signal')
        self.motor_on.data = False

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude
        self.get_logger().info('Test')
        self.get_logger().info('Latitude: %.2f, Longitude %.2f, Altitude, %.2f', self.lat, self.long, self.alt)




def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
