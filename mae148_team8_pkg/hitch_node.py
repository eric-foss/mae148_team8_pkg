import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile


class HitchNode(Node):

    motor_on = Bool()
    success_msg = Bool()
    motor_on.data = True
    success_msg.data = False


    def __init__(self):
        #Initialize Node
        super().__init__('hitch_node')
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

        #Successful Latch Publisher
        self.success_publisher_ = self.create_publisher(Bool, 'successful_latch', 10);
        self.success_publisher_.publish(self.success_msg)
        

    def cart_callback(self, msg):
        if msg.data == True:
            self.get_logger().info('Sending Motor Rotate Request')
            self.motor_publisher_.publish(self.motor_on)
        else: #LATCH
            self.motor_on.data = False
            self.success_msg.data = True
            self.success_publisher_.publish(self.success_msg)
            



def main(args=None):
    rclpy.init(args=args)
    node = HitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
