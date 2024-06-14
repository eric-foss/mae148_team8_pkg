import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class DispenseNode(Node):

    mode = 0

    def __init__(self):
        super().__init__('dispense_node')
        self.get_logger().info('Dispense Node Initialized')

        #Activtaion Subscription from Pathing Node
        self.activation_subscription_ = self.create_subscription(
            Int32,
            'pathing_complete',
            self.activation_callback,
            10)
        
        self.servo1_publisher_ = self.create_publisher(Bool, 'servo1', 10)
        self.servo2_publihser_ = self.create_publisher(Bool, 'servo2', 10)        
        
        self.servo_timer_ = self.create_timer(1, self.servo_timer_callback)

    def activation_callback(self, msg):
        self.mode = msg.data

    def servo_timer_callback(self):
        if self.mode == 1 or self.mode == 3:
            self.servo1_publisher_.publish(True)
        if self.mode == 2 or self.mode == 3:
            self.servo2_publisher_.publish(True)
    

def main(args=None):
    rclpy.init(args=args)
    node = DispenseNode
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


