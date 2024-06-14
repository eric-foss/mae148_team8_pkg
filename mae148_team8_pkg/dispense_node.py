import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class DispenseNode(Node):

    true = Bool()
    true.data = True
    false = Bool()
    false.data = False

    def __init__(self):
        super().__init__('dispense_node')
        self.get_logger().info('Dispense Node Initialized')

        #Activation Subscription from Pathing Node
        self.activation_subscription_ = self.create_subscription(
            Int32,
            'pathing_complete',
            self.activation_callback,
            10)

        self.servo1_publisher_ = self.create_publisher(Bool, 'servo1', 10)
        self.servo1_publisher_.publish(self.false)

        self.servo2_publisher_ = self.create_publisher(Bool, 'servo2', 10)
        self.servo2_publisher_.publish(self.false)


    def activation_callback(self, msg):
        self.get_logger().info(f"Activation callback received mode: {msg}")
        if msg.data == 1:
            self.servo1_publisher_.publish(self.true)
            self.servo2_publisher_.publish(self.false)
        elif msg.data == 2:
            self.servo1_publisher_.publish(self.false)
            self.servo2_publisher_.publish(self.true)
        elif msg.data == 3:
            self.servo1_publisher_.publish(self.true)
            self.servo2_publisher_.publish(self.true)
        else:
            self.servo1_publisher_.publish(self.false)
            self.servo2_publisher_.publish(self.false)

def main(args=None):
    rclpy.init(args=args)
    node = DispenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


