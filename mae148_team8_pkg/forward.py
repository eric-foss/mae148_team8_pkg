import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class ForwardNode(Node):

    go = Bool()
    go.data = False
    speed = Twist()
    speed.linear.x = 2.0


    def __init__(self):
        super().__init__('forward_node')
        self.get_logger().info("Initialized Forward Node")

        self.endpoint_publisher_ = self.create_publisher(Bool, 'pathing_complete', 10)

        self.speed_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.activate_subscription_ = self.create_subscription(
            Bool,
            'successful_latch',
            self.activation_callback,
            10)
        

    def activation_callback(self, msg):
        if msg.data == True:
            self.go.data = True
            self.speed_publisher_.publish(self.speed)
            self.stop_timer_ = self.create_timer(10, self.stop_callback)
        
    def stop_callback(self):
        self.speed.linear.x = 0.0
        self.speed_publisher_.publish(self.speed)
        self.endpoint_publisher_.publish(self.go.data)
        self.go.data = False
        self.destroy_timer(self.stop_timer__)



def main(args=None):
    rclpy.init(args=args)
    node = ForwardNode()
    rclpy.spin(Node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()