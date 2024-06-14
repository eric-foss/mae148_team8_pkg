import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist


class ForwardNode(Node):

    go = Bool()
    go.data = False
    drop = Int32()
    drop.data = 3
    speed = Twist()
    speed.linear.x = 2.0
    counter = 0


    def __init__(self):
        super().__init__('forward_node')
        self.get_logger().info("Initialized Forward Node")

        self.endpoint_publisher_ = self.create_publisher(Int32, 'pathing_complete', 10)

        self.speed_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.activate_subscription_ = self.create_subscription(
            Bool,
            'successful_latch',
            self.activation_callback,
            10)
        

    def activation_callback(self, msg):
        if msg.data == True:
            self.go.data = True
            self.stop_timer_ = self.create_timer(0.1, self.stop_callback)
        
    def stop_callback(self):
        self.counter = self.counter + 1
        self.speed_publisher_.publish(self.speed)
        if self.counter > 100:
            self.endpoint_publisher_.publish(self.drop)
            self.go.data = False
            self.destroy_timer(self.stop_timer_)



def main(args=None):
    rclpy.init(args=args)
    node = ForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
