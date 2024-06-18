import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
#add the path to the reference scripts
#import sys, os
#sys.path.append(os.path.join(os.path.dirname(sys.path[0]), 'scripts'))
#sys.path.append(os.path.join(sys.path[0], 'scripts'))
from .workspace3 import Workspace
from .coord_handling import initialize_wksp_obs,LonLat_To_XY,XY_To_LonLat
import shapely
from .transform import PIDController
from .path import CTE
from .steering_guidance import getzangrot
#random comment

class GPSNode(Node):

    servo_command = Int32()
    
    servo_message = ''
    startmov_bool=False
    stop_drop_roll_bool=False
    speed = Twist()
    counter=0

    
    if servo_command.data == 0:
        servo_message + 'Standing by at Destination'
    elif servo_command.data == 1:
        servo_message + 'Delivering Left Package'
    elif servo_command.data == 2:
        servo_message + 'Delivering Right Package'
    else:
        servo_message + 'Delivering Both Packages'

    master = initialize_wksp_obs('test3.txt')
    workspace = master['wksp']
    origin = master['origin']
    obs_dict = master['obs_dict']
    
    
    def __init__(self):
        super().__init__('gps_node')
        self.get_logger().info('Initialized Node')
        P=0.075
        I=0.01
        D=0.15 
        self.initial_bool=True
        self.PID=PIDController(P,I,D)
        look_ahead=1
        look_behind=0
        self.CTE=CTE(look_ahead,look_behind)

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
        self.servo1_publisher_ = self.create_publisher(Bool,'servo1',10)
        self.get_logger().info(self.servo_message)

        self.servo2_publisher_ = self.create_publisher(Bool,'servo2',10)
        self.get_logger().info(self.servo_message)
    
    def hitch_callback(self,msg):
        self.get_logger().info("Successful latch. Begin movement.")
        self.startmov_bool=True


    def gps_callback(self, msg):
        #location list:
        #origin:(477974.31,3638149.78)
        #eb1:(0,0)
        #geisel steps:(477933.58,3638132.46)
        #geisel steps
        #
        origin=(477974.31,3638149.78)
        goal_loc=shapely.Point(0,0)
        if self.startmov_bool and not self.stop_drop_roll_bool:
            self.counter+=1
            lat = msg.latitude
            long = msg.longitude
            alt=msg.altitude
            #self.get_logger().info(f'Latitude: {lat:.2f}, Longitude: {long:.2f}, Altitude: {alt:.2f}')
            currPoi=LonLat_To_XY(long,lat)
            currPoi=(currPoi[0]-origin[0],currPoi[1]-origin[1])
            currPoint=shapely.Point(currPoi[0],currPoi[1])
            self.get_logger().info(str(self.initial_bool))
            if self.initial_bool:
                self.initial_bool=False
                start_loc=currPoint
                self.get_logger().info('start loc: '+str(currPoi[0])+','+str(currPoi[1]))
                our_ws=self.getPath(start_loc,goal_loc)
                path=our_ws.path_coords
            
            zang=getzangrot(path,currPoi,self.PID,self.CTE)
            if our_ws.isNearGoal(currPoint):
                self.speed.linear.x=0.0
                self.speed.angular.z=0.0
                self.stop_drop_roll_bool=True
                self.servo1_publisher_.publish(True)
                self.servo2_publisher_.publish(True)
            else:
                self.speed.linear.x=1.0
                self.speed.angular.z=zang
            self.vesc_publisher_.publish(self.speed)

    def getPath(self,start_location,goal_location):
        ws = Workspace(self.workspace,self.obs_dict,start_location,goal_location,boundary_type = 1)
        return ws
    


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
