import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

#--------------------------------------------------------------------------------------------------------------------#

n = 0.0
ne = 0.0
e = 0.0
no = 0.0
o = 0.0
a = 0.0

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#INIZIO CLASSE

class LidarNode(Node):

    def __init__(self):

        super().__init__('driver')

#-------------------------------------------------------------------------------------------------------------------#

        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.listener_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 50)

        timer_period2 = 0.101

        self.timer2 = self.create_timer(timer_period2, self.timer_callback2)

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

    def listener_callback(self, msg):

        global n
        global ne
        global e
        global no
        global o


        
        # dist_back = format(msg.ranges[180], '.2f')
        # dist_left = format(msg.ranges[90], '.2f')
        # dist_right = format(msg.ranges[270], '.2f')
        # dist_head = format(msg.ranges[0], '.2f')
        # self.get_logger().info(f'{dist_back} {dist_left} {dist_right} {dist_head}')     

        n = msg.ranges[240]
        ne = msg.ranges[195] 
        e = msg.ranges[149] 
        no = msg.ranges[285]
        o = msg.ranges[331]

        # print("NORD:", n)
        # print("NORD EST:", ne)
        # print("EST:", e)
        # print("NORD OVEST:", no)
        # print("OVEST:", o)
        # print("-------------------------------------------------")



    def timer_callback2(self):

        global a

        a = Twist()

        a = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
         
# geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
        self.vel_pub.publish(a)

        

#-------------------------------------------------------------------------------------------------------------------#
        

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

def main(args=None):

    rclpy.init(args=args)

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    lidar_node.destroy_node()
    rclpy.shutdown()

