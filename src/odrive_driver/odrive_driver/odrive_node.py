import tf
import math
import time 
import rclpy
import numpy as np
import geometry_msgs
from odrive.enums import *
from rclpy.node import Node
from std_msgs.msg import Char
#from std_msgs.msg import time
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import String
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from .odrive_command import ODriveController
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import ParameterDescriptor


#VARIABILI GLOBALI

demandx = 0.0
demandz = 0.0
pos0 = 0.0
pos1 = 0.0
pos0_diff = 0.0
pos1_diff = 0.0
pos0_old = 0.0
pos1_old = 0.0
pos0_mm_diff = 0.0
pos1_mm_diff = 0.0
phi = 0.0
delta_th = 0.0

x = 0.0
y = 0.0
dth = 0.0

prev_update_time = time.time()
current_time = time.time()


#CONVERSIONE EULER A QUATERNION
 
def get_quaternion_from_euler(roll, pitch, yaw):
  
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  
 
    return [qw, qx, qy, qz]


#INIZIO CLASSE

class ODriveNode(Node):

   

   

    def __init__(self, odrv0):

        super().__init__('driver')
        self.odrv0 = odrv0

        self.axis0_vel_pub = self.create_publisher(
            Float32, 'axis0_vel_pub', 50)
        self.axis1_vel_pub = self.create_publisher(
            Float32, 'axis1_vel_pub', 50)

        self.axis0_pos_pub = self.create_publisher(
            Float32, 'axis0_pos_pub', 50)
        self.axis1_pos_pub = self.create_publisher(
            Float32, 'axis1_pos_pub', 50)

        self.right_wheel_rpm = self.create_publisher(
            Int64, 'right_wheel_rpm', 10)
        self.left_wheel_rpm = self.create_publisher(
            Int64, 'left_wheel_rpm', 10)

        timer_period = 1 / 100
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.axis0_vel_sub = self.create_subscription(
            Float32, 'axis0_vel_sub', self.axis0_vel_callback, 50)
        self.axis1_vel_sub = self.create_subscription(
            Float32, 'axis1_vel_sub', self.axis1_vel_callback, 50)

        self.axis0_pos_sub = self.create_subscription(
            Float32, 'axis0_pos_sub', self.axis0_pos_callback, 50)
        self.axis1_pos_sub = self.create_subscription(
            Float32, 'axis1_pos_sub', self.axis1_pos_callback, 50)


        self.cmd_vel_sub = self.create_subscription(
           Twist, '/cmd_vel', self.cmd_callback, 50)  



    def timer_callback(self):


        global x
        global y
        global dth
        global prev_update_time
        global current_time
        global demandx
        global demandz
        global pos0
        global pos1
        global pos0_diff
        global pos1_diff
        global pos0_old
        global pos1_old
        global pos0_mm_diff
        global pos1_mm_diff
        global phi
        global delta_th

        #odometria = Int32()
        vel = Int32()
        msg3 = Int64()
        msg4 = Int64()
        msg = Float32()
        msg2 = Twist()
        #tempo = Time()
        msg.data = self.odrv0.get_velocity(0)
        self.axis0_vel_pub.publish(msg)

        msg.data = self.odrv0.get_velocity(1)
        self.axis1_vel_pub.publish(msg)

        msg.data = self.odrv0.get_position(0)
        pos0 = msg.data
        self.axis0_pos_pub.publish(msg)

        msg.data = self.odrv0.get_position(1)
        pos1 = msg.data
        self.axis1_pos_pub.publish(msg)


        msg3.data = int((self.odrv0.get_velocity(0))*60)
        self.right_wheel_rpm.publish(msg3)


        msg4.data = int((self.odrv0.get_velocity(1))*60)
        self.left_wheel_rpm.publish(msg4)


    def axis0_vel_callback(self, msg):
        #print(f'axis0 vel: {msg}')
        self.odrv0.command_velocity(0, msg.data)

    def axis1_vel_callback(self, msg):
        self.get_logger().info(f'axis1 vel: {msg}')
        self.odrv0.command_velocity(1, msg.data)

    def axis0_pos_callback(self, msg):
        self.get_logger().info(f'axis0 pos: {msg}')
        self.odrv0.command_position(0, msg.data)

    def axis1_pos_callback(self, msg):
        self.get_logger().info(f'axis1 pos: {msg}')
        self.odrv0.command_position(1, msg.data)

    def cmd_callback(self, msg2):

        global demandx
        global demandz

        #print('Received a /cmd_vel message!')
        #print(f'Linear Components: {msg2.linear.x}')
        demandx = msg2.linear.x
        demandz = msg2.angular.z
        #print(f'angular Components: {msg2.angular.z}')
        self.odrv0.command_velocity(0, msg2.linear.x + msg2.angular.z)
        self.odrv0.command_velocity(1, msg2.linear.x - msg2.angular.z)
        #self.odrv0.command_velocity(1, msg2.data)



def main(args=None):
    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()
