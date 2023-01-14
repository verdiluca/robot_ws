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

        self.odom = self.create_publisher(
            Odometry, 'odom', 100)

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

        self.odometry = Odometry()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'

        pos0_diff = pos0 - pos0_old
        pos1_diff = pos1 - pos1_old
        pos0_old = pos0
        pos1_old = pos1

        pos0_mm_diff = pos0_diff / 0.0058
        pos1_mm_diff = pos1_diff / 0.0058


        current_time = time.time()
        phi = ((pos1_mm_diff - pos0_mm_diff) / 360)
        dt = (current_time - prev_update_time)
        #delta_th = (demandz) * dt
        delta_th += phi

        print(delta_th)

        if delta_th >= 6.28 :
            delta_th -= 6.28
            
        if delta_th <= (-6.28) :
            delta_th += 6.28

        delta_x = pos0 * math.cos(delta_th) #* dt
        delta_y = pos1 * math.sin(delta_th) #* dt

        x += delta_x
        y += delta_y
        dth += delta_th

        #quat_tf = [0.0, 1.0, 0.0, 0.0]
        quat_tf = get_quaternion_from_euler(0,0,dth)
        msg_quat = Quaternion(w=quat_tf[0], x=quat_tf[1], y=quat_tf[2], z=quat_tf[3])


        self.odometry.pose.pose.position.x = x/1000
        self.odometry.pose.pose.position.y = y/1000
        self.odometry.pose.pose.position.z = 0.0
        self.odometry.pose.pose.orientation = msg_quat
        #self.odometry.pose.pose.orientation.y = 
        #self.odometry.pose.pose.orientation.z = 
        #self.odometry.pose.pose.orientation.w = 1.
        self.odometry.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) /2)/10
        self.odometry.twist.twist.linear.y = 0.0
        self.odometry.twist.twist.linear.z = 0.0
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) /360)*100
        #self.odometry.header.stamp = current_time
        self.odom.publish(self.odometry)
        #self.odom.publish()


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
