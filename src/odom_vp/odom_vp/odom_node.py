import rclpy
import tf
import math
import time
import numpy as np
import geometry_msgs
from odrive.enums import *
from rclpy.node import Node
from std_msgs.msg import Char
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

#-------------------------------------------------------------------------------------------------------------------#

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
angularz = 0.0
pos_total_mm = 0.0
pos_average_mm_diff = 0.0
theta = 0.0
TWO_PI = 0.0
x = 0.0
y = 0.0
dth = 0.0
c_time = 0.0
p_time = 0.0
loopTime = 10.0
prev_update_time = time.time()
current_time = time.time()

#-------------------------------------------------------------------------------------------------------------------#

def get_quaternion_from_euler(roll, pitch, yaw):

    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)


    return [qw, qx, qy, qz]

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#INIZIO CLASSE

class OdomNode(Node):

    def __init__(self):

        super().__init__('driver')

#-------------------------------------------------------------------------------------------------------------------#

        self.odom = self.create_publisher(
            Odometry, 'odom', 10)

        self.Joint_State = self.create_publisher(
            JointState, 'joint_states', 10)
        
        self.pos0_sub = self.create_subscription(
            Float32, 'pos0', self.pos0_callback, 50)
        
        self.pos1_sub = self.create_subscription(
            Float32, 'pos1', self.pos1_callback, 50)

        timer_period2 = 0.101 #calcolo odometria + jointstates

        self.timer2 = self.create_timer(timer_period2, self.timer_callback2)

#---------------------------------------------------------------------------------------------------------------------------------------------------------#


    def pos0_callback(self, msg):

        global pos0
        pos0 = msg.data

    def pos1_callback(self, msg):

        global pos1
        pos1 = msg.data
                                    

    def timer_callback2(self):

        global x
        global y
        global dth
        global prev_update_time
        global current_time
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
        global angularz
        global pos_total_mm
        global pos_average_mm_diff
        global theta
        global TWO_PI
        global c_time
        global p_time
        global loopTime

        joint_state_position = JointState()
        joint_state_velocity = JointState()
        joint_state_position.name = ["joint0", "joint1"]
        # joint_state_velocity.name = ["wheel_left_joint", "wheel_right_joint"]
        joint_state_position.position = [pos0,pos1]
        # joint_state_velocity.velocity = [vel0,vel1]
        joint_state_position.header.stamp = self.get_clock().now().to_msg()
        # joint_state_velocity.header.stamp = self.get_clock().now().to_msg()
        self.Joint_State.publish(joint_state_position)
        # self.Joint_State.publish(joint_state_velocity)

#-------------------------------------------------------------------------------------------------------------------#

        self.broadcaster = TransformBroadcaster(self, 10)
        self.odometry = Odometry()
        odom_trans = TransformStamped()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'

        pos0_diff = pos0 - pos0_old
        pos1_diff = pos1 - pos1_old
        pos0_old = pos0
        pos1_old = pos1

        pos0_mm_diff = (pos0_diff  / (1 / 519)) #1 impulso al giro per ogni giro di ruota (530mm)
        pos1_mm_diff = (pos1_diff  / (1 / 519))

        current_time = self.get_clock().now().to_msg()

        phi = ((pos1_mm_diff - pos0_mm_diff) / 360) #angolo in radianti
        delta_th += phi

        if delta_th >= (math.pi)*2 :
            delta_th -= (math.pi)*2

        if delta_th <= (-math.pi)*2 :
            delta_th += (math.pi)*2

        x += ((pos0_mm_diff + pos1_mm_diff) / 2) * math.cos(delta_th)
        y += ((pos0_mm_diff + pos1_mm_diff) / 2) * math.sin(delta_th)

        quat_tf = get_quaternion_from_euler(0,0,delta_th)
        msg_quat = Quaternion(w=quat_tf[0], x=quat_tf[1], y=quat_tf[2], z=quat_tf[3])

        self.odometry.pose.pose.position.x = x/1000
        self.odometry.pose.pose.position.y = y/1000
        self.odometry.pose.pose.position.z = 0.0
        self.odometry.pose.pose.orientation = msg_quat
        self.odometry.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) /2)/10 #lasciare 10 con ciclo ogni 10ms
        self.odometry.twist.twist.linear.y = 0.0
        self.odometry.twist.twist.linear.z = 0.0
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) /6.28)*100
        self.odometry.header.stamp = current_time
        self.odom.publish(self.odometry)

        odom_trans.header.stamp = current_time
        odom_trans.transform.translation.x = self.odometry.pose.pose.position.x
        odom_trans.transform.translation.y = self.odometry.pose.pose.position.y
        odom_trans.transform.translation.z = self.odometry.pose.pose.position.z
        odom_trans.transform.rotation = self.odometry.pose.pose.orientation
        self.broadcaster.sendTransform(odom_trans)

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

def main(args=None):

    rclpy.init(args=args)

    odom_node = OdomNode()

    rclpy.spin(odom_node)

    odom_node.destroy_node()
    rclpy.shutdown()