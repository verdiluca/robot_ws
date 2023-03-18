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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .odrive_command import ODriveController
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped





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
angularz = 0.0
pos_total_mm = 0.0
pos_average_mm_diff = 0.0
theta = 0.0
TWO_PI = 0.0

x = 0.0
y = 0.0
dth = 0.0

prev_update_time = time.time()
current_time = time.time()



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

        self.right_wheel_rpm = self.create_publisher(
            Int64, 'right_wheel_rpm', 10)
        self.left_wheel_rpm = self.create_publisher(
            Int64, 'left_wheel_rpm', 10)

        self.odom = self.create_publisher(
            Odometry, 'odom', 5)


        self.Joint_State = self.create_publisher(
            JointState, 'joint_states', 10)

        timer_period = 1 / 100
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cmd_vel_sub = self.create_subscription(
           Twist, '/cmd_vel', self.cmd_callback, 10)

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
        global angularz
        global pos_total_mm
        global pos_average_mm_diff
        global theta
        global TWO_PI

        msg3 = Int64()
        msg4 = Int64()

        msg3.data = int((self.odrv0.get_velocity(0))*60)
        self.right_wheel_rpm.publish(msg3)

        msg4.data = int((self.odrv0.get_velocity(1))*60)
        self.left_wheel_rpm.publish(msg4)


        vel0 = Float32()
        vel1 = Float32()
        pos0 = Float32()
        pos1 = Float32()

        vel0 = self.odrv0.get_velocity(0)

        vel1 = self.odrv0.get_velocity(1)

        pos0 = (self.odrv0.get_position(0))*0.53 #numero del giro

        pos1 = (self.odrv0.get_position(1))*0.53

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

        ###################################################################################################

        vel = Int32()
        msg = Float32()
        msg2 = Twist()
        v_l = Float32()
        v_r = Float32()

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

        pos0_mm_diff = pos0_diff  #0.0058
        pos1_mm_diff = pos1_diff  #0.0058


        current_time = time.time()
        phi = ((pos1_mm_diff - pos0_mm_diff))
        dt = (current_time - prev_update_time)
        #delta_th = (demandz) * dt
        delta_th += phi

        #print(pos0)
        # print(pos1)
        #print(pos0_mm_diff)
        # print(pos1_mm_diff)

        if delta_th >= (math.pi)*2 :
            delta_th -= (math.pi)*2

        if delta_th <= (-math.pi)*2 :
            delta_th += (math.pi)*2

        delta_x = pos0_mm_diff * math.cos(delta_th) * dt
        delta_y = pos1_mm_diff * math.sin(delta_th) * dt

        x += delta_x
        y += delta_y
        dth = delta_th

        quat_tf = get_quaternion_from_euler(0,0,dth)
        msg_quat = Quaternion(w=quat_tf[0], x=quat_tf[1], y=quat_tf[2], z=quat_tf[3])


        self.odometry.pose.pose.position.x = x/1000
        self.odometry.pose.pose.position.y = y/1000
        self.odometry.pose.pose.position.z = 0.0
        self.odometry.pose.pose.orientation = msg_quat
        # self.odometry.pose.pose.orientation.y = 
        # self.odometry.pose.pose.orientation.z = 
        # self.odometry.pose.pose.orientation.w = 1.
        self.odometry.twist.twist.linear.x = ((pos0_mm_diff + pos1_mm_diff) /2)/10
        self.odometry.twist.twist.linear.y = 0.0
        self.odometry.twist.twist.linear.z = 0.0
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = ((pos1_mm_diff - pos0_mm_diff) /360)*100
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odom.publish(self.odometry)

        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.transform.translation.x = self.odometry.pose.pose.position.x
        odom_trans.transform.translation.y = self.odometry.pose.pose.position.y
        odom_trans.transform.translation.z = self.odometry.pose.pose.position.z
        odom_trans.transform.rotation = self.odometry.pose.pose.orientation
        self.broadcaster.sendTransform(odom_trans)


        ######################################################################################################à


    def cmd_callback(self, msg2):

        global angularz

        self.odrv0.command_velocity(0, msg2.linear.x + msg2.angular.z)
        self.odrv0.command_velocity(1, msg2.linear.x - msg2.angular.z)
        angularz = msg2.angular.z






def main(args=None):
    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()