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

        global pos0
        global pos1


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

        pos0 = (self.odrv0.get_position(0))

        pos1 = (self.odrv0.get_position(1))

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