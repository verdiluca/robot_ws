import rclpy
import time
import numpy as np
from odrive.enums import *
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from .odrive_command import ODriveController

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

class ODriveNode(Node):

    def __init__(self, odrv0):

        super().__init__('driver')
        self.odrv0 = odrv0

#-------------------------------------------------------------------------------------------------------------------#
    
        self.pos0_pub = self.create_publisher(
            Float32, 'pos0', 50)
        self.pos1_pub = self.create_publisher(
            Float32, 'pos1', 50)

        timer_period1 = 0.103 #lettura da odrive

        self.timer = self.create_timer(timer_period1, self.timer_callback)


        self.cmd_vel_sub = self.create_subscription(
           Twist, '/cmd_vel', self.cmd_callback, 10)

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

    def timer_callback(self):

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

#-------------------------------------------------------------------------------------------------------------------#

        vel0 = Float32()
        vel1 = Float32()
        pos0 = Float32()
        pos1 = Float32()
        vel = Int32()
        msg = Float32()
        msg2 = Twist()

#-------------------------------------------------------------------------------------------------------------------#


        vel0 = self.odrv0.get_velocity(0)

        vel1 = self.odrv0.get_velocity(1)

        pos1.data = (self.odrv0.get_position(0))
        self.pos1_pub.publish(pos1)

        pos0.data = (self.odrv0.get_position(1))
        self.pos0_pub.publish(pos0)

#-------------------------------------------------------------------------------------------------------------------#

    def cmd_callback(self, msg2):

        global angularz

        self.odrv0.command_velocity(0, msg2.linear.x + msg2.angular.z)
        self.odrv0.command_velocity(1, msg2.linear.x - msg2.angular.z)
        angularz = msg2.angular.z

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

def main(args=None):

    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()