import rclpy
from rclpy.node import Node
from odrive.enums import *
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Char
import time 
from .odrive_command import ODriveController
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
from scipy.spatial.transform import Rotation as R
#import tensorflow as tf
import tf
import geometry_msgs
from geometry_msgs.msg import Quaternion




demandx = 0.0
demandz = 0.0
pos0 = 0.0
pos1 = 0.0

x = 0.0
y = 0.0
dth = 0.0

prev_update_time = time.time()
current_time = time.time()


import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  
 
    return [qw, qx, qy, qz]

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


        # Set default topic-name for publishing. Accessed via ROS Parameters...
        #self.declare_parameter( 'ros_topic_twist', 'twist/cmd_vel', ParameterDescriptor(description='ROS-topc name. Publish twist/velocity from Joystick [default "twist/cmd_vel"]') )
        #self.ROS_TOPIC_TWIST = self.get_parameter('ros_topic_twist').get_parameter_value().string_value

   
    




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


        # Set default topic-name for publishing. Accessed via ROS Parameters...
        #self.declare_parameter( 'ros_topic_twist', 'twist/cmd_vel', ParameterDescriptor(description='ROS-topc name. Publish twist/velocity from Joystick [default "twist/cmd_vel"]') )
        #self.ROS_TOPIC_TWIST = self.get_parameter('ros_topic_twist').get_parameter_value().string_value

        #odometria = Int32()
        vel = Int32()
        msg = Float32()
        msg2 = Twist()
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

        current_time = time.time()
        dt = (current_time - prev_update_time)
        delta_th = (demandz) * dt
        

        delta_x = pos0 * math.cos(delta_th) * dt
        delta_y = pos1 * math.sin(delta_th) * dt

        

        x += delta_x
        y += delta_y
        dth += delta_th

        #quat_tf = [0.0, 1.0, 0.0, 0.0]
        quat_tf = get_quaternion_from_euler(0,0,dth)
        msg_quat = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])

        self.odometry.pose.pose.position.x = x
        self.odometry.pose.pose.position.y = y
        self.odometry.pose.pose.position.z = 0.0
        self.odometry.pose.pose.orientation = msg_quat #get_quaternion_from_euler(0,0,dth) # roll,pitch,yaw
        print(get_quaternion_from_euler(0,0,dth))
        print(demandx)
        print(demandz)
        #self.odometry.pose.pose.orientation.y = 
        #self.odometry.pose.pose.orientation.z = 
        #self.odometry.pose.pose.orientation.w = 1.
        self.odometry.twist.twist.linear.x = demandx
        self.odometry.twist.twist.linear.y = 0.0
        self.odometry.twist.twist.linear.z = 0.0
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = demandz
        #self.odometry.header.stamp = current_time
        self.odom.publish(self.odometry)



            #self.odom.publish()


    def axis0_vel_callback(self, msg):
        print(f'axis0 vel: {msg}')
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

        print('Received a /cmd_vel message!')
        print(f'Linear Components: {msg2.linear.x}')
        demandx = msg2.linear.x
        demandz = msg2.angular.z
        print(f'angular Components: {msg2.angular.z}')
        self.odrv0.command_velocity(0, msg2.linear.x + msg2.angular.z)
        self.odrv0.command_velocity(1, msg2.linear.x - msg2.angular.z)
        #self.odrv0.command_velocity(1, msg2.data)


    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))



def main(args=None):
    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()
