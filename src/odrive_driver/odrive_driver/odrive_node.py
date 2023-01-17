import rclpy
from odrive.enums import *
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from .odrive_command import ODriveController

#INIZIO CLASSE

class ODriveNode(Node):

    def __init__(self, odrv0):

        super().__init__('driver')
        self.odrv0 = odrv0

        self.right_wheel_rpm = self.create_publisher(
            Int64, 'right_wheel_rpm', 10)
        self.left_wheel_rpm = self.create_publisher(
            Int64, 'left_wheel_rpm', 10)

        timer_period = 1 / 100
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cmd_vel_sub = self.create_subscription(
           Twist, '/cmd_vel', self.cmd_callback, 10)  

    def timer_callback(self):

        msg3 = Int64()
        msg4 = Int64()

        msg3.data = int((self.odrv0.get_velocity(0))*60)
        self.right_wheel_rpm.publish(msg3)

        msg4.data = int((self.odrv0.get_velocity(1))*60)
        self.left_wheel_rpm.publish(msg4)

    def cmd_callback(self, msg2):

        self.odrv0.command_velocity(0, msg2.linear.x + msg2.angular.z)
        self.odrv0.command_velocity(1, msg2.linear.x - msg2.angular.z)



def main(args=None):
    rclpy.init(args=args)

    odrv0 = ODriveController()

    odrive_node = ODriveNode(odrv0)
    odrv0.encoder_offset_calibration()
    odrv0.arm_velocity_control()

    rclpy.spin(odrive_node)

    odrive_node.destroy_node()
    rclpy.shutdown()s