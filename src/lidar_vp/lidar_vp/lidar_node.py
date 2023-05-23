import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

#---------------------------------------------------------------DICHIARAZIONE_VARIABILI-------------------------------------------------------------------#
#sssss
#lidar
n = 0.0
ne = 0.0
e = 0.0
no = 0.0
o = 0.0
#ultrasuoni
sas = 0.0
sac = 0.0
sad = 0.0
scs = 0.0
scc = 0.0
scd = 0.0
sbs = 0.0
sbc = 0.0
sbd = 0.0

Media_O = 0.0
Media_E = 0.0

#motori
Motori_avanti = 0.0
Motori_destra = 0.0
Motori_sinistra = 0.0
Motori_fermo = 0.0

#fasi
logica_proto = 1
logica_azzera_variabili = 1
logica_controllo_sensori_partenza = 1
logica_proto_avanti = 1
logica_sterzo_dx = 1
logica_sterzo_sx = 1
logica_sterzo_av = 1
logica_arresto_proto = 1


#generali
Chiave = False
Stop = False

Azzeramento_completato = False

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
        
        self.nord_pub = self.create_publisher(
            Float32, 'nord', 50)
        self.nordest_pub = self.create_publisher(
            Float32, 'nordest', 50)
        self.nordovest_pub = self.create_publisher(
            Float32, 'nordovest', 50)
        self.ovest_pub = self.create_publisher(
            Float32, 'ovest', 50)
        self.est_pub = self.create_publisher(
            Float32, 'est', 50)

#--------------------------------------------------------------------AVVIO_VOID---------------------------------------------------------------------------#

        timer_period2 = 0.101
        self.timer2 = self.create_timer(timer_period2, self.Assegna_Uscite)

        timer_period3 = 0.103
        self.timer3 = self.create_timer(timer_period3, self.Logica_Proto)

        timer_period4 = 0.107
        self.timer4 = self.create_timer(timer_period4, self.Logica_Azzera_Variabili)

#---------------------------------------------------------------------------------------------------------------------------------------------------------#

    def listener_callback(self, msg):

        global n
        global ne
        global e
        global no
        global o
        global Media_E
        global Media_O
        global dist_back


        an =  Float32()
        ane =  Float32()
        ano =  Float32()
        ao =  Float32()
        ae =  Float32()

        an.data = msg.ranges[240]
        ano.data = msg.ranges[285]
        ane.data = msg.ranges[195]
        ao.data = msg.ranges[331]
        ae.data = msg.ranges[149]
        # dist_back = format(msg.ranges[240], '.3g')
        # dist_left = format(msg.ranges[90], '.2f')
        # dist_right = format(msg.ranges[270], '.2f')
        # dist_head = format(msg.ranges[0], '.2f')
        # self.get_logger().info(f'{dist_back} {dist_left} {dist_right} {dist_head}')
        
        n = Float32()
        e = Float32()
        o = Float32()
        no = Float32()
        ne = Float32()
        
        n = float(format(msg.ranges[240], '.2f'))
        ne = float(format(msg.ranges[195], '.2f'))
        e = float(format(msg.ranges[149], '.2f'))
        no = float(format(msg.ranges[285], '.2f'))
        o = float(format(msg.ranges[331], '.2f'))

        Media_E = (e+ne)/2
        Media_O = (o+no)/2

        
        self.nord_pub.publish(an)
        self.nordest_pub.publish(ane)
        self.nordovest_pub.publish(ano)
        self.ovest_pub.publish(ao)
        self.est_pub.publish(ae)


        # print("NORD:", n)
        # print("NORD EST:", ne)
        # print("EST:", e)
        # print("NORD OVEST:", no)
        # print("OVEST:", o)
        # print("-------------------------------------------------")

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-------------------------------------------------------------------ASSEGNA_USCITE-----------------------------------------------------------------------------#

    def Assegna_Uscite(self):
        
        #MOTORI
        global Motori_avanti
        global Motori_destra
        global Motori_sinistra
        global Motori_fermo
        global logica_proto

        Motori_avanti = Twist()
        Motori_destra = Twist()
        Motori_sinistra = Twist()
        Motori_fermo = Twist()

        Motori_avanti = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        Motori_destra = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        Motori_sinistra = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))
        Motori_fermo = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
         
        # geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
        
        #USCITE
        if logica_proto == 3:
            self.vel_pub.publish(Motori_avanti)
        
        if logica_proto == 4 or logica_proto == 8:
            self.vel_pub.publish(Motori_destra)
        
        if logica_proto == 5 or logica_proto == 7:
            self.vel_pub.publish(Motori_sinistra)
        
        if logica_proto == 6 or logica_proto == 9 or logica_proto == 1:
            self.vel_pub.publish(Motori_fermo)

        print(logica_proto)

        
#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------LOGICA_PROTO-----------------------------------------------------------------------------#
    def Logica_Proto(self):

        global n
        global ne
        global e
        global no
        global o
        global Media_E
        global Media_O
        global logica_proto
        global Stop
        global Chiave

        if logica_proto == 1 and Chiave == True:
            logica_proto = 2


        if logica_proto == 2 and logica_azzera_variabili == 3:
            logica_proto = 3


        if logica_proto == 3 and logica_controllo_sensori_partenza == 4:
            logica_proto = 4


        if logica_proto == 4 and (o < 0.35 or no < 0.35 or sas < 0.35 or scs < 0.35 or sbs < 0.35):
            logica_proto = 5
        
        elif logica_proto == 4 and (e < 0.35 or ne < 0.35 or sad < 0.35 or scd < 0.35 or sbd < 0.35):
            logica_proto = 6

        elif logica_proto == 4 and (n < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            logica_proto = 7


        if logica_proto == 5 and logica_sterzo_dx == 3 and n > 0.35 and ne > 0.35 and no > 0.35 and e > 0.35 and o > 0.35 and sas > 0.35 and scs > 0.35 and sbs > 0.35 and sac > 0.35 and scc > 0.35 and sbc > 0.35 and sad > 0.35 and scd > 0.35 and sbd > 0.35:
            logica_proto = 4
        
        elif logica_proto == 5 and logica_sterzo_dx == 3 and (e < 0.35 or ne < 0.35 or sad < 0.35 or scd < 0.35 or sbd < 0.35):
            logica_proto = 6
        
        elif logica_proto == 5 and logica_sterzo_dx == 3 and (n < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            logica_proto = 7
        

        if logica_proto == 6 and logica_sterzo_sx == 3 and n > 0.35 and ne > 0.35 and no > 0.35 and e > 0.35 and o > 0.35 and sas > 0.35 and scs > 0.35 and sbs > 0.35 and sac > 0.35 and scc > 0.35 and sbc > 0.35 and sad > 0.35 and scd > 0.35 and sbd > 0.35:
            logica_proto = 4
        
        elif logica_proto == 6 and logica_sterzo_sx == 3 and (o < 0.35 or no < 0.35 or sas < 0.35 or scs < 0.35 or sbs < 0.35):
            logica_proto = 5
        
        elif logica_proto == 6 and logica_sterzo_sx == 3 and (n < 0.35 or sac < 0.35 or scc < 0.35 or sbc < 0.35):
            logica_proto = 7


        if logica_proto == 7 and (Media_O > Media_E) and logica_sterzo_av == 3:
            logica_proto = 6
        if logica_proto == 7 and (Media_E > Media_O) and logica_sterzo_av == 3:
            logica_proto = 5


        if logica_proto == 8 and logica_arresto_proto == 3:
            logica_proto = 1


        if Stop == True:
            logica_proto = 8
    

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------LOGICA_AZZERA_VARIABILI------------------------------------------------------------------#

    def Logica_Azzera_Variabili(self):

        global Azzeramento_completato

        if logica_azzera_variabili == 1 and logica_proto == 2:
            logica_azzera_variabili = 2

        if logica_azzera_variabili == 2 and Azzeramento_completato == True:
            logica_azzera_variabili = 3
        
        if logica_azzera_variabili == 3 and logica_proto == 3:
            logica_azzera_variabili = 1

#--------------------------------------------------------------------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------LOGICA_CONTROLLO_SENSORI_PARTENZA------------------------------------------------------------------#











































def main(args=None):

    rclpy.init(args=args)

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    lidar_node.destroy_node()
    rclpy.shutdown()