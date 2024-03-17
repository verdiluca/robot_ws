import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
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
logica_sterzo_disincastro = 1
logica_sterzo_disincastro_SX = 1
logica_sterzo_disincastro_DX = 1

#generali
chiave = 0

motori_avviati = False
Proto_fermo = False
Proto_arrestato = False
Sterzo_DX_finito = False
Sterzo_SX_finito = False
Sterzo_disincastro_DX_finito = False
Sterzo_disincastro_SX_finito = False

Azzeramento_completato = False

#timers
Timer_T1_avviato = False
Timer_T1_concluso = False

Timer_T2_sterzo_iniziato = False
Timer_T3_sterzo_iniziato = False
Timer_T4_sterzo_iniziato = False
Timer_T5_sterzo_iniziato = False

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------INIZIO CLASSE------------------------------------------------------------------------------------------------#


class LidarNode(Node):

    def __init__(self):

        super().__init__('driver')

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------#

        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.listener_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

#---------------------------------------------------------------------------------------------------------------------------------------------------------#
#----------------------------------------------------------------------RICHIAMO_VARIABILI-----------------------------------------------------------------#

    def listener_callback(self, msg):

        global n
        global ne
        global e
        global no
        global o
        global Media_E
        global Media_O
        global dist_back



        
        n = Float32()

        n = float(format(msg.ranges[240] , '.2f'))

        



        print("NORD:", n)
        print("proto:", logica_proto)
        print("chiave:", chiave)
        print("-------------------------------------------------")



def main(args=None):

    rclpy.init(args=args)

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    lidar_node.destroy_node()
    rclpy.shutdown()