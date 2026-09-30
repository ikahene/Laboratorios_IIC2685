#!/usr/bin/env python3

import rclpy
import os
import time
from tf_transformations import quaternion_from_euler
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class Client(Node):
  def __init__( self):
    super().__init__( 'pose_loader' )
    #El nodo publica la lista de poses hacia nuestro nodo principal
    self.goals_pub = self.create_publisher( PoseArray, 'goal_list', 10 )

    #Ruta 
    ruta_default = os.path.expanduser( '~/iic2685_ws/src/lab1/lab1/poses.txt' )
    self.declare_parameter( 'ruta', ruta_default )

  def determinar_ruta(self):
    #Verificar que todo esté bien con la ruta 
    ruta = os.path.expanduser( self.get_parameter( 'ruta' ).value )

    if os.path.exists(ruta):
      return ruta
    else:
      return None

  def leer_txt(self):
    #Creamos el pose array vacío e iniciamos la ruta
    msg_goal_list = PoseArray()
    ruta = self.determinar_ruta()

    #Abrimos y analizamos cada linea, transformándola en pose
    with open(ruta, 'r') as poses:
      for pose in poses:
        valores = pose.replace( ',', ' ' ).split()
        if len(valores) < 3:
          continue

        x = float(valores[0])
        y = float(valores[1])
        yaw = float(valores[2])

        P = Pose()
        P.position.x = x
        P.position.y = y
        P.position.z = 0.0

        #pasamos el yaw a quaternion para crear la pose
        quaternion = quaternion_from_euler( 0.0, 0.0, yaw )
        P.orientation.x = quaternion[0]
        P.orientation.y = quaternion[1]
        P.orientation.z = quaternion[2]
        P.orientation.w = quaternion[3]

        #añadimos la pose al array
        msg_goal_list.poses.append(P)

    #Agregamos esto, para sólo mandar una vez este el nav conectado
    while self.goals_pub.get_subscription_count() == 0:
      time.sleep( 0.5 )

    #Finalmente publicamos
    self.goals_pub.publish( msg_goal_list )


def main(args=None):
  rclpy.init()
  Nodo_cliente = Client()

  Nodo_cliente.leer_txt()
  rclpy.spin(Nodo_cliente)

  Nodo_cliente.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
