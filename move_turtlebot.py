
#!/usr/bin/env python3

import rclpy
import threading 
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from tf_transformations import euler_from_quaternion


#Nodo principal
class Move_turtle( Node ):

    # t = d/v
  def __init__( self):
    super().__init__( 'dead reckoning nav' )
    #Nodo publica velocidades
    self.cmd_vel_mux_pub = self.create_publisher( Twist, '/cmd_vel', 10 )
    #Nodo escucha comandos
    self.poses_sub = self.create_subscription( PoseArray, 'goal_list', self.accion_mover_cb, 10 )
    #Velocidades constantes
    self.vel_lin = 0.2 #[m/s]
    self.vel_rot = 1 #[rad/s]
    #Timer con valores por defecto
    self.timer = self.create_timer(0.1, self.aplicar_velocidad)

    #Recibe y aplica las velocidades al robot
  def aplicar_velocidad( self, speed_command_list : list):
    #FALTARÍA PONER TODA LA LÓGICA DEL TIMER PARA QUE CADA ACCIÓN DIURE t
    speed = Twist()
    # Revisamos y ejecutamos cada comando
    for v, w, t in speed_command_list: 
      speed.linear.x = v
      speed.angular.z = w       
      self.cmd_vel_mux_pub.publish( speed ) #Publicamos las velocidades
        


  def mover_robot_a_destino(self, goal_pose):
    #RELLENAR AQUÍ LA LÓGICA PARA CREAR LA LISTA DE COMANDOS
    speed_command_list = None

    #Luego se itera sobre la lista 
    self.aplicar_velocidad(speed_command_list)

  def accion_mover_cb(self):
    pass


def main(args= None):
  rclpy.init()
  N = Move_turtle()
  #Aquí faltaría ir llamando los métods y etc
  rclpy.spin(N)
    

if __name__ == '__main__':

  main()
