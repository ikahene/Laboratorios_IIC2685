
#!/usr/bin/env python3

import rclpy
import threading 
import time
from numpy import pi
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from tf_transformations import euler_from_quaternion


#Nodo principal
class Move_turtle( Node ):

  def __init__( self):
    super().__init__( 'dead_reckoning_nav' )
    #Nodo publica velocidades
    self.cmd_vel_mux_pub = self.create_publisher( Twist, '/cmd_vel', 10 )
    #Nodo escucha comandos
    self.poses_sub = self.create_subscription( PoseArray, 'goal_list', self.accion_mover_cb, 10 )
    #Guardamos la pose actual como la de inicio
    self.pose_actual = (1.0, 1.0, 0.0) #x, y, yaw
    #Velocidades constantes
    self.vel_lin = 0.2 #[m/s]
    self.vel_rot = 1.0 #[rad/s]

  #Recibe y aplica las velocidades al robot
  def aplicar_velocidad( self, speed_command_list : list):

    #Creamos el twist vacío
    speed = Twist()

    # Revisamos y ejecutamos cada comando en la lista
    for v, w, t in speed_command_list: 
      speed.linear.x = v
      speed.angular.z = w    
      #Consultamos tiempo actual del sistema 
      t_inicio = time.monotonic()  
      t_actual = t_inicio
      #Mandamos el mismo comando por t segundos
      while (t_actual - t_inicio) < t:
        t_actual = time.monotonic()
        self.cmd_vel_mux_pub.publish( speed ) #Publicamos las velocidades
        
  def comando_giro(self, yaw_desde, yaw_hacia):
    d = yaw_hacia - yaw_desde
    if d >= 0:
      w = self.vel_rot 
    else:
      w = -self.vel_rot
    return (0.0, w, abs(d) / self.vel_rot)

  def calcular_lista_velocidades(self, goal_pose):
    #Primero necesitamos conocer la distancias
    x_0, y_0, yaw_0 = self.pose_actual
    x_1, y_1, yaw_1 = goal_pose

    dis_horizontal = abs(x_1 - x_0)
    t_horizontal = dis_horizontal/self.vel_lin
    dis_vertical = abs(y_1 - y_0)
    t_vertical = dis_vertical/self.vel_lin
  
    #Primero nos movemos verticalmente 
    if y_0 < y_1: 
      #1. orientamos hacia arriba el robot
      c1 = self.comando_giro(y_0, pi/2)
      y_0 = pi/2
      
    else: 
      #1. orientamos hacia abajo el robot
      c1 = self.comando_giro(y_0, -pi/2)
      y_0 = -pi/2

    #2. Avanzamos 
    c2 = (self.vel_lin, 0.0, t_vertical)
    
    #Ahora nos movemos horizontalmente
    if x_0 < x_1:
      #3. Orientamos el robot hacia la derecha
      c3 = self.comando_giro(y_0, 0.0)
      y_0 = 0.0

    else:
      #3. Orientamos el robot hacia la izquierda
      c3 = self.comando_giro(y_0, pi)
      y_0 = pi

    #4. Avanzamos horizontalmente
    c4 = (self.vel_lin, 0.0, t_horizontal)

    #5. Dejamos el robot con su yaw final
    c5 = self.comando_giro(y_0, y_1)
    self.pose_actual = goal_pose

    return [c1, c2, c3, c4, c5]

  def mover_robot_a_destino(self, goal_pose):
    #Llamamos a la función para crear a lista
    speed_command_list = self.calcular_lista_velocidades(goal_pose)

    #Luego se itera sobre la lista 
    self.aplicar_velocidad(speed_command_list)

  def accion_mover_cb(self, msg):
    for goal_pose in msg:
      self.mover_robot_a_destino(goal_pose)

def main(args= None):
  rclpy.init()
  Nodo_principal = Move_turtle()
  Move_turtle.poses_sub()
  rclpy.spin(N)
    

if __name__ == '__main__':
  main()
