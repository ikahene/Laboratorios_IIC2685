
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
      speed.linear.x = float(v)
      speed.angular.z = float(w)    
      #Consultamos tiempo actual del sistema 
      t_inicio = time.monotonic()  
      t_actual = t_inicio
      #Mandamos el mismo comando por t segundos
      while (t_actual - t_inicio) < t:
        self.cmd_vel_mux_pub.publish( speed ) #Publicamos las velocidades
        time.sleep(0.05)
        t_actual = time.monotonic()

    #Publicamos uno vacío para que deje de moverse
    self.cmd_vel_mux_pub.publish( Twist() )

  #Función auxiliar para crear los comandos con velocidad angular  
  def comando_giro(self, yaw_desde, yaw_hacia):
    d = yaw_hacia - yaw_desde
    #Buscamos el giro mas corto, para ello comparamos con pi
    if d > pi:
      d = d - 2*pi   
    elif d < -pi:
      d = d + 2*pi 

    #Definimos hacia qué lado girar
    if d >= 0:
      w = self.vel_rot #Antihorario
    else:
      w = -self.vel_rot
    return (0.0, w, abs(d) / self.vel_rot)

  #Función angular parea determinar los comandos a aplicar
  def calcular_lista_velocidades(self, goal_pose):
    lista_comandos = []

    #Primero necesitamos conocer la distancias
    x_0, y_0, yaw_0 = self.pose_actual
    x_1, y_1, yaw_1 = goal_pose

    yaw_actual = yaw_0
    dis_horizontal = abs(x_1 - x_0)
    t_horizontal = dis_horizontal/self.vel_lin
    dis_vertical = abs(y_1 - y_0)
    t_vertical = dis_vertical/self.vel_lin
  
    #Primero nos movemos verticalmente 
    if y_0 < y_1: 
      #1. orientamos hacia arriba el robot
      lista_comandos.append((self.comando_giro(yaw_actual, pi/2)))
      yaw_actual = pi/2
      
    elif y_0 > y_1: 
      #1. orientamos hacia abajo el robot
      lista_comandos.append((self.comando_giro(yaw_actual, -pi/2)))
      yaw_actual = -pi/2

    #2. Avanzamos 
    if dis_vertical != 0.0:
      lista_comandos.append(((self.vel_lin, 0.0, t_vertical)))
    
    #Ahora nos movemos horizontalmente
    if x_0 < x_1:
      #3. Orientamos el robot hacia la derecha
      lista_comandos.append((self.comando_giro(yaw_actual, 0.0)))
      yaw_actual = 0.0

    elif x_0 > x_1:
      #3. Orientamos el robot hacia la izquierda
      lista_comandos.append((self.comando_giro(yaw_actual, pi)))
      yaw_actual = pi

    #4. Avanzamos horizontalmente
    if dis_horizontal != 0.0:
      lista_comandos.append(((self.vel_lin, 0.0, t_horizontal)))

    #5. Dejamos el robot con su yaw final
    giro_final = self.comando_giro(yaw_actual, yaw_1)
    if (giro_final[2] != 0):
      lista_comandos.append(giro_final)

    return lista_comandos

  def mover_robot_a_destino(self, goal_pose):
    #Llamamos a la función para crear a lista
    speed_command_list = self.calcular_lista_velocidades(goal_pose)

    #Luego se itera sobre la lista 
    self.aplicar_velocidad(speed_command_list)

    #Asumimos que llegamos a destino
    self.pose_actual = goal_pose

    #Agregamos unas lineas para nos avise cuando llegó a destino
    x, y, yaw = goal_pose
    self.get_logger().info( f'Llegué a la pose estimada: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}' )

  def accion_mover_cb(self, msg):
    for goal_pose in msg.poses:
      #Extraemos de la pose lo que necesitamos
      x = goal_pose.position.x
      y = goal_pose.position.y
      quat = goal_pose.orientation
 
      #Pasamos el cuaternion a Euler 
      roll, pitch, yaw = euler_from_quaternion( [quat.x, 
                                                 quat.y, 
                                                 quat.z, 
                                                 quat.w] )


      self.mover_robot_a_destino((x, y, yaw))

def main(args= None):
  rclpy.init()

  Nodo_principal = Move_turtle()
  rclpy.spin(Nodo_principal)

  Nodo_principal.destroy_node()
  rclpy.shutdown()
    

if __name__ == '__main__':
  main()
