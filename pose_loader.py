import rclpy
import os
from tf_transformations import quaternion_from_euler
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class Pose(Node):
  def __init__( self):
    super().__init__( 'pose_loader' )
    #Nodo comunica goal list
    self.goals_pub = self.create_publisher( PoseArray, 'goal_list', 10 )

  def determinar_ruta(self):
    ruta_default = "~/iic2685_ws/src/lab1_pkg/lab1_pkg/poses.txt"

    if os.path.exists(ruta_default):
      return ruta_default
    else:
      return self.get_logger().info( 'No existe la ruta' )


  def leer_txt(self):
    #Identificamos la ruta 
    msg_goal_list = PoseArray() 
    ruta = self.determinar_ruta()

    #Abrimos y comenzamos a leer 
    with open(ruta, 'r') as poses:
      for pose in poses:
        #Quitamos el salto de linea
        pose = pose.strip()

        #Asignamos los valores
        x = float(pose[0])
        y = float(pose[1])
        yaw = float(pose[2])

        P = Pose()

        #Mapeamos a las poses
        P.position.x = x
        P.position.y = y
        P.position.z = 0.0

        #Usamos la transformación de euler a quaternion para el yaw
        quaternion = quaternion_from_euler( 0.0, 0.0, yaw )
        P.orientation.x = quaternion[0]
        P.orientation.y = quaternion[1]
        P.orientation.z = quaternion[2]
        P.orientation.w = quaternion[3]

        #Añadimos la pose creada al array
        msg_goal_list.poses.append(P)
      
    #Finalmente publicamos la lista
    while self.goals_pub.get_subscription_count() == 0:
      self.get_logger().info( 'Esperando conexión' )

    self.goals_pub.publish( msg_goal_list )


def main(args=None):
  rclpy.init()
  Nodo_cliente = Pose()

  #LLamamos para leer el txt
  Nodo_cliente.leer_txt()
  rclpy.spin(Nodo_cliente)

  Nodo_cliente.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
