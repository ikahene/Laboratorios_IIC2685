import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class Pose(Node):
  def __init__( self):
    super().__init__( 'pose_loader' )
    #Nodo comunica goal list
    self.goals_pub = self.create_publisher( PoseArray, 'goal_list', 10 )

  def determinar_ruta(self):
    ruta_default = "/home/ikahene/iic2685_ws/src/lab1_pkg/lab1_pkg/poses.txt"
    #pedimos una ruta en caso de tener otro tipo de archivo
    print("Presiona ENTER sin escribir nada para usar la ruta por defecto.")
    respuesta = input("Ingresa la ruta absoluta del txt: ")

    #Si no se ingresa nada se usa el archivo local
    if respuesta == "":
      self.get_logger().info("Usando ruta por defecto...")
      ruta_final = ruta_default
    #Si se ingresa algo verificamos que exista y seguimos
    else:
      self.get_logger().info("Usando ruta ingresada por el usuario...")
      if os.path.exists(respuesta):
        ruta_final = respuesta
      else:
        ruta_final = ruta_default

    return ruta_final

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

        P.orientation.x = 0.0
        P.orientation.y = 0.0
        P.orientation.z = yaw
        P.orientation.w = 1.0

        #Añadimos la pose creada al array
        msg_goal_list.poses.append(P)
      
    #Finalmente publicamos la lista
    self.goals_pub.publish( msg_goal_list )


def main(args=None):
  rclpy.init()
  N = Client()

  #LLamamos para leer el txt
  N.leer_txt()
  rclpy.spin(N)


if __name__ == '__main__':
  main()
